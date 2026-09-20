r"""Screw presenters, one per blow feeder (plan P4 B6).

    tools/.venv/bin/python tools/cad/screw_presenter.py [--out DIR] [--spec PATH] [--dev]

A blow feeder shoots screws down a tube (§16); the presenter is the block at the end of it that
holds one screw under every spindle of the gang head it feeds, so the head drives a whole pattern
in one stroke. The geometry is simple — a block with a bore per spindle — and **position is what
matters**: the bores are the gang head's own screw pattern, read through
`gang_heads.<head>.pattern_from`, so the presenter can never drift from the plate it screws into.

           feed tube ->o===[]===o<- feed tube        one port per bore, from the nearest face
                       |  | |   |
                       |  | |   |                    bore = screw + clearance, `length` deep
                       ====|=|====                   z = 0: the screw tips, facing the work

Everything is derived:

  * the bore pattern is `parts.<plate>.<field>.positions` — whatever `pattern_from` names — in the
    plate's own frame, so the report's positions are directly comparable with the plate's;
  * the block is as thick as the screw is long, plus its head and a lead for the bit, from
    `fasteners.<id>.length`;
  * `gang_heads.<head>.spindles` has to equal the number of holes in that pattern, and
    `gang_heads.<head>.stroke` has to be long enough to push a screw out of its bore and into the
    work. Either one disagreeing is the spec contradicting itself, so it is a FAIL.

Frame: the plate frame of the part `pattern_from` names (SolidWorks `CS_DATUM`), the same frame
the pallet and the carriers use, with z = 0 at the screw tips — the face the presenter turns
towards the work. The block is above it, the way the head sees it.

Only M3 is handled: minimizing distinct screw lengths is a line rule (§7.9) and the whole line is
M3, so the bore diameters below are M3's. A fastener that is not M3 is a FAIL that says so.

Outputs (`--out`, default `cad_out/`): `presenter_<feeder>.step` for review in SolidWorks and
`presenter_<feeder>.stl` in millimetres, through mesh_pipeline.py so the committed mesh obeys the
5 MB rule. Writes reports/cad_presenters.json with one entry per blow feeder.

Any spec value a presenter needs that is null (or absent) is a FAIL that names the key and the
feeder; `--dev` merges the placeholder overlay instead and writes to `<out>/dev/` and
reports/cad_presenters.dev.json, so a placeholder presenter can never be mistaken for a real one.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from build123d import Align, Box, Cylinder, Compound, Pos, Rot

sys.path.insert(0, str(Path(__file__).resolve().parent))         # tools/cad: common
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: spec

from common import (Values, bbox_doc, collect, export_solids, lookup, masses,  # noqa: E402
                    new_item, new_run_report, pascal, write_report)
from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, load_merged,  # noqa: E402
                  parse_spec, report_path)

DEFAULT_OUT = REPO / "cad_out"
REPORT = "cad_presenters"

# ---- fabrication choices, millimetres. Local to the presenters: nothing else depends on them. ----
BORE_D = 3.4          # M3 clearance: the screw slides, the thread does not bite
HEAD_D = 6.2          # M3 socket head Ø5.5 plus a clearance: the counterbore that centres the bit
HEAD_H = 3.0          # M3 socket head height
LEAD = 4.0            # bore above the head, so the bit enters square before it touches the screw
FEED_D = 4.2          # cross-drilled feed port: where the blow tube lands on each bore
MIN_WALL = 2.5        # least material between two counterbores
EDGE = 8.0            # block beyond the outermost bore, all round
DENSITY = 7850.0      # kg/m3, tool steel: the bits hit this block every cycle
MATERIAL = "hardened tool steel: a bit enters every bore every cycle (summary 16)"


# ---------------------------------------------------------------- layout

@dataclass
class Layout:
    """Everything one presenter's build needs, in millimetres, derived from the spec."""

    feeder: str
    head: str
    fastener: str
    plate: str
    pattern: str                            # the `pattern_from` string, for the report
    bores: list[tuple[float, float]]        # plate frame
    length: float                           # screw length
    spindles: int
    stroke: float
    direction: str

    @property
    def name(self) -> str:
        """`gh_arm_sandwich` -> `GangHead_ArmSandwich_Presenter` (CLAUDE.md conventions)."""
        return f"GangHead_{pascal(self.head.removeprefix('gh_'))}_Presenter"

    @property
    def thick(self) -> float:
        """Block thickness: the screw, its head, and a lead for the bit above it."""
        return self.length + HEAD_H + LEAD

    @property
    def span(self) -> tuple[float, float, float, float]:
        xs = [b[0] for b in self.bores]
        ys = [b[1] for b in self.bores]
        return min(xs), min(ys), max(xs), max(ys)

    @property
    def centre(self) -> tuple[float, float]:
        x0, y0, x1, y1 = self.span
        return (x0 + x1) / 2.0, (y0 + y1) / 2.0

    @property
    def size(self) -> tuple[float, float]:
        x0, y0, x1, y1 = self.span
        return x1 - x0 + 2.0 * EDGE, y1 - y0 + 2.0 * EDGE

    def closest_bores(self) -> tuple[float, int, int]:
        """(distance, i, j) of the two bores nearest each other — the tightest wall in the block."""
        best = (math.inf, 0, 0)
        for i, a in enumerate(self.bores):
            for j, b in enumerate(self.bores[i + 1:], i + 1):
                best = min(best, (math.dist(a, b), i, j))
        return best


def blow_feeders(raw: dict) -> list[str]:
    """Every feeder the spec says is blow-fed, in spec order."""
    feeders = raw.get("feeders") or {}
    return [fid for fid, f in feeders.items() if isinstance(f, dict) and f.get("type") == "blow"]


def pattern_positions(v: Values, head: str, pattern: str) -> list[tuple[float, float]] | None:
    """The screw pattern `gang_heads.<head>.pattern_from` names, in the plate's frame.

    `<part>.<field>` addresses either a list of positions or a `{per_arm, positions}` block; both
    are hole patterns on a plate, and the presenter only needs where the holes are.
    """
    part, _, field = pattern.partition(".")
    if not field:
        v.missing.append(f"gang_heads.{head}.pattern_from: '{pattern}' is not '<part>.<field>'")
        return None
    base = f"parts.{part}.{field}"
    node = lookup(v.raw, base)
    if isinstance(node, dict):                      # {per_arm, positions}, as on a plate
        base, node = f"{base}.positions", node.get("positions")
    if not isinstance(node, list) or not node:
        v.missing.append(f"{base} (named by gang_heads.{head}.pattern_from)")
        return None
    return [(v.mm(f"{base}[{i}][0]"), v.mm(f"{base}[{i}][1]")) for i in range(len(node))]


def layout(v: Values, fid: str) -> Layout | None:
    """Resolve one blow feeder into a Layout, or None when a value it needs is null."""
    fastener = v.plain(f"feeders.{fid}.part")
    head = v.plain(f"feeders.{fid}.target")
    if fastener is None or head is None:
        return None
    if lookup(v.raw, f"fasteners.{fastener}") is None:
        v.missing.append(f"fasteners.{fastener} (feeders.{fid}.part)")
        return None
    if lookup(v.raw, f"gang_heads.{head}") is None:
        v.missing.append(f"gang_heads.{head} (feeders.{fid}.target)")
        return None

    length = v.mm(f"fasteners.{fastener}.length")
    spindles = v.plain(f"gang_heads.{head}.spindles")
    stroke = v.mm(f"gang_heads.{head}.stroke")
    direction = v.plain(f"gang_heads.{head}.direction")
    pattern = v.plain(f"gang_heads.{head}.pattern_from")
    bores = pattern_positions(v, head, pattern) if pattern else None

    if v.missing or bores is None:
        return None
    return Layout(feeder=fid, head=head, fastener=fastener, plate=pattern.split(".")[0],
                  pattern=pattern, bores=bores, length=length, spindles=int(spindles),
                  stroke=stroke, direction=direction)


# ---------------------------------------------------------------- geometry

def feed_port(lo: Layout, bore: tuple[float, float]):
    """The blow tube's landing: a cross-drilling from the nearest face into the bore's head room."""
    x0, y0, x1, y1 = lo.span
    # Straight out of the face this bore is closest to, so no port crosses the block. Each
    # candidate is (how far that face is, which way the port runs from the bore to reach it).
    reach = [(bore[0] - (x0 - EDGE), (-1.0, 0.0)), ((x1 + EDGE) - bore[0], (1.0, 0.0)),
             (bore[1] - (y0 - EDGE), (0.0, -1.0)), ((y1 + EDGE) - bore[1], (0.0, 1.0))]
    depth, (dx, dy) = min(reach)
    # It runs from the face to the bore's axis, at the height of the screw head: that is where the
    # blow tube delivers, above the shank and below the bit's lead-in.
    mid = (bore[0] + dx * depth / 2.0, bore[1] + dy * depth / 2.0, lo.length + HEAD_H / 2.0)
    port = Cylinder(FEED_D / 2.0, depth, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return Pos(*mid) * (Rot(Y=90.0) if dx else Rot(X=90.0)) * port


def build(lo: Layout) -> Compound:
    """One block, bored and counterbored per spindle, with a feed port into each bore."""
    cx, cy = lo.centre
    block = Pos(cx, cy) * Box(*lo.size, lo.thick, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for bore in lo.bores:
        block -= Pos(*bore) * Cylinder(BORE_D / 2.0, lo.thick,
                                       align=(Align.CENTER, Align.CENTER, Align.MIN))
        block -= Pos(*bore, lo.length) * Cylinder(HEAD_D / 2.0, lo.thick - lo.length,
                                                  align=(Align.CENTER, Align.CENTER, Align.MIN))
        block -= feed_port(lo, bore)
    block.label = f"{lo.name}_Block"
    presenter = Compound(children=[block])
    # export_step writes the root label into the STEP header, so the material note travels with
    # the file into SolidWorks (plan P4 B6).
    presenter.label = f"{lo.name} [{MATERIAL}]"
    return presenter


# ---------------------------------------------------------------- report

def derived(lo: Layout) -> dict:
    """Positions another tool can check against the spec, all in millimetres."""
    gap, i, j = lo.closest_bores()
    cx, cy = lo.centre
    return {
        "frame": f"parts.{lo.plate} plate frame (CS_DATUM); z=0 at the screw tips",
        "model_name": lo.name,
        "head": lo.head,
        "fastener": lo.fastener,
        "pattern_from": lo.pattern,
        "spindles": lo.spindles,
        "bores_mm": [[round(c, 6) for c in b] for b in lo.bores],
        "bore_d_mm": BORE_D,
        "counterbore": {"d_mm": HEAD_D, "floor_mm": round(lo.length, 6)},
        "block_centre_mm": [round(cx, 6), round(cy, 6)],
        "block_size_mm": [round(lo.size[0], 6), round(lo.size[1], 6), round(lo.thick, 6)],
        "screw_length_mm": round(lo.length, 6),
        "closest_bores": {"pair": [i, j], "pitch_mm": round(gap, 6),
                          "wall_mm": round(gap - HEAD_D, 6)},
        "stroke_mm": round(lo.stroke, 6),
        "stroke_needed_mm": round(lo.thick + lo.length, 6),
        "feed_port_d_mm": FEED_D,
    }


def check(lo: Layout, item: dict) -> None:
    """Everything the presenter has to be true about, on top of building at all."""
    d = item["derived"]
    if not lo.fastener.lower().startswith("m3"):
        item["errors"].append(
            f"fasteners.{lo.fastener} is not an M3: this presenter is bored for M3 only, and the "
            "line is one thread (summary §7.9)")
    if lo.spindles != len(lo.bores):
        item["errors"].append(
            f"gang_heads.{lo.head}.spindles is {lo.spindles} but {lo.pattern} has "
            f"{len(lo.bores)} holes: the head cannot drive a pattern it has no spindle for")
    if d["closest_bores"]["wall_mm"] < MIN_WALL:
        i, j = d["closest_bores"]["pair"]
        item["errors"].append(
            f"bores {i} and {j} of {lo.pattern} are {d['closest_bores']['pitch_mm']:.2f} mm apart, "
            f"leaving {d['closest_bores']['wall_mm']:.2f} mm between their Ø{HEAD_D:g} mm "
            f"counterbores (min {MIN_WALL:g} mm): the block breaks through between them")
    if lo.stroke < d["stroke_needed_mm"]:
        item["errors"].append(
            f"gang_heads.{lo.head}.stroke is {lo.stroke:.1f} mm but the bit has to cross the "
            f"{lo.thick:.1f} mm block and drive a {lo.length:.1f} mm screw out of it "
            f"({d['stroke_needed_mm']:.1f} mm)")
    if lo.direction != "down":
        item["errors"].append(
            f"gang_heads.{lo.head}.direction is '{lo.direction}': this presenter holds screws by "
            "gravity and hands them down to the work")


# ---------------------------------------------------------------- CLI

def one(fid: str, raw: dict, overlay: set[str]) -> tuple[dict, Values, Compound | None]:
    """Resolve and build one presenter. Nothing is written yet: one bad feeder writes no files."""
    v = Values(raw, overlay)
    item = new_item(lookup(raw, f"feeders.{fid}.part") or "?")
    lo = layout(v, fid)
    if lo is None:
        item["missing"] = v.missing
        return item, v, None

    presenter = build(lo)
    item["bbox_mm"] = bbox_doc(presenter)
    item["mass_kg"] = masses(presenter, {}, default=DENSITY)
    item["derived"] = derived(lo)
    check(lo, item)
    return item, v, presenter


def unfed_fasteners(raw: dict, built: list[str]) -> list[str]:
    """Fasteners whose `feeder` is not a blow feeder the spec defines: nobody presents them."""
    out = []
    for fid, f in (raw.get("fasteners") or {}).items():
        feeder = f.get("feeder") if isinstance(f, dict) else None
        if feeder not in built:
            out.append(f"fasteners.{fid}.feeder is '{feeder}', which is not a blow feeder in the "
                       f"spec: no presenter is built for {fid}")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate one screw presenter per blow feeder")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help=f"default {DEFAULT_OUT}")
    ap.add_argument("--max-faces", type=int, default=50000)
    ap.add_argument("--bbox-tol-mm", type=float, default=0.2,
                    help="allowed difference between the STL bbox and the solid bbox")
    ap.add_argument("--stl-tolerance-mm", type=float, default=0.05,
                    help="STL tessellation tolerance")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    out_dir = args.out / "dev" if args.dev else args.out
    doc = new_run_report("tools/cad/screw_presenter.py", out_dir, args.dev)
    report = report_path(REPORT, args.dev)

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        write_report(report, doc)
        return code

    try:
        merged = load_merged(args.spec, args.dev)
        parse_spec(merged.raw)                      # schema first: a typo is not a missing value
    except SpecError as exc:
        doc["errors"] = exc.errors
        print(f"presenter: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)

    overlay = {p["path"] for p in merged.placeholders}
    feeders = blow_feeders(merged.raw)
    if not feeders:
        print("presenter: OK — no feeder of type 'blow' in the spec; nothing to build")
        return finish(0)

    built = []
    for fid in feeders:
        item, v, presenter = one(fid, merged.raw, overlay)
        collect(doc, fid, item, v)
        built.append((fid, item, presenter))
    doc["warnings"] += unfed_fasteners(merged.raw, feeders)

    # Every presenter is resolved before any is written, so a run that fails leaves no files
    # behind for the next reader to mistake for the current spec.
    if not doc["missing"] and not doc["errors"]:
        for fid, item, presenter in built:
            error = export_solids(item, presenter, out_dir, f"presenter_{fid}",
                                  stl_tolerance_mm=args.stl_tolerance_mm,
                                  max_faces=args.max_faces, bbox_tol_mm=args.bbox_tol_mm)
            if error:
                item["errors"].append(error)
                doc["errors"].append(f"{fid}: {error}")

    for m in doc["missing"]:
        print(f"presenter: FAIL — no value for {m}", file=sys.stderr)
    for w in doc["warnings"]:
        print(f"presenter: WARN {w}", file=sys.stderr)
    for e in doc["errors"]:
        print(f"presenter: FAIL — {e}", file=sys.stderr)
    if doc["missing"]:
        print("presenter: fill the key(s) in, or re-run with --dev to use the placeholder overlay",
              file=sys.stderr)
    if doc["missing"] or doc["errors"]:
        return finish(1)

    if args.dev:
        print(f"presenter: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{out_dir} is not a deliverable")
    for fid, item in doc["items"].items():
        d, size = item["derived"], item["bbox_mm"]["size"]
        print(f"presenter: OK — {fid} presents {d['spindles']} x {item['part']} to {d['head']}, "
              f"{size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm, "
              f"{item['mass_kg']['total']:.3f} kg "
              f"({item['mesh']['faces_after']} faces) -> {out_dir}")
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
