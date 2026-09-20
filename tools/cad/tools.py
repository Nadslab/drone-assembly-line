r"""Gantry end-effectors, one per `gantry.tools` entry (plan P4 B7).

    tools/.venv/bin/python tools/cad/tools.py [--out DIR] [--spec PATH] [--dev]

`gantry.tools.<id>.dims` is the envelope a tool needs at a pick feature — [x, y, reach] — and the
fixtures already size their pick features against it (tools/cad/gripper.py). This generator builds
the other half: the tool that envelope belongs to, in the two grips the spec allows.

    vacuum (plate tool)                     finger (carrier tool)
     ______________                          ______________
    |   manifold   |                        |     body     |
    |__|_|____|_|__|                        |  ||      ||  |       jaws reach `reach` down
       |_|    |_|     cups reach down       |  ||      ||  |       and close on a pick boss
    ====O======O====  to the pick plane     ==================      z = 0: the pick plane

Everything is derived:

  * which tools exist, and their footprint and reach, are `gantry.tools`;
  * a vacuum tool's cups are laid out inside the **smallest part that tool picks** — read off the
    `sequence` steps that name it — so the cups always land on the part and never overhang it;
  * a vacuum tool is checked against the **heaviest** part it picks: cup area x working vacuum,
    divided by a lifting safety factor, has to beat that part's weight, or it is a FAIL;
  * a finger tool reports the pick-boss diameters it can actually close on, which is the range the
    fixture generators (B3, B4) have to keep their bosses inside.

Frame: the tool's own frame — origin at the tool centre point, z = 0 at the pick plane (the face
that meets the part, or the plane the jaw tips reach). The tool is entirely above it, so a
generated model hangs off the gantry z axis with no offset.

Outputs (`--out`, default `cad_out/`): `<tool>.step` for review in SolidWorks and `<tool>.stl` in
millimetres, through mesh_pipeline.py so the committed mesh obeys the 5 MB rule. Writes
reports/cad_tools.json with one entry per tool.

Any spec value a tool needs that is null (or absent) is a FAIL that names the key and the tool;
`--dev` merges the placeholder overlay instead and writes to `<out>/dev/` and
reports/cad_tools.dev.json, so a placeholder tool can never be mistaken for a real one.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

from build123d import Align, Box, Compound, Cylinder, Pos

sys.path.insert(0, str(Path(__file__).resolve().parent))         # tools/cad: common
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: spec

from common import (Values, bbox_doc, collect, export_solids, lookup, masses,  # noqa: E402
                    new_item, new_run_report, pascal, write_report)
from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, load_merged,  # noqa: E402
                  parse_spec, report_path)

DEFAULT_OUT = REPO / "cad_out"
REPORT = "cad_tools"
G = 9.80665                   # m/s2

# ---- fabrication choices, millimetres unless said otherwise. Local to the end-effectors. ----
BODY_T = 18.0         # manifold / gripper body above the pick feature
FLANGE_D = 63.0       # mount flange to the gantry z axis (ISO 9409-1-50-4-M6 spigot)
FLANGE_T = 10.0
CUP_D = 25.0          # suction cup, bellows type
CUP_GAP = 8.0         # least gap between two cups
CUP_INSET = 10.0      # cup edge inside the part edge: a cup over the edge does not seal
STEM_D = 10.0         # cup stem between the manifold and the cup
JAW_T = 8.0           # finger thickness
MIN_BOSS_D = 10.0     # below this the jaws are gripping a pin, not a boss
GRIP_CLEAR = 2.0      # gap the jaws need around a boss, per side (matches the fixtures')
VACUUM_KPA = 60.0     # working vacuum a shop ejector holds, not the pump's peak
LIFT_SAFETY = 3.0     # vacuum lifting is a 3x business: seals leak and parts swing
DENSITY = 2710.0      # kg/m3, 6061-T6
MATERIAL = "6061-T6 aluminium: gantry end-effector (plan P4 B7)"


# ---------------------------------------------------------------- layout

@dataclass
class Handled:
    """A part the sequence picks with this tool."""

    part: str
    foot: tuple[float, float] | None
    mass: float | None                      # kg


@dataclass
class Layout:
    """Everything one tool's build needs, in millimetres, derived from the spec."""

    tool: str
    grip: str
    dims: tuple[float, float, float]        # x, y, reach
    handles: list[Handled] = field(default_factory=list)

    @property
    def name(self) -> str:
        """`tool_plate` -> `Tool_Plate` (CLAUDE.md naming conventions)."""
        return f"Tool_{pascal(self.tool.removeprefix('tool_'))}"

    @property
    def reach(self) -> float:
        return self.dims[2]

    @property
    def top(self) -> float:
        return self.reach + BODY_T + FLANGE_T

    @property
    def smallest(self) -> tuple[float, float]:
        """The footprint the cups have to stay inside: the tool's own, or a smaller part's."""
        foot = [self.dims[0], self.dims[1]]
        for h in self.handles:
            if h.foot:
                foot = [min(foot[i], h.foot[i]) for i in (0, 1)]
        return foot[0], foot[1]

    @property
    def heaviest(self) -> Handled | None:
        weighed = [h for h in self.handles if h.mass is not None]
        return max(weighed, key=lambda h: h.mass) if weighed else None

    def cup_axis(self, axis: int) -> list[float]:
        """Cup centres along one axis: two if they fit inside the footprint, else one on centre.

        `band` is what the cups may cover — the smallest part, less the inset that keeps a cup's
        edge off the part's edge — so two cups need room for both plus the gap between them.
        """
        band = self.smallest[axis] - 2.0 * CUP_INSET
        if band < 2.0 * CUP_D + CUP_GAP:
            return [0.0]
        offset = (band - CUP_D) / 2.0
        return [-offset, offset]

    def cups(self) -> list[tuple[float, float]]:
        return [(x, y) for x in self.cup_axis(0) for y in self.cup_axis(1)]

    @property
    def lift_n(self) -> float:
        """What the cups can actually hold, in newtons, after the lifting safety factor."""
        area = len(self.cups()) * 3.141592653589793 * (CUP_D / 2.0) ** 2   # mm2
        return area * (VACUUM_KPA / 1000.0) / LIFT_SAFETY                  # N/mm2 * mm2 = N

    @property
    def jaw_opening(self) -> float:
        """The widest boss the jaws close on: gripper.py's rule, boss + 2 x clearance <= jaw."""
        return min(self.dims[0], self.dims[1]) - 2.0 * GRIP_CLEAR


def handled_parts(v: Values, tid: str, grip: str) -> list[Handled]:
    """The parts the sequence picks with this tool, with what the tool needs to know about them.

    A vacuum tool has to hold the part up and seal on it, so its footprint and mass are read (and
    a null one is a FAIL, like any other value a generator needs). A finger tool grips a fixture's
    pick boss, which the fixture generator sizes — so nothing about the part is needed here.
    """
    out = []
    for step in v.raw.get("sequence") or []:
        if not isinstance(step, dict) or step.get("tool") != tid or not step.get("part"):
            continue
        pid = step["part"]
        if any(h.part == pid for h in out):
            continue
        if grip != "vacuum":
            out.append(Handled(pid, None, None))
            continue
        p = f"parts.{pid}"
        source = "envelope" if lookup(v.raw, f"{p}.envelope") is not None else "dims"
        foot = (v.mm(f"{p}.{source}[0]"), v.mm(f"{p}.{source}[1]"))
        mass = v.plain(f"{p}.mass")
        out.append(Handled(pid, foot, float(mass) if mass is not None else None))
    return out


def layout(v: Values, tid: str) -> Layout | None:
    """Resolve one tool into a Layout, or None when a value it needs is null."""
    grip = v.plain(f"gantry.tools.{tid}.grip")
    dims = tuple(v.mm(f"gantry.tools.{tid}.dims[{i}]") for i in range(3))
    handles = handled_parts(v, tid, grip) if grip else []
    if v.missing:
        return None
    return Layout(tool=tid, grip=grip, dims=dims, handles=handles)


# ---------------------------------------------------------------- geometry

def flange(lo: Layout):
    """The mount to the gantry z axis, on top of everything else."""
    disc = Pos(0.0, 0.0, lo.reach + BODY_T) * Cylinder(
        FLANGE_D / 2.0, FLANGE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    disc.label = f"{lo.name}_Flange"
    return disc


def vacuum_tool(lo: Layout) -> list:
    """A manifold on the flange, with a stem and a bellows cup down to the pick plane."""
    body = Pos(0.0, 0.0, lo.reach) * Box(lo.dims[0], lo.dims[1], BODY_T,
                                         align=(Align.CENTER, Align.CENTER, Align.MIN))
    body.label = f"{lo.name}_Manifold"
    cups = None
    for x, y in lo.cups():
        stem = Pos(x, y, CUP_D / 4.0) * Cylinder(STEM_D / 2.0, lo.reach - CUP_D / 4.0,
                                                 align=(Align.CENTER, Align.CENTER, Align.MIN))
        cup = Pos(x, y) * Cylinder(CUP_D / 2.0, CUP_D / 4.0,
                                   align=(Align.CENTER, Align.CENTER, Align.MIN))
        cups = (stem + cup) if cups is None else (cups + stem + cup)
    cups.label = f"{lo.name}_Cups"
    return [body, cups]


def finger_tool(lo: Layout) -> list:
    """A body on the flange, with two jaws reaching down to close across a pick boss."""
    body = Pos(0.0, 0.0, lo.reach) * Box(lo.dims[0], lo.dims[1], BODY_T,
                                         align=(Align.CENTER, Align.CENTER, Align.MIN))
    body.label = f"{lo.name}_Body"
    jaws = None
    for side in (-1.0, 1.0):
        jaw = Pos(side * (lo.dims[0] - JAW_T) / 2.0, 0.0) * Box(
            JAW_T, lo.dims[1], lo.reach, align=(Align.CENTER, Align.CENTER, Align.MIN))
        jaws = jaw if jaws is None else jaws + jaw
    jaws.label = f"{lo.name}_Jaws"
    return [body, jaws]


def build(lo: Layout) -> Compound:
    """Flange, body and grip, in the tool frame with z = 0 at the pick plane."""
    children = (vacuum_tool(lo) if lo.grip == "vacuum" else finger_tool(lo)) + [flange(lo)]
    tool = Compound(children=children)
    # export_step writes the root label into the STEP header, so the material note travels with
    # the file into SolidWorks (plan P4 B7).
    tool.label = f"{lo.name} [{MATERIAL}]"
    return tool


# ---------------------------------------------------------------- report

def derived(lo: Layout) -> dict:
    """Numbers another tool can check against the spec, in millimetres unless named otherwise."""
    doc = {
        "frame": f"gantry.tools.{lo.tool}; z=0 at the pick plane (the face that meets the part)",
        "model_name": lo.name,
        "grip": lo.grip,
        "dims_mm": [round(c, 6) for c in lo.dims],
        "height_mm": round(lo.top, 6),
        "handles": [h.part for h in lo.handles],
        "flange": {"d_mm": FLANGE_D, "z_mm": round(lo.reach + BODY_T, 6)},
    }
    if lo.grip == "vacuum":
        heaviest = lo.heaviest
        doc["cups"] = {"d_mm": CUP_D, "count": len(lo.cups()),
                       "centres_mm": [[round(c, 6) for c in p] for p in lo.cups()],
                       "inside_footprint_mm": [round(c, 6) for c in lo.smallest],
                       "inset_mm": CUP_INSET}
        doc["lift"] = {"vacuum_kpa": VACUUM_KPA, "safety": LIFT_SAFETY,
                       "capacity_n": round(lo.lift_n, 4),
                       "heaviest_part": heaviest.part if heaviest else None,
                       "heaviest_mass_kg": heaviest.mass if heaviest else None,
                       "demand_n": round(heaviest.mass * G, 4) if heaviest else 0.0}
    else:
        doc["jaws"] = {"thickness_mm": JAW_T, "reach_mm": round(lo.reach, 6),
                       "clearance_mm": GRIP_CLEAR,
                       "grips_boss_d_mm": [MIN_BOSS_D, round(lo.jaw_opening, 6)]}
    return doc


def check(lo: Layout, item: dict) -> None:
    """Everything the tool has to be true about, on top of building at all."""
    d = item["derived"]
    if not lo.handles:
        item["warnings"].append(
            f"no sequence step picks anything with '{lo.tool}': it is sized by "
            f"gantry.tools.{lo.tool}.dims alone")
    if lo.grip == "vacuum":
        if min(lo.smallest) - 2.0 * CUP_INSET < CUP_D:
            smallest = min(lo.handles, key=lambda h: min(h.foot) if h.foot else 1e9, default=None)
            item["errors"].append(
                f"a Ø{CUP_D:g} mm cup inset {CUP_INSET:g} mm does not fit on "
                f"{lo.smallest[0]:.1f} x {lo.smallest[1]:.1f} mm"
                + (f" ({smallest.part})" if smallest and smallest.foot else "")
                + ": the cup would hang over the edge and never seal")
        elif d["lift"]["demand_n"] > d["lift"]["capacity_n"]:
            lift = d["lift"]
            item["errors"].append(
                f"{len(lo.cups())} x Ø{CUP_D:g} mm cup(s) hold {lift['capacity_n']:.1f} N at "
                f"{VACUUM_KPA:g} kPa with a {LIFT_SAFETY:g}x safety factor, but "
                f"parts.{lift['heaviest_part']}.mass is {lift['heaviest_mass_kg']} kg "
                f"({lift['demand_n']:.1f} N)")
    elif lo.jaw_opening < MIN_BOSS_D:
        item["errors"].append(
            f"gantry.tools.{lo.tool}.dims open {min(lo.dims[0], lo.dims[1]):.1f} mm, so with "
            f"{GRIP_CLEAR:g} mm of clearance per side the jaws close on a Ø{lo.jaw_opening:.1f} mm "
            f"boss: below Ø{MIN_BOSS_D:g} mm that is a pin, not a pick boss")


# ---------------------------------------------------------------- CLI

def one(tid: str, raw: dict, overlay: set[str]) -> tuple[dict, Values, Compound | None]:
    """Resolve and build one tool. Nothing is written yet: one bad tool writes no files."""
    v = Values(raw, overlay)
    item = new_item(tid)
    lo = layout(v, tid)
    if lo is None:
        item["missing"] = v.missing
        return item, v, None

    tool = build(lo)
    item["bbox_mm"] = bbox_doc(tool)
    item["mass_kg"] = masses(tool, {}, default=DENSITY)
    item["derived"] = derived(lo)
    check(lo, item)
    return item, v, tool


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate one end-effector per gantry tool")
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
    doc = new_run_report("tools/cad/tools.py", out_dir, args.dev)
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
        print(f"tools: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)

    overlay = {p["path"] for p in merged.placeholders}
    tools = list((merged.raw.get("gantry") or {}).get("tools") or {})
    if not tools:
        print("tools: OK — gantry.tools is empty; nothing to build")
        return finish(0)

    built = []
    for tid in tools:
        item, v, tool = one(tid, merged.raw, overlay)
        collect(doc, tid, item, v)
        built.append((tid, item, tool))

    # Every tool is resolved before any is written, so a run that fails leaves no files behind
    # for the next reader to mistake for the current spec.
    if not doc["missing"] and not doc["errors"]:
        for tid, item, tool in built:
            stem = tid if tid.startswith("tool_") else f"tool_{tid}"
            error = export_solids(item, tool, out_dir, stem,
                                  stl_tolerance_mm=args.stl_tolerance_mm,
                                  max_faces=args.max_faces, bbox_tol_mm=args.bbox_tol_mm)
            if error:
                item["errors"].append(error)
                doc["errors"].append(f"{tid}: {error}")

    for m in doc["missing"]:
        print(f"tools: FAIL — no value for {m}", file=sys.stderr)
    for w in doc["warnings"]:
        print(f"tools: WARN {w}", file=sys.stderr)
    for e in doc["errors"]:
        print(f"tools: FAIL — {e}", file=sys.stderr)
    if doc["missing"]:
        print("tools: fill the key(s) in, or re-run with --dev to use the placeholder overlay",
              file=sys.stderr)
    if doc["missing"] or doc["errors"]:
        return finish(1)

    if args.dev:
        print(f"tools: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{out_dir} is not a deliverable")
    for tid, item in doc["items"].items():
        d, size = item["derived"], item["bbox_mm"]["size"]
        grip = (f"{d['cups']['count']} cup(s), {d['lift']['capacity_n']:.0f} N"
                if d["grip"] == "vacuum"
                else f"jaws close on Ø{d['jaws']['grips_boss_d_mm'][1]:.0f} mm")
        print(f"tools: OK — {tid} ({d['grip']}, {grip}), "
              f"{size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm, "
              f"{item['mass_kg']['total']:.3f} kg "
              f"({item['mesh']['faces_after']} faces) -> {out_dir}")
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
