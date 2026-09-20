r"""Magazines, one per `feeders` entry of type `magazine` (plan P4 B5).

    tools/.venv/bin/python tools/cad/magazine.py [--out DIR] [--spec PATH] [--dev]

One generator, parameterized: every magazine is the same tube around a different part, so the
plates, the side plates and anything else a magazine ever feeds come out of this file. Nothing
about which magazines exist is typed here — the spec's `feeders` decides, and each one reads only
its own part (§16: "Large, asymmetric or fragile -> tray or magazine, oriented by the container,
not the robot").

          rim = pick plane, z = 0      the gantry always picks here, however full the magazine is
        ==|===================|==
          |  [ part       ]   |        stack of `capacity` parts, `SEPARATION` apart
          |  [ part       ]   |
          |  [ follower   ]   |        spring follower pushes the stack up to the rim
          |___________________|        base

Everything is derived:

  * the pocket is the part's envelope plus a fit clearance, so the tube tracks the part;
  * the tube is `capacity` parts deep — the spec's own number for how many it holds;
  * the **pick plane is the rim**, so the tool never reaches inside the tube and the gantry goes to
    one fixed coordinate every cycle (§16), which is what `feeders.<id>.pose` means;
  * the poka-yoke key is `parts.<part>.key`, the corner cut at the part's +x,+y corner (§7.5). The
    magazine grows a rib in that corner, and the generator **proves** it works: the part is seated
    both ways round and the mirrored one has to clash with the rib. A key that a mirrored part
    clears is a FAIL, not a drawing.

Frame: the feeder's own frame — origin at `feeders.<id>.pose`, z = 0 at the pick plane (the top
face of the presented part), so the Gazebo model drops in at the feeder pose with no offset, and
everything the magazine needs below the pick plane is a negative z.

Outputs (`--out`, default `cad_out/`): `magazine_<feeder>.step` for review in SolidWorks and
`magazine_<feeder>.stl` in millimetres, through mesh_pipeline.py so the committed mesh obeys the
5 MB rule. Writes reports/cad_magazines.json with one entry per feeder: bbox, mass, and every
derived position.

Any spec value a magazine needs that is null (or absent) is a FAIL that names the key and the
feeder; `--dev` merges the placeholder overlay instead and writes to `<out>/dev/` and
reports/cad_magazines.dev.json, so a placeholder magazine can never be mistaken for a real one.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from build123d import Align, Box, Compound, Pos

sys.path.insert(0, str(Path(__file__).resolve().parent))         # tools/cad: common
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: spec

from common import (Values, bbox_doc, collect, export_solids, lookup, masses,  # noqa: E402
                    new_item, new_run_report, pascal, write_report)
from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, load_merged,  # noqa: E402
                  parse_spec, report_path)

DEFAULT_OUT = REPO / "cad_out"
REPORT = "cad_magazines"

# ---- fabrication choices, millimetres. Local to the magazines: nothing else depends on them. ----
FIT = 1.0             # pocket beyond the part footprint, per side: the stack must slide freely
WALL = 6.0            # tube wall outside that clearance
BASE_T = 8.0          # plate under the follower's lowest position
FOLLOWER_T = 10.0     # spring follower that holds the top part at the rim
SEPARATION = 2.0      # gap between two stacked parts (§16: flat plates stick to each other)
KEY_CLEAR = 0.3       # rib under the part's key cut, per side (drop-in, not a press fit)
WINDOW_W = 0.5        # level window in the -y wall, as a fraction of the pocket width
WINDOW_MARGIN = 12.0  # material left above and below the window
TOOL_CLEAR = 1.0      # gap a tool that reaches into the pocket needs, per side
MIN_KEY = 1.5         # smallest corner cut that can stop a part rather than shave it
DENSITY = 2710.0      # kg/m3, 6061-T6
MATERIAL = "6061-T6 aluminium: welded tube, spring follower (summary 16)"


# ---------------------------------------------------------------- layout

@dataclass
class Layout:
    """Everything one magazine's build needs, in millimetres, derived from the spec."""

    feeder: str
    part: str
    foot: tuple[float, float]       # part footprint x, y
    thick: float                    # part thickness: the stack pitch is this plus SEPARATION
    capacity: int
    pose: tuple[float, float, float]
    key: tuple[float, float] | None
    tool: str | None
    grip: str | None
    jaw: tuple[float, float, float]

    @property
    def name(self) -> str:
        """`fd_mid_plate` -> `Feeder_MidPlate` (CLAUDE.md naming conventions)."""
        return f"Feeder_{pascal(self.feeder.removeprefix('fd_'))}"

    @property
    def pitch(self) -> float:
        return self.thick + SEPARATION

    @property
    def stack(self) -> float:
        """Depth of pocket the parts themselves occupy, below the rim."""
        return self.capacity * self.pitch

    @property
    def pocket(self) -> tuple[float, float]:
        return self.foot[0] + 2.0 * FIT, self.foot[1] + 2.0 * FIT

    @property
    def size(self) -> tuple[float, float]:
        return self.pocket[0] + 2.0 * WALL, self.pocket[1] + 2.0 * WALL

    @property
    def pocket_floor(self) -> float:
        """Bottom of the pocket: the stack plus the follower under it, all below the rim."""
        return -(self.stack + FOLLOWER_T)

    @property
    def base(self) -> float:
        return self.pocket_floor - BASE_T

    @property
    def base_z_line(self) -> float:
        """Where the magazine's underside lands in the line frame: it may not go below the floor."""
        return self.pose[2] + self.base


def magazine_feeders(raw: dict) -> list[str]:
    """Every feeder the spec says is a magazine, in spec order."""
    feeders = raw.get("feeders") or {}
    return [fid for fid, f in feeders.items()
            if isinstance(f, dict) and f.get("type") == "magazine"]


def picking_tool(raw: dict, fid: str) -> str | None:
    """The gantry tool the sequence takes parts out of this feeder with, if it names one.

    Optional: a step may draw from a feeder without a tool (a gang head fits the part instead),
    and then there is no tool to leave room for.
    """
    for step in raw.get("sequence") or []:
        if isinstance(step, dict) and step.get("from") == fid and step.get("tool"):
            return step["tool"]
    return None


def layout(v: Values, fid: str) -> Layout | None:
    """Resolve one feeder into a Layout, or None when a value that magazine needs is null."""
    part = v.plain(f"feeders.{fid}.part")
    if part is None:
        return None
    p = f"parts.{part}"
    envelope = lookup(v.raw, f"{p}.envelope")
    dims = lookup(v.raw, f"{p}.dims")
    if envelope is not None:
        foot = (v.mm(f"{p}.envelope[0]"), v.mm(f"{p}.envelope[1]"))
        thick = v.mm(f"{p}.envelope[2]")
    elif dims is not None and lookup(v.raw, f"{p}.shape") == "box":
        foot = (v.mm(f"{p}.dims[0]"), v.mm(f"{p}.dims[1]"))
        thick = v.mm(f"{p}.dims[2]")
    else:
        v.missing.append(f"{p}.envelope (a magazine stacks a flat, rectangular part)")
        return None

    capacity = v.plain(f"feeders.{fid}.capacity")
    pose = tuple(v.mm(f"feeders.{fid}.pose[{i}]") for i in range(3))

    key = None
    if lookup(v.raw, f"{p}.key") is not None:
        key = (v.mm(f"{p}.key[0]"), v.mm(f"{p}.key[1]"))

    tool = picking_tool(v.raw, fid)
    jaw = tuple(v.mm(f"gantry.tools.{tool}.dims[{i}]") for i in range(3)) if tool else (0.0, 0.0,
                                                                                        0.0)
    grip = v.plain(f"gantry.tools.{tool}.grip") if tool else None

    if v.missing:
        return None
    return Layout(feeder=fid, part=part, foot=foot, thick=thick, capacity=int(capacity), pose=pose,
                  key=key, tool=tool, grip=grip, jaw=jaw)


# ---------------------------------------------------------------- geometry

def key_rib(lo: Layout):
    """The rib filling the part's key corner, running the full pocket: a flipped part hits it.

    It reaches from the pocket's +x,+y corner as deep as the part's key cut less a clearance, so a
    correctly oriented part drops past it and a mirrored one — whose corner is solid there —
    cannot.
    """
    kx, ky = lo.key
    rib = Pos(lo.pocket[0] / 2.0, lo.pocket[1] / 2.0) * Box(
        kx + FIT - KEY_CLEAR, ky + FIT - KEY_CLEAR, -lo.pocket_floor,
        align=(Align.MAX, Align.MAX, Align.MAX))
    rib.label = f"{lo.name}_Key"
    return rib


def level_window(lo: Layout):
    """Slot in the -y wall: the operator sees how many parts are left without opening anything."""
    height = -lo.pocket_floor - 2.0 * WINDOW_MARGIN
    if height <= 0.0:
        return None
    return Pos(0.0, -lo.pocket[1] / 2.0, -WINDOW_MARGIN) * Box(
        WINDOW_W * lo.pocket[0], 2.0 * WALL, height, align=(Align.CENTER, Align.CENTER, Align.MAX))


def tube(lo: Layout):
    """Walls and base around the stack, open at the top so the rim presents the part."""
    body = Box(*lo.size, -lo.base, align=(Align.CENTER, Align.CENTER, Align.MAX))
    body -= Box(*lo.pocket, -lo.pocket_floor, align=(Align.CENTER, Align.CENTER, Align.MAX))
    window = level_window(lo)
    if window is not None:
        body -= window
    body.label = f"{lo.name}_Tube"
    return body


def follower(lo: Layout):
    """The spring follower, drawn at the bottom of its travel: a full magazine is the worst case.

    It is notched around the key rib, so the rib doubles as the follower's anti-rotation guide.
    """
    plate = Pos(0.0, 0.0, lo.pocket_floor) * Box(
        lo.pocket[0] - 2.0 * KEY_CLEAR, lo.pocket[1] - 2.0 * KEY_CLEAR, FOLLOWER_T,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    if lo.key:
        plate -= key_rib(lo)
    plate.label = f"{lo.name}_Follower"
    return plate


def build(lo: Layout) -> Compound:
    """Tube, follower and (when the part is keyed) the rib, in the feeder frame."""
    children = [tube(lo), follower(lo)]
    if lo.key:
        children.append(key_rib(lo))
    mag = Compound(children=children)
    # export_step writes the root label into the STEP header, so the material note travels with
    # the file into SolidWorks (plan P4 B5).
    mag.label = f"{lo.name} [{MATERIAL}]"
    return mag


def seated_part(lo: Layout, mirrored: bool):
    """The presented part lying at the rim, with its key corner cut away.

    `mirrored` flips it about the y axis — the way a side plate goes in inside-out (§16). The key
    exists to make that impossible, so the generator checks it rather than trusting the drawing.
    """
    part = Box(*lo.foot, lo.thick, align=(Align.CENTER, Align.CENTER, Align.MAX))
    if lo.key:
        sign = -1.0 if mirrored else 1.0
        kx, ky = lo.key
        part -= Pos(sign * lo.foot[0] / 2.0, lo.foot[1] / 2.0) * Box(
            2.0 * kx, 2.0 * ky, 2.0 * lo.thick, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return part


def key_clash(lo: Layout, mirrored: bool) -> float:
    """Material the seated part and the key rib want in the same place, in mm3."""
    if not lo.key:
        return 0.0
    return (seated_part(lo, mirrored) & key_rib(lo)).volume


def tool_clash(lo: Layout, mag: Compound) -> float:
    """Magazine material inside the volume a finger tool sweeps when it reaches into the pocket.

    A vacuum tool lands on the part at the rim and never enters, so it sweeps nothing; a finger
    tool comes down `reach` past the pick plane to get around the part's edges.
    """
    if lo.grip != "finger":
        return 0.0
    swept = Box(lo.jaw[0] + 2.0 * TOOL_CLEAR, lo.jaw[1] + 2.0 * TOOL_CLEAR, lo.jaw[2],
                align=(Align.CENTER, Align.CENTER, Align.MAX))
    return sum((child & swept).volume for child in mag.children)


# ---------------------------------------------------------------- report

def derived(lo: Layout, mag: Compound) -> dict:
    """Positions another tool can check against the spec, all in millimetres."""
    doc = {
        "frame": f"feeders.{lo.feeder}.pose; z=0 at the pick plane "
                 "(top face of the presented part)",
        "model_name": lo.name,
        "part": lo.part,
        "part_footprint_mm": [round(c, 6) for c in lo.foot],
        "part_thickness_mm": round(lo.thick, 6),
        "capacity": lo.capacity,
        "stack": {"pitch_mm": round(lo.pitch, 6), "depth_mm": round(lo.stack, 6),
                  "separation_mm": SEPARATION},
        "pocket_mm": [round(c, 6) for c in lo.pocket],
        "tube_size_mm": [round(lo.size[0], 6), round(lo.size[1], 6), round(-lo.base, 6)],
        "rim_z_mm": 0.0,
        "pocket_floor_mm": round(lo.pocket_floor, 6),
        "base_z_mm": round(lo.base, 6),
        "pose_mm": [round(c, 6) for c in lo.pose],
        "base_z_line_mm": round(lo.base_z_line, 6),
        "follower": {"thickness_mm": FOLLOWER_T, "travel_mm": round(lo.stack, 6)},
        "key": None,
        "tool": None,
    }
    if lo.key:
        doc["key"] = {"corner": "+x+y", "size_mm": [round(c, 6) for c in lo.key],
                      "clearance_mm": KEY_CLEAR,
                      "seated_clash_mm3": round(key_clash(lo, mirrored=False), 4),
                      "flipped_clash_mm3": round(key_clash(lo, mirrored=True), 4)}
    if lo.tool:
        doc["tool"] = {"id": lo.tool, "grip": lo.grip,
                       "reach_mm": round(lo.jaw[2], 6),
                       "keep_out_volume_mm3": round(tool_clash(lo, mag), 4)}
    return doc


def check(lo: Layout, item: dict, v: Values) -> None:
    """Everything the magazine has to be true about, on top of building at all."""
    d = item["derived"]
    if lo.base_z_line < 0.0:
        path = f"feeders.{lo.feeder}.pose[2]"
        message = (f"{path} presents {lo.part} {lo.pose[2]:.1f} mm off the floor, but "
                   f"{lo.capacity} parts plus the follower and base need "
                   f"{-lo.base:.1f} mm under the pick plane: the magazine would stand "
                   f"{-lo.base_z_line:.1f} mm below the floor. Raise the pose or cut the capacity")
        # A placeholder pose has nothing to be right about (CLAUDE.md: no deliverable number comes
        # from the overlay), so it warns where a real pose is a contradiction in the spec.
        (item["warnings"] if v.is_placeholder(path) else item["errors"]).append(message)

    if lo.key is None:
        item["warnings"].append(
            f"parts.{lo.part}.key is not set: nothing in the magazine tells {lo.part} from its "
            "mirror image, so an inside-out part can seat (summary §7.5)")
    else:
        if min(lo.key) <= MIN_KEY:
            item["errors"].append(
                f"parts.{lo.part}.key is {lo.key[0]:.1f} x {lo.key[1]:.1f} mm: below "
                f"{MIN_KEY:g} mm the rib shaves the part instead of stopping it")
        elif d["key"]["flipped_clash_mm3"] <= 0.0:
            item["errors"].append(
                f"parts.{lo.part}.key does not key anything: a mirrored {lo.part} clears the rib "
                "and seats in the magazine")
        if d["key"]["seated_clash_mm3"] > 0.0:
            item["errors"].append(
                f"the key rib fouls a correctly oriented {lo.part} by "
                f"{d['key']['seated_clash_mm3']:.1f} mm3")

    if d["tool"] and d["tool"]["keep_out_volume_mm3"] > 0.0:
        item["errors"].append(
            f"tool '{lo.tool}' reaches {lo.jaw[2]:.1f} mm into the pocket and "
            f"{d['tool']['keep_out_volume_mm3']:.1f} mm3 of the magazine is in the way: widen "
            "the pocket or pick with a vacuum tool")


# ---------------------------------------------------------------- CLI

def one(fid: str, raw: dict, overlay: set[str]) -> tuple[dict, Values, Compound | None]:
    """Resolve and build one magazine. Nothing is written yet: one bad feeder writes no files."""
    v = Values(raw, overlay)
    part = lookup(raw, f"feeders.{fid}.part") or "?"
    item = new_item(part)
    lo = layout(v, fid)
    if lo is None:
        item["missing"] = v.missing
        return item, v, None

    mag = build(lo)
    item["bbox_mm"] = bbox_doc(mag)
    item["mass_kg"] = masses(mag, {}, default=DENSITY)
    item["derived"] = derived(lo, mag)
    check(lo, item, v)
    return item, v, mag


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate one magazine per magazine feeder")
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
    doc = new_run_report("tools/cad/magazine.py", out_dir, args.dev)
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
        print(f"magazine: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)

    overlay = {p["path"] for p in merged.placeholders}
    feeders = magazine_feeders(merged.raw)
    if not feeders:
        print("magazine: OK — no feeder of type 'magazine' in the spec; nothing to build")
        return finish(0)

    built = []
    for fid in feeders:
        item, v, mag = one(fid, merged.raw, overlay)
        collect(doc, fid, item, v)
        built.append((fid, item, mag))

    # Every magazine is resolved before any is written, so a run that fails leaves no files behind
    # for the next reader to mistake for the current spec.
    if not doc["missing"] and not doc["errors"]:
        for fid, item, mag in built:
            error = export_solids(item, mag, out_dir, f"magazine_{fid}",
                                  stl_tolerance_mm=args.stl_tolerance_mm,
                                  max_faces=args.max_faces, bbox_tol_mm=args.bbox_tol_mm)
            if error:
                item["errors"].append(error)
                doc["errors"].append(f"{fid}: {error}")

    for m in doc["missing"]:
        print(f"magazine: FAIL — no value for {m}", file=sys.stderr)
    for w in doc["warnings"]:
        print(f"magazine: WARN {w}", file=sys.stderr)
    for e in doc["errors"]:
        print(f"magazine: FAIL — {e}", file=sys.stderr)
    if doc["missing"]:
        print("magazine: fill the key(s) in, or re-run with --dev to use the placeholder overlay",
              file=sys.stderr)
    if doc["missing"] or doc["errors"]:
        return finish(1)

    if args.dev:
        print(f"magazine: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{out_dir} is not a deliverable")
    for fid, item in doc["items"].items():
        size = item["bbox_mm"]["size"]
        print(f"magazine: OK — {fid} holds {item['derived']['capacity']} x {item['part']}, "
              f"{size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm, "
              f"{item['mass_kg']['total']:.3f} kg "
              f"({item['mesh']['faces_after']} faces) -> {out_dir}")
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
