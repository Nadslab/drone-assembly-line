r"""Avionics-pod tray, generated from line_spec.yaml (plan P4 B4).

    tools/.venv/bin/python tools/cad/pod_tray.py [--out DIR] [--spec PATH] [--dev]

Sub-line B builds the avionics pod in this tray; the main line takes the tray off the buffer,
drops the pod onto the ESC (`place_pod`, summary §5 step 7) and lifts the tray away once the pod
screws are driven. Same pattern as the spider carrier, with one fewer problem to solve: nothing
has to pass through the tray while it is on the stack, so the pod hangs in a plain shroud instead
of from a bridge.

           ___________________________         roof, with the pick boss on top
          |  _______________________  |
          | |                       | |        walls locate the pod
          |_|        [pod]          |_|  <-- open at the bottom: the tray releases by lifting

Everything is derived:

  * the pocket is `parts.pod.dims` plus a fit clearance, so the tray tracks the pod;
  * its **floor sits on top of the ESC** — `parts.esc.z_in_stack + parts.esc.dims[2]` — because
    that is where the pod lands, so the tray presents the pod at its final height with no number
    of its own;
  * the pick boss is sized against `gantry.tools.<tool>.dims` of whichever tool the sequence uses
    to place `carrier_pod`, and the volume the jaws sweep around it is checked empty
    (tools/cad/gripper.py).

Frame: the plate frame of `parts.bottom_plate` (SolidWorks `CS_DATUM`) — the same frame the pallet
and the spider carrier use, so every z here is directly comparable with `z_in_stack` in the spec.
The tray is centred on the plate centre, the datum midpoint.

Material: no soldering iron ever touches this fixture — sub-line B hands it a finished pod — so it
is printed rather than machined. `MATERIAL` below goes into the STEP header with the file.

Outputs (`--out`, default `cad_out/`): `pod_tray.step` for review in SolidWorks and `pod_tray.stl`
in millimetres, through mesh_pipeline.py so the committed mesh obeys the 5 MB rule. Writes
reports/cad_carrier_pod.json: bbox, mass and every derived position.

Any spec value the tray needs that is null (or absent) is a FAIL that names the key; `--dev`
merges the placeholder overlay instead and writes to `<out>/dev/` and
reports/cad_carrier_pod.dev.json, so a placeholder tray can never be mistaken for a real one.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from build123d import Align, Box, Compound, Pos

sys.path.insert(0, str(Path(__file__).resolve().parent))         # tools/cad: common, gripper
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: spec

import gripper  # noqa: E402
from common import (Values, bbox_doc, export_solids, fouling, lookup, masses,  # noqa: E402
                    new_report, pick_xy, pick_z_note, placing_tool, write_report, z_travel)
from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, load_merged,  # noqa: E402
                  parse_spec, report_path)

DEFAULT_OUT = REPO / "cad_out"
PART = "carrier_pod"
STEM = "pod_tray"                # file/generator name; PART is the spec id
BOSS_LABEL = "Carrier_Pod_Boss"

# ---- fabrication choices, millimetres. Local to this tray: nothing else depends on them. ----
POD_CLEAR = 0.5          # pocket beyond the pod, per side (drop-in, not a press fit)
WALL = 4.0               # tray wall outside that clearance
ROOF_T = 4.0             # tray material above the pod
MID_CLEAR = 4.0          # clear height between the mid plate's top face and the tray's underside
BOSS_D = 25.0            # pick boss diameter
BOSS_H = 35.0            # pick boss height above the roof
GRIP_CLEAR = 2.0         # gap the gantry jaws need around the boss, per side and at the tips
PICK_TOL = 0.05          # spec vs. built pick pose
DENSITY = 1010.0         # kg/m3, ESD-safe PA12 (SLS)
MATERIAL = "ESD-safe PA12 (SLS): carries electronics, never a soldering iron"


# ---------------------------------------------------------------- layout

@dataclass
class Layout:
    """Everything the build needs, in millimetres, derived from the spec."""

    centre: tuple[float, float]     # plate centre = datum midpoint, as in pallet.py
    pod: tuple[float, float, float]
    floor: float                    # top of the ESC: where the pod lands
    mid_top: float                  # top face of the mid plate in its final position
    pick_xy: tuple[float, float]
    tool: str
    grip: gripper.GripDims
    travel_z: float

    @property
    def pocket(self) -> tuple[float, float, float]:
        return (self.pod[0] + 2.0 * POD_CLEAR, self.pod[1] + 2.0 * POD_CLEAR,
                self.pod[2] + POD_CLEAR)

    @property
    def size(self) -> tuple[float, float]:
        return self.pocket[0] + 2.0 * WALL, self.pocket[1] + 2.0 * WALL

    @property
    def roof_top(self) -> float:
        return self.floor + self.pocket[2] + ROOF_T


def layout(v: Values) -> Layout | None:
    """Resolve the spec into a Layout, or None when a value the tray needs is null."""
    p, m, e = "parts.bottom_plate", "parts.mid_plate", "parts.esc"
    round_xy = (v.mm(f"{p}.datums.round[0]"), v.mm(f"{p}.datums.round[1]"))
    diamond_xy = (v.mm(f"{p}.datums.diamond[0]"), v.mm(f"{p}.datums.diamond[1]"))
    centre = ((round_xy[0] + diamond_xy[0]) / 2.0, (round_xy[1] + diamond_xy[1]) / 2.0)

    pod = tuple(v.mm(f"parts.pod.dims[{i}]") for i in range(3))
    # The pod drops onto the ESC, so the ESC's top face is the tray's floor. No number of its own.
    floor = v.mm(f"{e}.z_in_stack") + v.mm(f"{e}.dims[2]")
    mid_top = v.mm(f"{m}.z_in_stack") + v.mm(f"{m}.envelope[2]")

    tool = placing_tool(v, PART)
    jaw = tuple(v.mm(f"gantry.tools.{tool}.dims[{i}]") for i in range(3)) if tool else (0, 0, 0)
    travel = z_travel(v)
    grip_at = pick_xy(v, PART, default=centre)

    if v.missing:
        return None
    return Layout(
        centre=centre, pod=pod, floor=floor, mid_top=mid_top, pick_xy=grip_at, tool=tool,
        grip=gripper.GripDims(boss_d=BOSS_D, boss_h=BOSS_H, jaw_x=jaw[0], jaw_y=jaw[1],
                              reach=jaw[2], clearance=GRIP_CLEAR),
        travel_z=travel)


# ---------------------------------------------------------------- geometry

def shroud(lo: Layout):
    """Walls and roof around the pod, open at the bottom so the tray releases by lifting."""
    cx, cy = lo.centre
    body = Pos(cx, cy, lo.floor) * Box(*lo.size, lo.roof_top - lo.floor,
                                       align=(Align.CENTER, Align.CENTER, Align.MIN))
    body -= Pos(cx, cy, lo.floor) * Box(*lo.pocket, align=(Align.CENTER, Align.CENTER, Align.MIN))
    body.label = "Carrier_Pod_Shroud"
    return body


def build(lo: Layout) -> Compound:
    """Shroud plus pick boss, in the plate frame."""
    boss = Pos(*lo.pick_xy, lo.roof_top) * gripper.boss(lo.grip)
    boss.label = BOSS_LABEL
    tray = Compound(children=[shroud(lo), boss])
    # export_step writes the root label into the STEP header, so the material note travels with
    # the file into SolidWorks (plan P4 B4).
    tray.label = f"Carrier_Pod [{MATERIAL}]"
    return tray


def keep_out_clash(tray: Compound, lo: Layout) -> float:
    """Material of the tray inside the volume the jaws sweep around the pick boss."""
    return fouling(tray, Pos(*lo.pick_xy, lo.roof_top) * gripper.keep_out(lo.grip), BOSS_LABEL)


# ---------------------------------------------------------------- report

def derived(tray: Compound, lo: Layout) -> dict:
    """Positions another tool can check against the spec, all in millimetres."""
    cx, cy = lo.centre
    return {
        "frame": "parts.bottom_plate plate frame (CS_DATUM); z=0 at the plate rest plane",
        "plate_centre_mm": [round(cx, 6), round(cy, 6)],
        "tray_size_mm": [round(c, 6) for c in lo.size],
        "pocket": {"centre_mm": [round(cx, 6), round(cy, 6)],
                   "size_mm": [round(c, 6) for c in lo.pocket],
                   "floor_mm": round(lo.floor, 6),
                   "clearance_mm": POD_CLEAR},
        "roof_top_mm": round(lo.roof_top, 6),
        "mid_plate_clearance_mm": round(lo.floor - lo.mid_top, 6),
        "pick": {"tool": lo.tool,
                 "pose_mm": [round(lo.pick_xy[0], 6), round(lo.pick_xy[1], 6),
                             round(lo.roof_top + BOSS_H, 6)],
                 "boss_d_mm": BOSS_D, "boss_h_mm": BOSS_H,
                 "jaw_mm": [round(lo.grip.jaw_x, 6), round(lo.grip.jaw_y, 6)],
                 "reach_mm": round(lo.grip.reach, 6),
                 "keep_out_volume_mm3": round(keep_out_clash(tray, lo), 4)},
        "gantry_z_travel_mm": round(lo.travel_z, 6),
    }


# ---------------------------------------------------------------- CLI

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate the avionics-pod tray from the spec")
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
    doc = new_report(PART, f"tools/cad/{STEM}.py", out_dir, args.dev)
    report = report_path(f"cad_{PART}", args.dev)

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        write_report(report, doc)
        return code

    def fail(message: str) -> int:
        doc["errors"].append(message)
        print(f"{STEM}: FAIL — {message}", file=sys.stderr)
        return finish(1)

    try:
        merged = load_merged(args.spec, args.dev)
        parse_spec(merged.raw)                      # schema first: a typo is not a missing value
    except SpecError as exc:
        doc["errors"] = exc.errors
        print(f"{STEM}: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)

    v = Values(merged.raw, {p["path"] for p in merged.placeholders})
    lo = layout(v)
    doc["spec_values"] = v.used
    doc["placeholders"] = v.placeholders
    if lo is None:
        doc["missing"] = v.missing
        print(f"{STEM}: FAIL — {args.spec.name} has no value for: {', '.join(v.missing)}",
              file=sys.stderr)
        print(f"{STEM}: fill the key(s) in, or re-run with --dev to use the placeholder overlay",
              file=sys.stderr)
        return finish(1)

    if lo.floor < lo.mid_top + MID_CLEAR:
        return fail(f"the tray's underside sits {lo.floor:.1f} mm above the rest plane (on top of "
                    f"the ESC) but the mid plate tops out at {lo.mid_top:.1f} mm: only "
                    f"{lo.floor - lo.mid_top:.1f} mm of the {MID_CLEAR:g} mm the walls need to "
                    "come down around the pod")
    grip_problems = gripper.problems(lo.grip, lo.tool)
    if grip_problems:
        doc["errors"].extend(grip_problems)
        for message in grip_problems:
            print(f"{STEM}: FAIL — {message}", file=sys.stderr)
        return finish(1)

    tray = build(lo)
    doc["bbox_mm"] = bbox_doc(tray)
    doc["mass_kg"] = masses(tray, {}, default=DENSITY)
    doc["derived"] = derived(tray, lo)
    pick_z_note(v, PART, lo.roof_top + BOSS_H, PICK_TOL, doc)

    clash = doc["derived"]["pick"]["keep_out_volume_mm3"]
    if clash > 0.0:
        return fail(f"{clash:.1f} mm3 of the tray sits in the volume tool '{lo.tool}' jaws sweep "
                    "around the pick boss")
    height = doc["bbox_mm"]["size"][2]
    if height > lo.travel_z:
        return fail(f"the tray is {height:.1f} mm tall but gantry.axes.z travels only "
                    f"{lo.travel_z:.1f} mm: the gantry cannot lift it clear")
    if doc["errors"]:
        print(f"{STEM}: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)

    error = export_solids(doc, tray, out_dir, STEM, stl_tolerance_mm=args.stl_tolerance_mm,
                          max_faces=args.max_faces, bbox_tol_mm=args.bbox_tol_mm)
    if error:
        return fail(error)

    for w in doc["warnings"]:
        print(f"{STEM}: WARN {w}", file=sys.stderr)
    if args.dev:
        print(f"{STEM}: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{out_dir} is not a deliverable")
    size = doc["bbox_mm"]["size"]
    print(f"{STEM}: OK — {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm, "
          f"{doc['mass_kg']['total']:.3f} kg ({doc['mesh']['faces_after']} faces) -> {out_dir}")
    if lookup(merged.raw, f"parts.{PART}.mass") is None:
        print(f"{STEM}: parts.{PART}.mass is null in the spec; measured "
              f"{doc['mass_kg']['total']:.3f} kg", file=sys.stderr)
    if lookup(merged.raw, f"parts.{PART}.pick") is None:
        print(f"{STEM}: parts.{PART}.pick is absent in the spec; the pick boss gives "
              f"{doc['derived']['pick']['pose_mm']} mm", file=sys.stderr)
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
