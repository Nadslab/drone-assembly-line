r"""Spider carrier, generated from line_spec.yaml (plan P4 B3).

    tools/.venv/bin/python tools/cad/spider_carrier.py [--out DIR] [--spec PATH] [--dev]

The carrier is the fixture sub-line A builds the propulsion module in and the main line places
(summary §7.10): it holds four arms at their final radial spacing and the ESC at its final height,
and releases both by lifting straight up once the arm sandwich and the ESC screws are driven.

**The arm pockets are computed from the plate's own screw pattern.** `parts.bottom_plate.arm_holes`
lists every arm-sandwich screw; `per_arm` says how many hold one arm, so the holes group into arms
by their angle about the plate centre. Each arm's pocket is centred on the centroid of its own
group and aimed along the line through its holes, with `parts.arm.hole_inset` placing the arm's
inboard end. Nothing about an arm's position is typed in this file — if the plate's hole pattern
moves, the pockets move with it.

Frame: the plate frame of `parts.bottom_plate` (SolidWorks `CS_DATUM`), the same frame the pallet
uses — origin at the round datum, +Z up, z = 0 at the rest plane (the underside of the bottom
plate). So every z here is directly comparable with `z_in_stack` in the spec.

Built solids, bottom to top:

      ==========================================================   bridge (beams + pick boss)
           ||                  [ESC nest]                 ||       nest hangs below the bridge
           ||  - - - - - - - - - - - - - - - - - - - - -  ||       mid plate slides in under it
      =====++==========\  arm  /=========================++=====   ring, arm pockets in its top
                        \_____/

  * **ring** — a rectangular frame standing clear of the plate footprint, with one pocket per arm
    cut into its top face. The pocket's nominal rectangle reaches inboard over the ring's opening,
    where there is no material: the arm is carried over the run of pocket that is inside the ring
    and its root cantilevers over the plate to land on its screws;
  * **nest** — a downward-open pocket holding the ESC with its underside at `parts.esc.z_in_stack`,
    hung from the bridge so nothing supports it from below;
  * **bridge** — beams across the ring on posts, carrying the nest and the pick boss. Its underside
    clears the mid plate's top face, so the mid plate can be brought in under the suspended ESC
    (main line step 3) and the carrier lifted off afterwards.

Gripper clearance comes from the spec too: the pick boss is sized against
`gantry.tools.<tool>.dims` of whichever tool the sequence uses to place `carrier_spider`, and the
volume the jaws sweep around the boss is checked empty against the rest of the carrier
(tools/cad/gripper.py).

Material: the carrier is hand-soldered in and wiped with IPA between builds (§7.10), so it is
aluminium — `MATERIAL` below is written into the STEP header so the note travels with the file.

Outputs (`--out`, default `cad_out/`): `spider_carrier.step` for review in SolidWorks and
`spider_carrier.stl` in millimetres, through mesh_pipeline.py so the committed mesh obeys the
5 MB rule. Writes reports/cad_spider_carrier.json: bbox, mass, and every derived position.

Any spec value the carrier needs that is null (or absent) is a FAIL that names the key; `--dev`
merges the placeholder overlay instead and writes to `<out>/dev/` and
reports/cad_spider_carrier.dev.json, so a placeholder carrier can never be mistaken for a real one.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from build123d import Align, Box, Compound, Pos, Rot

sys.path.insert(0, str(Path(__file__).resolve().parent))         # tools/cad: common, gripper
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: spec

import gripper  # noqa: E402
from common import (Values, bbox_doc, export_solids, fouling, lookup, masses,  # noqa: E402
                    new_report, pick_xy, pick_z_note, placing_tool, write_report, xyz, z_travel)
from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, load_merged,  # noqa: E402
                  parse_spec, report_path)

DEFAULT_OUT = REPO / "cad_out"
PART = "carrier_spider"
STEM = "spider_carrier"          # file/generator name; PART is the spec id

# ---- fabrication choices, millimetres. Local to this carrier: nothing else depends on them. ----
PLATE_CLEAR = 1.5        # ring opening beyond the largest plate footprint, per side
RING_W = 22.0            # ring width outside the opening
RIB_T = 4.0              # ring material left under an arm pocket
ARM_FIT = 0.3            # pocket wider than the arm, per side (drop-in, not a press fit)
ARM_SUPPORT_MIN = 20.0   # shortest run of pocket that may carry an arm: below this it wobbles
MID_CLEAR = 6.0          # clear height above the mid plate's top face for everything overhead
NEST_CLEAR = 0.4         # nest pocket beyond the ESC, per side
NEST_WALL = 4.0          # nest wall outside that clearance
NEST_ROOF_T = 4.0        # nest material between the ESC and the bridge
BEAM_W = 14.0            # bridge beam width
BEAM_T = 8.0             # bridge beam thickness
POST_MIN = 10.0          # narrowest bridge post
BOSS_D = 25.0            # pick boss diameter
BOSS_H = 35.0            # pick boss height above the bridge
GRIP_CLEAR = 2.0         # gap the gantry jaws need around the boss, per side and at the tips
PICK_TOL = 0.05          # spec vs. built pick pose
DENSITY = 2710.0         # kg/m3, 6061-T6
MATERIAL = "6061-T6 aluminium: survives solder heat and IPA (summary 7.10)"
BOSS_LABEL = "Carrier_Spider_Boss"


# ---------------------------------------------------------------- the arm pattern

@dataclass(frozen=True)
class Arm:
    """One arm, entirely derived from its own group of plate screw holes."""

    holes: tuple[tuple[float, float], ...]   # mm, in the plate frame
    centre: tuple[float, float]              # mm, centroid of `holes` — the pocket's datum
    axis: tuple[float, float]                # unit vector, outboard along the arm

    @property
    def angle_deg(self) -> float:
        return math.degrees(math.atan2(self.axis[1], self.axis[0]))


def _span(angles: list[float]) -> float:
    """Angular width of a group that is already in cyclic order, first to last."""
    return (angles[-1] - angles[0]) % (2.0 * math.pi)


def group_arms(holes: list[tuple[float, float]], per_arm: int,
               centre: tuple[float, float]) -> list[Arm]:
    """Split the plate's arm-sandwich holes into one Arm per group of `per_arm`.

    The holes are sorted by angle about the plate centre and cut into consecutive groups. Which
    cut is right is not a matter of the order they are written in: of the `per_arm` distinct ways
    to cut the cycle, the arms are the one whose groups are angularly tightest.
    """
    cx, cy = centre
    ordered = sorted(holes, key=lambda h: math.atan2(h[1] - cy, h[0] - cx))
    angles = [math.atan2(h[1] - cy, h[0] - cx) for h in ordered]
    n = len(ordered)

    def groups(offset: int) -> list[list[int]]:
        return [[(offset + g * per_arm + k) % n for k in range(per_arm)]
                for g in range(n // per_arm)]

    best = min(range(per_arm),
               key=lambda o: sum(_span([angles[i] for i in g]) for g in groups(o)))

    arms = []
    for group in groups(best):
        pts = [ordered[i] for i in group]
        gx = sum(p[0] for p in pts) / len(pts)
        gy = sum(p[1] for p in pts) / len(pts)
        # The arm lies along its own holes; with a single hole there is no line, so use the
        # radial direction. Either way the axis is oriented outboard from the plate centre.
        dx, dy = (pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) if len(pts) > 1 else (0.0, 0.0)
        if math.hypot(dx, dy) < 1e-9:
            dx, dy = gx - cx, gy - cy
        length = math.hypot(dx, dy)
        if length < 1e-9:
            raise ValueError(f"arm holes {pts} give no direction: they sit on the plate centre")
        if dx * (gx - cx) + dy * (gy - cy) < 0.0:       # point it away from the plate centre
            dx, dy = -dx, -dy
        arms.append(Arm(tuple(pts), (gx, gy), (dx / length, dy / length)))
    return sorted(arms, key=lambda a: a.angle_deg)


# ---------------------------------------------------------------- layout

@dataclass
class Layout:
    """Everything the build needs, in millimetres, derived from the spec."""

    centre: tuple[float, float]     # plate centre = datum midpoint, as in pallet.py
    opening: tuple[float, float]    # ring opening: every plate passes through it
    arms: list[Arm]
    arm_l: float
    arm_w: float
    arm_t: float
    hole_inset: float               # arm inboard end -> centroid of its own holes
    arm_floor: float                # top of the bottom plate: the arms rest here
    mid_top: float                  # top face of the mid plate in its final position
    esc: tuple[float, float, float]
    esc_floor: float
    pick_xy: tuple[float, float]
    tool: str
    grip: gripper.GripDims
    travel_z: float

    @property
    def size(self) -> tuple[float, float]:
        return self.opening[0] + 2.0 * RING_W, self.opening[1] + 2.0 * RING_W

    @property
    def arm_top(self) -> float:
        return self.arm_floor + self.arm_t

    @property
    def pocket_w(self) -> float:
        return self.arm_w + 2.0 * ARM_FIT

    @property
    def esc_top(self) -> float:
        return self.esc_floor + self.esc[2]

    @property
    def bridge_z(self) -> float:
        """Bridge underside: above the nest roof, and clear of everything the gantry brings in."""
        return max(self.esc_top + NEST_CLEAR + NEST_ROOF_T, self.mid_top + MID_CLEAR,
                   self.arm_top + MID_CLEAR)

    @property
    def boss_base(self) -> float:
        return self.bridge_z + BEAM_T

    def pocket_centre(self, arm: Arm) -> tuple[float, float]:
        """Centre of the arm's pocket: its holes' centroid, walked out along the arm.

        `hole_inset` puts the arm's inboard end at `centre - hole_inset * axis`, so the pocket
        rectangle runs from there for the arm's full length.
        """
        offset = self.arm_l / 2.0 - self.hole_inset
        return (arm.centre[0] + offset * arm.axis[0], arm.centre[1] + offset * arm.axis[1])


def layout(v: Values) -> Layout | None:
    """Resolve the spec into a Layout, or None when a value the carrier needs is null."""
    p, m, a, e = "parts.bottom_plate", "parts.mid_plate", "parts.arm", "parts.esc"
    round_xy = (v.mm(f"{p}.datums.round[0]"), v.mm(f"{p}.datums.round[1]"))
    diamond_xy = (v.mm(f"{p}.datums.diamond[0]"), v.mm(f"{p}.datums.diamond[1]"))
    centre = ((round_xy[0] + diamond_xy[0]) / 2.0, (round_xy[1] + diamond_xy[1]) / 2.0)

    # Every plate that passes through the ring while the carrier is on the pallet: the bottom
    # plate the carrier lands over, and the mid plate that follows it.
    plate_xy = [max(v.mm(f"{p}.envelope[{i}]"), v.mm(f"{m}.envelope[{i}]")) for i in (0, 1)]
    arm_floor = v.mm(f"{p}.envelope[2]")
    mid_top = v.mm(f"{m}.z_in_stack") + v.mm(f"{m}.envelope[2]")

    per_arm = v.plain(f"{p}.arm_holes.per_arm")
    raw_holes = lookup(v.raw, f"{p}.arm_holes.positions") or []
    holes = [(v.mm(f"{p}.arm_holes.positions[{i}][0]"), v.mm(f"{p}.arm_holes.positions[{i}][1]"))
             for i in range(len(raw_holes))]
    if not holes:
        v.missing.append(f"{p}.arm_holes.positions")

    arm_dims = tuple(v.mm(f"{a}.dims[{i}]") for i in range(3))
    hole_inset = v.mm(f"{a}.hole_inset")
    esc = tuple(v.mm(f"{e}.dims[{i}]") for i in range(3))
    esc_floor = v.mm(f"{e}.z_in_stack")

    tool = placing_tool(v, PART)
    jaw = tuple(v.mm(f"gantry.tools.{tool}.dims[{i}]") for i in range(3)) if tool else (0, 0, 0)
    travel = z_travel(v)

    grip_at = pick_xy(v, PART, default=centre)

    if v.missing:
        return None
    return Layout(
        centre=centre,
        opening=(plate_xy[0] + 2.0 * PLATE_CLEAR, plate_xy[1] + 2.0 * PLATE_CLEAR),
        arms=group_arms(holes, per_arm, centre),
        arm_l=arm_dims[0], arm_w=arm_dims[1], arm_t=arm_dims[2], hole_inset=hole_inset,
        arm_floor=arm_floor, mid_top=mid_top, esc=esc, esc_floor=esc_floor,
        pick_xy=grip_at, tool=tool,
        grip=gripper.GripDims(boss_d=BOSS_D, boss_h=BOSS_H, jaw_x=jaw[0], jaw_y=jaw[1],
                              reach=jaw[2], clearance=GRIP_CLEAR),
        travel_z=travel)


# ---------------------------------------------------------------- geometry

def arm_pockets(lo: Layout) -> list:
    """The cutter for each arm's pocket, labelled ArmPocket_1..n in order of angle."""
    out = []
    for n, arm in enumerate(lo.arms, 1):
        px, py = lo.pocket_centre(arm)
        cutter = (Pos(px, py, lo.arm_floor) * Rot(Z=arm.angle_deg)
                  * Box(lo.arm_l, lo.pocket_w, lo.arm_t,
                        align=(Align.CENTER, Align.CENTER, Align.MIN)))
        cutter.label = f"ArmPocket_{n}"
        out.append(cutter)
    return out


def ring(lo: Layout):
    """The frame the arms are carried on: a rectangle standing clear of the plate footprint."""
    cx, cy = lo.centre
    body = Pos(cx, cy, lo.arm_top) * Box(*lo.size, RIB_T + lo.arm_t,
                                         align=(Align.CENTER, Align.CENTER, Align.MAX))
    tall = 4.0 * (RIB_T + lo.arm_t)
    return body - Pos(cx, cy) * Box(*lo.opening, tall)


def nest(lo: Layout):
    """The ESC pocket: open at the bottom so the ESC is released by lifting the carrier."""
    cx, cy = lo.centre
    wall = 2.0 * (NEST_CLEAR + NEST_WALL)
    body = Pos(cx, cy, lo.esc_floor) * Box(lo.esc[0] + wall, lo.esc[1] + wall,
                                           lo.bridge_z - lo.esc_floor,
                                           align=(Align.CENTER, Align.CENTER, Align.MIN))
    pocket = Pos(cx, cy, lo.esc_floor) * Box(lo.esc[0] + 2.0 * NEST_CLEAR,
                                             lo.esc[1] + 2.0 * NEST_CLEAR,
                                             lo.esc[2] + NEST_CLEAR,
                                             align=(Align.CENTER, Align.CENTER, Align.MIN))
    body -= pocket
    body.label = "Carrier_Spider_Nest"
    return body


def bridge(lo: Layout):
    """Beams over the ring on posts: they carry the nest and the pick boss, and nothing else.

    A beam runs the full width of the ring under the pick point and another crosses it over the
    nest, so the boss is supported wherever `pick` puts it and the nest always hangs from a beam.
    """
    cx, cy = lo.centre
    sx, sy = lo.size
    body = None

    def add(shape):
        nonlocal body
        body = shape if body is None else body + shape

    def beam(x, y, w, h):
        add(Pos(x, y, lo.bridge_z) * Box(w, h, BEAM_T,
                                         align=(Align.CENTER, Align.CENTER, Align.MIN)))

    def post(x, y, w, h):
        add(Pos(x, y, lo.arm_top) * Box(max(w, POST_MIN), max(h, POST_MIN),
                                        lo.bridge_z - lo.arm_top,
                                        align=(Align.CENTER, Align.CENTER, Align.MIN)))

    beam(cx, cy, sx, BEAM_W)                                    # across the nest, along x
    for side in (-1.0, 1.0):
        post(cx + side * (sx - RING_W) / 2.0, cy, RING_W, BEAM_W)
    for x in dict.fromkeys((cx, lo.pick_xy[0])):                # skip a duplicate under the nest
        if x != cx and abs(x - cx) < BEAM_W:
            continue
        beam(x, cy, BEAM_W, sy)                                 # along y, under the pick point
        for side in (-1.0, 1.0):
            post(x, cy + side * (sy - RING_W) / 2.0, BEAM_W, RING_W)
    body.label = "Carrier_Spider_Bridge"
    return body


def build(lo: Layout) -> Compound:
    """Ring (with the arm pockets cut), ESC nest, bridge and pick boss, in the plate frame."""
    frame = ring(lo)
    for cutter in arm_pockets(lo):
        frame -= cutter
    frame.label = "Carrier_Spider_Ring"

    boss = Pos(*lo.pick_xy, lo.boss_base) * gripper.boss(lo.grip)
    boss.label = BOSS_LABEL

    carrier = Compound(children=[frame, nest(lo), bridge(lo), boss])
    # export_step writes the root label into the STEP header, so the material note travels with
    # the file into SolidWorks (plan P4 B3).
    carrier.label = f"Carrier_Spider [{MATERIAL}]"
    return carrier


def arm_support(lo: Layout) -> list[float]:
    """How long a run of each pocket is actually backed by ring material, in millimetres.

    The pocket is nominally the whole arm, but its inboard end reaches over the ring's opening
    where there is nothing. Dividing the material the cutter removes by the pocket's cross
    section gives the run that carries the arm.
    """
    frame = ring(lo)
    section = lo.pocket_w * lo.arm_t
    return [(cutter & frame).volume / section for cutter in arm_pockets(lo)]


# ---------------------------------------------------------------- report

def derived(carrier: Compound, lo: Layout) -> dict:
    """Positions another tool can check against the spec, all in millimetres."""
    cutters = arm_pockets(lo)
    support = arm_support(lo)
    arms = []
    for n, (arm, cutter, run) in enumerate(zip(lo.arms, cutters, support), 1):
        arms.append({
            "name": f"Arm_{n}",
            "holes_mm": [[round(c, 6) for c in h] for h in arm.holes],
            "hole_centre_mm": [round(c, 6) for c in arm.centre],
            "axis_deg": round(arm.angle_deg, 6),
            "pocket_centre_mm": xyz(cutter.center())[:2],       # measured off the cutter solid
            "pocket_mm": [round(lo.arm_l, 6), round(lo.pocket_w, 6), round(lo.arm_t, 6)],
            "supported_run_mm": round(run, 4)})
    return {
        "frame": "parts.bottom_plate plate frame (CS_DATUM); z=0 at the plate rest plane",
        "plate_centre_mm": [round(c, 6) for c in lo.centre],
        "ring_opening_mm": [round(c, 6) for c in lo.opening],
        "ring_size_mm": [round(c, 6) for c in lo.size],
        "arms": arms,
        "arm_floor_mm": round(lo.arm_floor, 6),
        "esc_nest": {"floor_mm": round(lo.esc_floor, 6), "roof_mm": round(lo.bridge_z, 6),
                     "pocket_mm": [round(lo.esc[0] + 2.0 * NEST_CLEAR, 6),
                                   round(lo.esc[1] + 2.0 * NEST_CLEAR, 6),
                                   round(lo.esc[2] + NEST_CLEAR, 6)],
                     "mid_plate_clearance_mm": round(lo.esc_floor - lo.mid_top, 6)},
        "bridge": {"underside_mm": round(lo.bridge_z, 6), "top_mm": round(lo.boss_base, 6),
                   "mid_plate_clearance_mm": round(lo.bridge_z - lo.mid_top, 6)},
        "pick": {"tool": lo.tool,
                 "pose_mm": [round(lo.pick_xy[0], 6), round(lo.pick_xy[1], 6),
                             round(lo.boss_base + BOSS_H, 6)],
                 "boss_d_mm": BOSS_D, "boss_h_mm": BOSS_H,
                 "jaw_mm": [round(lo.grip.jaw_x, 6), round(lo.grip.jaw_y, 6)],
                 "reach_mm": round(lo.grip.reach, 6),
                 "keep_out_volume_mm3": round(keep_out_clash(carrier, lo), 4)},
        "gantry_z_travel_mm": round(lo.travel_z, 6),
    }


def keep_out_clash(carrier: Compound, lo: Layout) -> float:
    """Material of the carrier inside the volume the jaws sweep around the pick boss."""
    return fouling(carrier, Pos(*lo.pick_xy, lo.boss_base) * gripper.keep_out(lo.grip), BOSS_LABEL)


# ---------------------------------------------------------------- CLI

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate the spider carrier from the spec")
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
    try:
        lo = layout(v)
    except ValueError as exc:                       # the hole pattern does not define arms
        doc["spec_values"] = v.used
        return fail(f"parts.bottom_plate.arm_holes: {exc}")
    doc["spec_values"] = v.used
    doc["placeholders"] = v.placeholders
    if lo is None:
        doc["missing"] = v.missing
        print(f"{STEM}: FAIL — {args.spec.name} has no value for: {', '.join(v.missing)}",
              file=sys.stderr)
        print(f"{STEM}: fill the key(s) in, or re-run with --dev to use the placeholder overlay",
              file=sys.stderr)
        return finish(1)

    if lo.esc_floor < lo.mid_top + MID_CLEAR:
        return fail(f"parts.esc.z_in_stack puts the ESC underside {lo.esc_floor:.1f} mm above the "
                    f"rest plane, but the mid plate tops out at {lo.mid_top:.1f} mm: the nest "
                    f"leaves {lo.esc_floor - lo.mid_top:.1f} mm, less than the {MID_CLEAR:g} mm "
                    "the mid plate needs to come in under the suspended ESC")
    grip_problems = gripper.problems(lo.grip, lo.tool)
    if grip_problems:
        doc["errors"].extend(grip_problems)
        for message in grip_problems:
            print(f"{STEM}: FAIL — {message}", file=sys.stderr)
        return finish(1)
    for n, run in enumerate(arm_support(lo), 1):
        if run < ARM_SUPPORT_MIN:
            return fail(f"arm {n}'s pocket is backed by only {run:.1f} mm of ring "
                        f"(min {ARM_SUPPORT_MIN:g} mm): widen RING_W or move the arm holes inboard")

    carrier = build(lo)
    doc["bbox_mm"] = bbox_doc(carrier)
    doc["mass_kg"] = masses(carrier, {}, default=DENSITY)
    doc["derived"] = derived(carrier, lo)
    pick_z_note(v, PART, lo.boss_base + BOSS_H, PICK_TOL, doc)

    clash = doc["derived"]["pick"]["keep_out_volume_mm3"]
    if clash > 0.0:
        return fail(f"{clash:.1f} mm3 of the carrier sits in the volume tool '{lo.tool}' jaws "
                    "sweep around the pick boss")
    height = doc["bbox_mm"]["size"][2]
    if height > lo.travel_z:
        return fail(f"the carrier is {height:.1f} mm tall but gantry.axes.z travels only "
                    f"{lo.travel_z:.1f} mm: the gantry cannot lift it clear")
    if doc["errors"]:
        print(f"{STEM}: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)

    error = export_solids(doc, carrier, out_dir, STEM, stl_tolerance_mm=args.stl_tolerance_mm,
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
          f"{doc['mass_kg']['total']:.3f} kg, {len(lo.arms)} arm pocket(s) "
          f"({doc['mesh']['faces_after']} faces) -> {out_dir}")
    if lookup(merged.raw, f"parts.{PART}.mass") is None:
        print(f"{STEM}: parts.{PART}.mass is null in the spec; measured "
              f"{doc['mass_kg']['total']:.3f} kg", file=sys.stderr)
    if lookup(merged.raw, f"parts.{PART}.pick") is None:
        print(f"{STEM}: parts.{PART}.pick is absent in the spec; the pick boss gives "
              f"{doc['derived']['pick']['pose_mm']} mm", file=sys.stderr)
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
