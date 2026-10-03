"""Layout-level CAD assembly of the main line, from line_spec.yaml, for review in SolidWorks.

    tools/.venv/bin/python tools/cad/layout.py [--dev] [--milestone NAME] [--no-reference]
                                               [--spec PATH] [--out FILE.step]

Simple solids in the line frame (origin = pallet datum at st_load_pallet, Z up; the floor is at
z = -line.work_height), exported in millimetres:

  Conveyor   Conveyor, Pallet_<n> (one block at station n's pose, n = its index), and
             Pallet_Slot_<n>_<k> (queued-pallet outlines after station n, at slot pitch =
             pallet length + layout.slot_gap, as many as fit: spec.queue_gaps, D-014)
  Stations   Station_<Name> (stop_unit_length x conveyor width, floor to belt top), and for a
             station with count > 1 one Station_<Name>_Cell<k> per cell on the test branch
  Robots     Scara_<Name>_Base (a layout.scara_base pedestal from the floor to the mounting face
             at the SCARA's resolved pose) with Scara_<Name> on it — its robot model's vendor STEP
             (tools/cad/vendor.py), posed folded (j1 = 0, j2 at its limit); with no STEP the
             pedestal is a layout.scara_base.height cylinder and the run WARNs — and
             GangHead_<Name> (a layout.gang_head_box at its working pose — or, on a slide, where
             it waits while the SCARA works; tools/robot_geometry.head_box) with, on a slide,
             its frame (D-014): GangHead_<Name>_Rail along the slide axis and GangHead_<Name>_Post
             beyond the retracted head
  Carriers   Carrier_<Name> on the pallet at every station it rides through (placed, not yet
             removed: robot_geometry.stack_parts), from its generator's STEP in cad_out/[dev/]
             (its bbox as a box when the STEP is not built)
  Feeders    Feeder_<Name> at its pose, sized by layout.feeder_footprints (a feeder with no pose —
             a blow feeder riding its gang head — has nothing to place)
  Buffers    Buffer_<Name>, the same
  Returns    ReturnPath_<Name> (D-013): the chute or conveyor segment an empty carrier goes back
             on — `width` across, `length` from its put-down point along its yaw, floor to the
             put-down plane — and ReturnSink_<Name> at its far end, the sub-line it feeds
             (SINK_SIZE square, floor to SINK_H below the put-down plane)
  Reference  non-physical solids: Floor (bbox of everything + 1 m), OperatorAisle (outside the
             fence on its side), Guard_Fence (D-013: one wall guard_offset outside everything
             worked at, tunnelled where the conveyor and each return chute pass) and
             Guard_Partition_<A>_<B> (a wall across it at each milestone boundary), Scara_<Name>_Envelope (an annular cylinder, the model's r_min..reach,
             over the TCP's z range) and GangHead_<Name>_Slide (the volume a head on a slide
             sweeps between waiting and working)

Every dimension carries the spec keys it came from; a solid that used any placeholder is coloured
by its category exactly as a real one and its name ends in `_PH` — placeholder status never
changes colour. The numbers typed below are envelopes (floor slab, guard wall, outline rim),
listed in the report as constants — no spec value comes from them.

Interference: every pair of solids is intersected (bbox first, then the boolean). Only the pairs in
ALLOWED_OVERLAPS may share volume, and those in ALLOWED_SAME_OWNER when both belong to the same
gang head (its head in its rail, its rail on its post); anything else above VOLUME_TOL_MM3 is a
FAIL naming both solids.

Park (D-014): for every SCARA that parks while its station's gang head works, the vendor STEP is
posed at its park_pose and intersected with the head at its working position and its slide frame
(layout.json `park`: overlap volumes, next to robot_geometry's capsule clearance; any overlap is a
FAIL), and reports/render/<station>_park.png shows that cell with the arm parked and the head in.
The arm capsules robot_geometry checks with are measured from the same STEP into
reports/vendor_<model>.json on every run.

Axes: all of the above — layout.json, bbox_mm, the interference/straddle checks — stay in the spec
(line) frame, Z up. Only the exported STEP is rotated: SolidWorks is Y-up, so the whole assembly is
rotated EXPORT_ROT_X_DEG (-90 deg about X) right before writing, mapping spec (x, y, z) to STEP
(x, z, -y) so SolidWorks' Top view is the plan view.

Renders (D-013): reports/render/<milestone or line>_top.png and _iso.png, offscreen through
pyvista (tools/render/render.py), for a quick look without SolidWorks.

Outputs: two STEP assemblies next to each other — line_layout.step (Line > Conveyor, Stations,
Robots, Feeders, Buffers, Reference; `--no-reference` drops Reference from this one too) and
line_layout_physical.step (always written, always without Reference) — a top-view SVG, and
reports/layout.json with every solid's bbox and dimension provenance. Each STEP is re-imported and
its names and hierarchy compared with what was built; if line_layout.step's hierarchy does not
survive, one STEP per group and a manifest.txt are written instead.
`--dev` writes to <out dir>/dev/ and reports/layout.dev.json. `--milestone NAME` (D-012) draws
only that milestone's stations, their SCARAs, gang heads, feeders and buffers
(spec.milestone_subset), on a conveyor just long enough for them, into <out dir>[/dev]/<NAME>/
with reports/layout.<NAME>[.dev].json. The STEP is generated: edit the spec, never the SolidWorks
file.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import re
import resource
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from build123d import (Box, Color, Compound, Cylinder, Location, Pos, Rot, Shape, export_step,
                       import_step)

sys.path.insert(0, str(Path(__file__).resolve().parent))         # tools/cad: common
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: spec
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "render"))

import render as render_png  # noqa: E402
import robot_geometry as rg  # noqa: E402
import vendor  # noqa: E402
from common import Values, lookup, pascal  # noqa: E402
from spec import (DEFAULT_SPEC, REPO, LineSpec, SpecError, add_dev_argument,  # noqa: E402
                  load_merged,
                  milestone_subset, parse_spec, queue_gaps, report_path, scara_ids)

DEFAULT_OUT = Path("/mnt/c/Users/Admin/Documents/drone-line-cad/60_layout/line_layout.step")
PHYSICAL_NAME = "line_layout_physical.step"   # the same assembly, Reference group dropped
SVG_NAME = "line_layout_top.svg"
GROUPS = ("Conveyor", "Stations", "Robots", "Feeders", "Buffers", "Returns", "Reference")
PARK_WINDOW = 0.9        # m either side of a station's x drawn in its park close-up
CAD_OUT = Path(__file__).resolve().parent.parent.parent / "cad_out"
REPORTS = Path(__file__).resolve().parent.parent.parent / "reports"
CARRIER_STEPS = {"carrier_spider": "spider_carrier.step", "carrier_pod": "pod_tray.step"}
PHYSICAL_GROUPS = GROUPS[:-1]                 # every group except Reference

# Export-only axis fix: SolidWorks is Y-up, the spec/line frame is Z-up. Rotating -90 deg about X
# sends spec (x, y, z) -> STEP (x, z, -y), so SolidWorks' Top view shows the plan view. Applied to
# a rotated copy of the assembly right before write_step() (rotate_tree()); layout.json, bbox_mm
# and every check (interference, straddle, opacity) stay in the spec/line frame.
EXPORT_ROT_X_DEG = -90.0

# ---- envelopes, metres. Drawing choices, not spec values; each is named in the report. ----
FLOOR_T = 0.02           # floor slab thickness
FLOOR_MARGIN = 1.0       # floor beyond the bounding box of everything, each side
AISLE_T = 0.005          # aisle marking on the floor
GUARD_T = 0.01           # guard wall thickness
GUARD_H = 2.0            # guard wall top above the floor
TUNNEL_CLEAR = 0.05      # opening above the pallet top where the conveyor passes a guard
OUTLINE_RIM = 0.01       # rim width of a queued-pallet outline
OUTLINE_H = 0.005        # height of a queued-pallet outline
SINK_SIZE = 0.30         # plan size of the block standing for a return path's sub-line end
SINK_H = 0.10            # how far below the put-down plane the sink's top is (a chute falls)
PALLET_DATUM_TOL = 0.001  # pallet top vs the datum plane: WARN beyond this
VOLUME_TOL_MM3 = 1.0     # shared volume below this is two faces touching, not an overlap

# Pairs of categories that may share volume. Everything else is a FAIL.
ALLOWED_OVERLAPS = (
    ("pallet", "conveyor"),      # a pallet (or a queued outline) rides on the belt
    ("carrier", "pallet"),       # a carrier's locating features sit into the pallet
    ("station", "conveyor"),     # a station footprint straddles the belt
    ("envelope", "*"),           # a reach/slide envelope is a volume a robot sweeps, not a body
    ("floor", "*"),
)
# Pairs that may share volume only when both solids belong to the same gang head (D-014).
ALLOWED_SAME_OWNER = (
    ("gang_head", "slide_frame"),    # the head's carriage runs in its rail
    ("slide_frame", "slide_frame"),  # the rail sits on its post
)

# Every solid is always coloured by category, placeholder or not: placeholder status shows only
# in the `_PH` name suffix and in layout.json, never as a colour override.
CATEGORY_COLOUR = {
    "floor": "reference", "aisle": "reference", "guard": "reference", "envelope": "reference",
    "conveyor": "conveyor", "pallet": "pallet", "station": "station", "robot": "robot",
    "gang_head": "gang_head", "feeder": "feeder", "buffer": "buffer", "carrier": "carrier",
    "return": "return", "slide_frame": "slide_frame",
}
COLOURS = {                      # by colour bucket, RGB 0-1
    "reference": (0.80, 0.80, 0.82), "conveyor": (0.45, 0.45, 0.47), "pallet": (0.20, 0.45, 0.85),
    "station": (0.25, 0.65, 0.35), "robot": (0.00, 0.55, 0.55), "gang_head": (0.65, 0.38, 0.15),
    "feeder": (0.95, 0.80, 0.20), "buffer": (0.55, 0.25, 0.70), "carrier": (0.85, 0.20, 0.25),
    "return": (0.95, 0.55, 0.65), "slide_frame": (0.50, 0.52, 0.58),
}
PLACEHOLDER_COLOUR = (1.0, 0.55, 0.0)  # SVG placeholder outline only — never a STEP fill colour


def colour_of(category: str) -> tuple:
    return COLOURS[CATEGORY_COLOUR[category]]

# Every solid is fully opaque, envelopes and placeholders included. OCCT writes an opacity entity
# only for alpha < 1, so at alpha 1.0 the file says nothing about opacity and a viewer is left to
# guess. write_step() has OCCT write the entity (PROBE_ALPHA) and then sets it to transparency 0.
OPAQUE = 1.0
PROBE_ALPHA = 0.99
ALPHA_TOL = 1e-6
TRANSPARENT_ENTITY = re.compile(r"SURFACE_STYLE_TRANSPARENT\(([^)]*)\)")


# ---------------------------------------------------------------- dimensions with provenance

@dataclass(frozen=True)
class Dim:
    """A length in metres, with the spec keys (and constants) it was computed from."""

    value: float
    keys: frozenset = frozenset()
    placeholder: bool = False
    consts: frozenset = frozenset()

    def _op(self, other, fn):
        o = other if isinstance(other, Dim) else Dim(float(other))
        return Dim(fn(self.value, o.value), self.keys | o.keys, self.placeholder or o.placeholder,
                   self.consts | o.consts)

    def __add__(self, o): return self._op(o, lambda a, b: a + b)
    def __radd__(self, o): return self._op(o, lambda a, b: b + a)
    def __sub__(self, o): return self._op(o, lambda a, b: a - b)
    def __rsub__(self, o): return self._op(o, lambda a, b: b - a)
    def __mul__(self, o): return self._op(o, lambda a, b: a * b)
    def __rmul__(self, o): return self._op(o, lambda a, b: b * a)
    def __truediv__(self, o): return self._op(o, lambda a, b: a / b)
    def __neg__(self): return Dim(-self.value, self.keys, self.placeholder, self.consts)

    def doc(self) -> dict:
        return {"value_mm": round(self.value * 1000.0, 3), "spec_keys": sorted(self.keys),
                "placeholder": self.placeholder, "constants": sorted(self.consts)}


def const(name: str) -> Dim:
    return Dim(float(globals()[name]), consts=frozenset({name}))


def dmin(*dims: Dim) -> Dim:
    return min(dims, key=lambda d: d.value)


def dmax(*dims: Dim) -> Dim:
    return max(dims, key=lambda d: d.value)


class Reader:
    """Spec reads as Dims: Values records what was missing and what came from the overlay."""

    def __init__(self, v: Values):
        self.v = v

    def __call__(self, path: str) -> Dim:
        value = self.v.m(path)
        return Dim(value, frozenset({path}), self.v.is_placeholder(path))

    def plain(self, path: str, default=None):
        return self.v.plain(path, default)


# ---------------------------------------------------------------- the solids

@dataclass
class Item:
    name: str
    category: str
    group: str
    centre: tuple[Dim, Dim, Dim]
    size: tuple[Dim, Dim, Dim]
    subgroup: str | None = None                 # a named assembly inside its group
    yaw: Dim | None = None                      # radians about z
    hollow: tuple[Dim, Dim] | None = None       # inner x, y of a ring (outline, guard)
    cylinder: bool = False                      # size x = diameter (= size y); hollow[0] inner d
    cuts: list[tuple[tuple[Dim, Dim, Dim], tuple[Dim, Dim, Dim]]] = field(default_factory=list)
    shape: object = None
    solid: object = None                        # a ready shape (vendor STEP, carrier), mm
    owner: str | None = None                    # the gang head a slide frame belongs to

    def dims(self) -> list[Dim]:
        out = list(self.centre) + list(self.size)
        out += [self.yaw] if self.yaw is not None else []
        out += list(self.hollow) if self.hollow else []
        for c, s in self.cuts:
            out += list(c) + list(s)
        return out

    @property
    def placeholder(self) -> bool:
        return any(d.placeholder for d in self.dims())

    @property
    def label(self) -> str:
        return f"{self.name}_PH" if self.placeholder else self.name

    def lo(self, i: int) -> Dim:
        return self.centre[i] - self.size[i] / 2.0

    def hi(self, i: int) -> Dim:
        return self.centre[i] + self.size[i] / 2.0


def relocated(shape, loc: Location | None = None):
    """`shape` at `loc` (or where it is) as a fresh leaf: the same OCC geometry, its label and
    colour, no parent. build123d's own `moved()` / `Location * shape` deep-copy the shape first,
    and for a shape already in an assembly that copies the whole tree: ~1 s and up to 100 MB per
    solid, which ran the full line (five vendor arms) out of a 7.8 GB machine (D-013)."""
    out = Shape.cast(shape.wrapped.Moved((loc or Location()).wrapped))
    if isinstance(out, Compound) and len(out.solids()) == 1:
        out = out.solids()[0]               # a one-solid part exports as a solid, colour kept
    out.label, out.color = shape.label, shape.color
    return out


def box_mm(centre, size):
    return Pos(*(c.value * 1000.0 for c in centre)) * Box(*(s.value * 1000.0 for s in size))


def build_shape(it: Item):
    if it.solid is not None:
        solid = it.solid
        solid.label = it.label
        solid.color = Color(*colour_of(it.category), OPAQUE)
        return solid
    if it.cylinder:
        solid = Cylinder(it.size[0].value * 500.0, it.size[2].value * 1000.0)
        if it.hollow:
            solid -= Cylinder(it.hollow[0].value * 500.0, it.size[2].value * 2000.0)
    else:
        solid = Box(*(s.value * 1000.0 for s in it.size))
    if it.hollow and not it.cylinder:
        solid -= Box(it.hollow[0].value * 1000.0, it.hollow[1].value * 1000.0,
                     it.size[2].value * 1000.0 * 2.0)
    if it.yaw is not None and it.yaw.value:
        solid = Rot(0, 0, it.yaw.value * 180.0 / 3.141592653589793) * solid
    solid = Pos(*(c.value * 1000.0 for c in it.centre)) * solid
    for c, s in it.cuts:
        solid -= box_mm(c, s)
    solid.label = it.label
    solid.color = Color(*colour_of(it.category), OPAQUE)
    return solid


def span(lo: Dim, hi: Dim) -> tuple[Dim, Dim]:
    """(centre, size) of the interval [lo, hi]."""
    return (lo + hi) / 2.0, hi - lo


def plan(v: Values, spec: LineSpec, cad: rg.CadHeights, notes: dict | None = None
         ) -> list[Item] | None:
    """Every solid, as Dims. None when a value the layout needs is null.

    `spec` is `v.raw` parsed; `cad` is where the gang heads' stack heights and the carriers come
    from (they sit on the stack, tools/robot_geometry.head_box). `notes` collects warnings and
    what was measured in the vendor STEPs."""
    notes = notes if notes is not None else {}
    notes.setdefault("warnings", [])
    notes.setdefault("vendor", {})
    notes.setdefault("vendor_warned", set())
    q = Reader(v)
    items: list[Item] = []
    L = "layout"
    floor_z = -q("line.work_height")
    belt_top = floor_z + q(f"{L}.conveyor.top_height")
    belt_w = q(f"{L}.conveyor.width")
    pallet = (q(f"{L}.pallet_block.length"), q(f"{L}.pallet_block.width"),
              q(f"{L}.pallet_block.thickness"))

    stations = sorted(v.raw.get("stations") or {},
                      key=lambda s: lookup(v.raw, f"stations.{s}.index") or 0)
    pose = {s: (q(f"stations.{s}.pose[0]"), q(f"stations.{s}.pose[1]")) for s in stations}
    index = {s: v.plain(f"stations.{s}.index") for s in stations}

    # Conveyor: along every station pose, floor to belt top, pitch/2 beyond the end stations.
    half_pitch = q("conveyor.pitch") / 2.0
    x0 = dmin(*(pose[s][0] for s in stations)) - half_pitch
    x1 = dmax(*(pose[s][0] for s in stations)) + half_pitch
    y0 = dmin(*(pose[s][1] for s in stations)) - belt_w / 2.0
    y1 = dmax(*(pose[s][1] for s in stations)) + belt_w / 2.0
    cx, sx = span(x0, x1)
    cy, sy = span(y0, y1)
    cz, sz = span(floor_z, belt_top)
    items.append(Item("Conveyor", "conveyor", "Conveyor", (cx, cy, cz), (sx, sy, sz)))

    # Pallets on the belt at each station, and queued outlines between consecutive stations.
    for s in stations:
        items.append(Item(f"Pallet_{index[s]}", "pallet", "Conveyor",
                          (pose[s][0], pose[s][1], belt_top + pallet[2] / 2.0), pallet))
    gaps = {(g["after"], g["before"]): g for g in queue_gaps(spec)}
    slot_pitch = q(f"{L}.pallet_block.length") + q(f"{L}.slot_gap")
    for a, b in zip(stations, stations[1:]):
        g = gaps.get((a, b)) or {}
        dist = math.hypot(pose[b][0].value - pose[a][0].value, pose[b][1].value - pose[a][1].value)
        for k in range(1, int(g.get("slots") or 0) + 1):
            f = slot_pitch * (k / dist if dist else 0.0)
            at = (pose[a][0] + (pose[b][0] - pose[a][0]) * f,
                  pose[a][1] + (pose[b][1] - pose[a][1]) * f)
            rim = const("OUTLINE_RIM") * 2.0
            items.append(Item(f"Pallet_Slot_{index[a]}_{k}", "pallet", "Conveyor",
                              (at[0], at[1], belt_top + const("OUTLINE_H") / 2.0),
                              (pallet[0], pallet[1], const("OUTLINE_H")),
                              hollow=(pallet[0] - rim, pallet[1] - rim)))

    # Stations: a footprint across the belt; a multi-cell station gets its cells on the branch.
    stop = q(f"{L}.stop_unit_length")
    for s in stations:
        name = f"Station_{pascal(s.removeprefix('st_'))}"
        count = v.plain(f"stations.{s}.count")
        if count and int(count) > 1:
            off = q(f"{L}.test_branch.offset_y")
            pitch = q(f"stations.{s}.cell_pitch")
            step = 1.0 if off.value >= 0 else -1.0
            for k in range(int(count)):
                items.append(Item(
                    f"{name}_Cell{k + 1}", "station", "Stations",
                    (pose[s][0], pose[s][1] + off + pitch * (step * k), cz),
                    (q(f"{L}.test_branch.cell_d"), q(f"{L}.test_branch.cell_w"), sz)))
        else:
            items.append(Item(name, "station", "Stations", (pose[s][0], pose[s][1], cz),
                              (stop, belt_w, sz)))

    # Carriers (D-013): on the pallet at every station they ride through.
    for s in stations:
        for part in rg.stack_parts(spec, s, own=True):
            if part not in CARRIER_STEPS:
                continue
            shape = carrier_shape(part, cad, notes)
            if shape is None:
                continue
            pz = q(f"stations.{s}.pose[2]")
            at = Pos(pose[s][0].value * 1000.0, pose[s][1].value * 1000.0, pz.value * 1000.0)
            keys = pose[s][0].keys | pose[s][1].keys | pz.keys
            ph = pose[s][0].placeholder or pose[s][1].placeholder or pz.placeholder
            sub = f"{pascal(part)}_{index[s]}"
            solids = shape.solids()
            for i, solid in enumerate(solids):
                solid = at * solid
                bb = solid.bounding_box()
                items.append(Item(sub if len(solids) == 1 else f"{sub}_{i + 1}", "carrier",
                                  "Conveyor",
                                  tuple(Dim(c / 1000.0, keys, ph) for c in tuple(bb.center())),
                                  tuple(Dim(c / 1000.0, keys, ph) for c in tuple(bb.size)),
                                  subgroup=sub if len(solids) > 1 else None, solid=solid))

    # Robots: a SCARA (pedestal + vendor arm) per SCARA, its reach envelope; a gang head box per
    # head.
    def derived(value: float, paths: set[str]) -> Dim:
        """A number robot_geometry worked out, carrying the spec keys it came from."""
        return Dim(value, frozenset(paths), any(v.is_placeholder(k) for k in paths))

    base_d, base_h = q(f"{L}.scara_base.diameter"), q(f"{L}.scara_base.height")
    reach_rects: dict[str, list[tuple[Dim, Dim, Dim, Dim]]] = {}   # station -> plan rects
    for sid in scara_ids(v.raw):
        s = f"scaras.{sid}"
        st = v.plain(f"{s}.station")
        rm = f"robot_models.{v.plain(f'{s}.model')}"
        name = f"Scara_{pascal(sid.removeprefix('scara_'))}"
        bx, by, bz = q(f"{s}.pose[0]"), q(f"{s}.pose[1]"), q(f"{s}.pose[2]")
        yaw = q(f"{s}.pose[3]")
        reach, stroke = q(f"{rm}.reach"), q(f"{rm}.z_stroke")
        j2 = q(f"{rm}.joint_limits.j2")
        k = rg.kin(spec, sid)
        if k is None or k.r_min is None:
            v.missing.append(f"{rm}.link1/link2 (the folded reach r_min of {sid})")
            continue
        r_min = derived(k.r_min, {f"{rm}.link1", f"{rm}.link2", f"{rm}.joint_limits.j2"})
        model = None
        step = q.plain(f"{rm}.step", "")
        try:
            model = vendor.load(step) if step else None
        except vendor.VendorError as exc:
            notes["warnings"].append(f"{sid}: vendor STEP not placed ({exc}); drawn as the "
                                     "layout.scara_base cylinder")
        if model is None and not step:
            notes["warnings"].append(f"{sid}: {rm}.step is not set; drawn as the "
                                     "layout.scara_base cylinder")
        if model is None:
            items.append(Item(f"{name}_Base", "robot", "Robots", (bx, by, floor_z + base_h / 2.0),
                              (base_d, base_d, base_h), cylinder=True))
        else:
            notes["vendor"][sid] = model.measured()
            mid = rm.removeprefix("robot_models.")
            if mid not in notes["vendor_warned"]:          # the capsules park_clearance uses
                notes["vendor_warned"].add(mid)
                try:
                    vendor.write_report(model, mid, cad.directory)
                except OSError as exc:
                    notes["warnings"].append(f"{mid}: cannot write vendor_{mid}.json ({exc})")
            # The STEP's shaft is at the top of its stroke: its lower end is the TCP's z top.
            mount = bz + stroke - derived(model.shaft_bottom / 1000.0, {f"{rm}.step"})
            items.append(Item(f"{name}_Base", "robot", "Robots",
                              (bx, by, (floor_z + mount) / 2.0), (base_d, base_d, mount - floor_z),
                              cylinder=True))
            keys = set(bx.keys | by.keys | bz.keys | yaw.keys | stroke.keys | j2.keys)
            for role, solid in vendor.placed(model, 0.0, j2.value, bx.value * 1000.0,
                                             by.value * 1000.0, mount.value * 1000.0, yaw.value):
                bb = solid.bounding_box()
                items.append(Item(f"{name}_{role}", "robot", "Robots",
                                  tuple(derived(c / 1000.0, keys) for c in tuple(bb.center())),
                                  tuple(derived(c / 1000.0, keys) for c in tuple(bb.size)),
                                  subgroup=name, solid=solid))
        items.append(Item(f"{name}_Envelope", "envelope", "Reference",
                          (bx, by, bz + stroke / 2.0), (reach * 2.0, reach * 2.0, stroke),
                          hollow=(r_min * 2.0, r_min * 2.0), cylinder=True))
        reach_rects.setdefault(st, []).append((bx - reach, bx + reach, by - reach, by + reach))
    for hid in sorted(v.raw.get("gang_heads") or {}):
        hb = rg.head_box(spec, hid, cad)
        if hb.rect is None:
            v.missing += [f"gang head {hid}: {u}" for u in hb.unresolved]
            continue
        g = f"gang_heads.{hid}"
        st = hb.station
        keys = {f"stations.{st}.pose[0]", f"stations.{st}.pose[1]", f"stations.{st}.pose[2]",
                f"{g}.stroke", f"{L}.gang_head_box[2]"} | (
            {f"{g}.slide.stroke", f"scaras.scara_{st.removeprefix('st_')}.base_side"}
            if hb.slide_from else set())
        box = [q(f"{L}.gang_head_box[{i}]") for i in range(3)]
        name = f"GangHead_{pascal(hid.removeprefix('gh_'))}"
        rect = hb.rect
        z0 = derived(hb.z0, keys)
        items.append(Item(name, "gang_head", "Robots",
                          (derived((rect.x0 + rect.x1) / 2.0, keys),
                           derived((rect.y0 + rect.y1) / 2.0, keys), z0 + box[2] / 2.0),
                          tuple(box), owner=hid))
        lo_y, hi_y = min(rect.y0, hb.working.y0), max(rect.y1, hb.working.y1)
        if hb.slide_from:
            items.append(Item(f"{name}_Slide", "envelope", "Reference",
                              (derived((rect.x0 + rect.x1) / 2.0, keys),
                               derived((lo_y + hi_y) / 2.0, keys), z0 + box[2] / 2.0),
                              (box[0], derived(hi_y - lo_y, keys), box[2])))
        reach_rects.setdefault(st, []).append(
            (derived(rect.x0, keys), derived(rect.x1, keys), derived(lo_y, keys),
             derived(hi_y, keys)))
        if hb.slide_from and not hb.frame:
            v.missing += [f"gang head {hid}: {u}" for u in hb.frame_unresolved]
        fkeys = keys | {f"{g}.slide.frame.post[0]", f"{g}.slide.frame.post[1]",
                        f"{g}.slide.frame.rail[0]", f"{g}.slide.frame.rail[1]", "line.work_height"}
        for part, fr, f0, f1 in hb.frame:
            items.append(Item(f"{name}_{part.capitalize()}", "slide_frame", "Robots",
                              (derived((fr.x0 + fr.x1) / 2.0, fkeys),
                               derived((fr.y0 + fr.y1) / 2.0, fkeys),
                               derived((f0 + f1) / 2.0, fkeys)),
                              (derived(fr.x1 - fr.x0, fkeys), derived(fr.y1 - fr.y0, fkeys),
                               derived(f1 - f0, fkeys)), owner=hid))
            reach_rects.setdefault(st, []).append(
                (derived(fr.x0, fkeys), derived(fr.x1, fkeys), derived(fr.y0, fkeys),
                 derived(fr.y1, fkeys)))

    # Feeders and buffers: floor-standing, footprint centred on the spec pose.
    for kind, coll, prefix in (("feeder", "feeders", "fd_"), ("buffer", "buffers", "buf_")):
        for fid in v.raw.get(coll) or {}:
            if lookup(v.raw, f"{coll}.{fid}.pose") is None:
                continue                            # a blow feeder rides its gang head
            fp = f"{L}.feeder_footprints.{fid}"
            w, d, h = q(f"{fp}[0]"), q(f"{fp}[1]"), q(f"{fp}[2]")
            yaw = (q(f"{coll}.{fid}.pose[3]")
                   if lookup(v.raw, f"{coll}.{fid}.pose[3]") is not None else None)
            items.append(Item(f"{kind.capitalize()}_{pascal(fid.removeprefix(prefix))}", kind,
                              f"{kind.capitalize()}s",
                              (q(f"{coll}.{fid}.pose[0]"), q(f"{coll}.{fid}.pose[1]"),
                               floor_z + h / 2.0), (w, d, h), yaw=yaw))
            # A source stands inside its station's guard too (D-013): it is in the cell.
            owner = lookup(v.raw, f"{coll}.{fid}.station")
            px, py = q(f"{coll}.{fid}.pose[0]"), q(f"{coll}.{fid}.pose[1]")
            reach_rects.setdefault(owner, []).append((px - w / 2.0, px + w / 2.0,
                                                      py - d / 2.0, py + d / 2.0))

    # Return paths (D-013): a strip from the put-down point along its yaw, floor to the put-down
    # plane, and a sink block at its far end.
    strips = []
    for rid in v.raw.get("return_paths") or {}:
        rp = f"return_paths.{rid}"
        rx, ry, rz = q(f"{rp}.pose[0]"), q(f"{rp}.pose[1]"), q(f"{rp}.pose[2]")
        ryaw = q(f"{rp}.pose[3]")
        length, width = q(f"{rp}.length"), q(f"{rp}.width")
        if v.missing:
            continue
        ux, uy = math.cos(ryaw.value), math.sin(ryaw.value)
        name = f"ReturnPath_{pascal(rid.removeprefix('rt_'))}"
        top = rz - floor_z
        items.append(Item(name, "return", "Returns",
                          (rx + length * (ux / 2.0), ry + length * (uy / 2.0),
                           floor_z + top / 2.0), (length, width, top), yaw=ryaw))
        sink = const("SINK_SIZE")
        far = length + sink / 2.0
        sh = top - const("SINK_H")
        items.append(Item(f"ReturnSink_{pascal(rid.removeprefix('rt_'))}", "return", "Returns",
                          (rx + far * ux, ry + far * uy, floor_z + sh / 2.0), (sink, sink, sh),
                          yaw=ryaw))
        # Inside the guard: the put-down end, where the SCARA works. The chute runs out through
        # the fence (a tunnel) to its sink, which stands outside with the sub-line.
        owner = lookup(v.raw, f"{rp}.station")
        reach_rects.setdefault(owner, []).append((rx - width / 2.0, rx + width / 2.0,
                                                  ry - width / 2.0, ry + width / 2.0))
        strips.append((rid, rx, ry, rz, ux, uy, length, width))

    # Guards (D-013): one continuous fence guard_offset outside everything that moves or is
    # worked at — SCARA reach discs, gang heads and their slides, feeders and buffers, the
    # put-down end of each return path — with a tunnel where the conveyor passes each end wall
    # and where a return chute leaves; and a partition wall across it at every milestone
    # boundary, half-way between the two stations either side, tunnelled for the conveyor.
    offset = q(f"{L}.guard_offset")
    tunnel_top = belt_top + pallet[2] + const("TUNNEL_CLEAR")
    wall = const("GUARD_T")
    gz = span(floor_z, floor_z + const("GUARD_H"))
    tz = span(floor_z, tunnel_top)
    guards = []
    rects = [r for rs in reach_rects.values() for r in rs]
    if rects and not v.missing:
        env = [(dmin(*(r[0] for r in rects)), dmax(*(r[1] for r in rects))),
               (dmin(*(r[2] for r in rects)), dmax(*(r[3] for r in rects)))]
        inner = [(env[i][0] - offset, env[i][1] + offset) for i in range(2)]
        gx, gy = span(inner[0][0] - wall, inner[0][1] + wall), span(inner[1][0] - wall,
                                                                   inner[1][1] + wall)
        cuts = [((gx[0], cy, tz[0]), (gx[1] + wall * 4.0, sy, tz[1]))]
        for rid, rx, ry, rz, ux, uy, length, width in strips:
            xs = [rx + a * length * ux - b * width / 2.0 * uy for a in (0, 1) for b in (-1, 1)]
            ys = [ry + a * length * uy + b * width / 2.0 * ux for a in (0, 1) for b in (-1, 1)]
            sx_, sy_ = span(dmin(*xs), dmax(*xs)), span(dmin(*ys), dmax(*ys))
            leaves = (sx_[0].value - sx_[1].value / 2 < inner[0][0].value
                      or sx_[0].value + sx_[1].value / 2 > inner[0][1].value
                      or sy_[0].value - sy_[1].value / 2 < inner[1][0].value
                      or sy_[0].value + sy_[1].value / 2 > inner[1][1].value)
            if not leaves:
                notes["warnings"].append(f"return path {rid} ends inside the fence: its sink "
                                         "(the sub-line) would stand in the guarded area")
                continue
            ctop = span(floor_z, rz + const("TUNNEL_CLEAR"))
            cuts.append(((sx_[0], sy_[0], ctop[0]), (sx_[1], sy_[1], ctop[1])))
        fence = Item("Guard_Fence", "guard", "Reference", (gx[0], gy[0], gz[0]),
                     (gx[1], gy[1], gz[1]),
                     hollow=(inner[0][1] - inner[0][0], inner[1][1] - inner[1][0]), cuts=cuts)
        guards.append(fence)
        items.append(fence)
        runs = sorted(((m, [st for st in members if st in pose])
                       for m, members in (v.raw.get("milestones") or {}).items()),
                      key=lambda mm: index.get(mm[1][0], 0) if mm[1] else 0)
        runs = [(m, members) for m, members in runs if members]
        for (ma, a_st), (mb, b_st) in zip(runs, runs[1:]):
            xp = (pose[a_st[-1]][0] + pose[b_st[0]][0]) / 2.0
            for st in a_st + b_st:
                for r in reach_rects.get(st, []):
                    if r[0].value - offset.value + rg.EPS < xp.value \
                            < r[1].value + offset.value - rg.EPS:
                        notes["warnings"].append(
                            f"partition {ma}|{mb} at x {xp.value:.3f} is inside guard_offset of "
                            f"{st}'s envelope ({r[0].value:.3f}..{r[1].value:.3f})")
                        break
            part = Item(f"Guard_Partition_{pascal(ma)}_{pascal(mb)}", "guard", "Reference",
                        (xp, gy[0], gz[0]), (wall, inner[1][1] - inner[1][0], gz[1]),
                        cuts=[((xp, cy, tz[0]), (wall * 4.0, sy, tz[1]))])
            guards.append(part)
            items.append(part)

    # Operator aisle: on its side, from the outermost guard outward, the length of the line.
    side = q.plain(f"{L}.operator_aisle.side")
    width = q(f"{L}.operator_aisle.width")
    if v.missing:
        return None
    lx = span(dmin(*(it.lo(0) for it in items)), dmax(*(it.hi(0) for it in items)))
    if side == "+y":
        edge = dmax(*(g.hi(1) for g in guards)) if guards else y1
        ay = span(edge, edge + width)
    else:
        edge = dmin(*(g.lo(1) for g in guards)) if guards else y0
        ay = span(edge - width, edge)
    az = span(floor_z, floor_z + const("AISLE_T"))
    aisle = Item("OperatorAisle", "aisle", "Reference", (lx[0], ay[0], az[0]), (lx[1], ay[1], az[1]))
    # `side` is not a length, but it decides where y is: carry its provenance on centre y.
    aisle_side = Dim(0.0, frozenset({f"{L}.operator_aisle.side"}),
                     v.is_placeholder(f"{L}.operator_aisle.side"))
    aisle.centre = (aisle.centre[0], aisle.centre[1] + aisle_side, aisle.centre[2])
    items.append(aisle)

    # Floor: under everything, FLOOR_MARGIN beyond it all round.
    margin = const("FLOOR_MARGIN")
    fx = span(dmin(*(it.lo(0) for it in items)) - margin, dmax(*(it.hi(0) for it in items)) + margin)
    fy = span(dmin(*(it.lo(1) for it in items)) - margin, dmax(*(it.hi(1) for it in items)) + margin)
    fz = span(floor_z - const("FLOOR_T"), floor_z)
    items.insert(0, Item("Floor", "floor", "Reference", (fx[0], fy[0], fz[0]), (fx[1], fy[1], fz[1])))
    return items


_STEP_CACHE: dict[str, object] = {}


def carrier_shape(part: str, cad: rg.CadHeights, notes: dict):
    """The carrier's own STEP (its generator's output), rest plane at z = 0, mm; its bbox as a
    box when that STEP is not built; None (and a warning) when neither exists."""
    path = CAD_OUT / ("dev" if cad.dev else "") / CARRIER_STEPS[part]
    if path.is_file():
        if str(path) not in _STEP_CACHE:
            _STEP_CACHE[str(path)] = Compound(import_step(str(path)).solids())
        return copy.deepcopy(_STEP_CACHE[str(path)])
    box = cad.bbox(part)
    if box is None:
        notes["warnings"].append(f"{part}: neither {path} nor its bbox report exists; not drawn")
        return None
    notes["warnings"].append(f"{part}: {path} not built; drawn as its bbox")
    lo, size = box["min"], box["size"]
    return Pos(*(lo[i] + size[i] / 2.0 for i in range(3))) * Box(*size)


# ---------------------------------------------------------------- park (D-014)

def park(v: Values, spec: LineSpec, cad: rg.CadHeights, items: list[Item], out_dir: Path | None
         ) -> dict:
    """Every SCARA that parks while its station's gang head works: its vendor STEP posed at
    park_pose against the head at its working position and its slide frame (boolean volumes),
    robot_geometry's capsule clearance next to it, and — when `out_dir` is given — the close-up
    <station>_park.png: the cell with that arm parked and the head in, everything else as built."""
    out: dict = {}
    for sid in scara_ids(v.raw):
        ends, starts = rg.park_steps(spec, sid)
        joints = rg.park_joints(spec, sid)
        if not ends and not starts:
            continue
        st = spec.scaras[sid].station
        row: dict = {"scara": sid, "park_pose": joints, "overlaps": [], "max_overlap_mm3": 0.0}
        out[st] = row
        if joints is None:
            row["error"] = f"scaras.{sid}.park_pose is null"
            continue
        m = spec.scaras[sid].model
        rm = spec.robot_models[m]
        k = rg.kin(spec, sid)
        try:
            model = vendor.load(rm.step) if rm.step else None
        except vendor.VendorError as exc:
            model, row["error"] = None, str(exc)
        pc = rg.park_clearance(spec, sid, joints, cad)
        row["capsule_clearance_m"] = pc["clearance_m"]
        row["capsule_nearest"] = pc["nearest"]
        if model is None or k is None:
            continue
        mount = k.z_hi - model.shaft_bottom / 1000.0
        arm = vendor.placed(model, joints[0], joints[1], k.base[0] * 1000.0,
                            k.base[1] * 1000.0, mount * 1000.0, k.yaw)
        envs, _ = rg.head_envelopes(spec, sid, cad)
        boxes = []
        for hid, env in envs.items():
            for part, rect, z0, z1 in env:
                boxes.append((hid, part, Pos((rect.x0 + rect.x1) * 500.0,
                                             (rect.y0 + rect.y1) * 500.0, (z0 + z1) * 500.0)
                              * Box((rect.x1 - rect.x0) * 1000.0, (rect.y1 - rect.y0) * 1000.0,
                                    (z1 - z0) * 1000.0)))
        for role, solid in arm:
            bb = solid.bounding_box()
            for hid, part, box in boxes:
                b = box.bounding_box()
                if (bb.min.X >= b.max.X or b.min.X >= bb.max.X or bb.min.Y >= b.max.Y
                        or b.min.Y >= bb.max.Y or bb.min.Z >= b.max.Z or b.min.Z >= bb.max.Z):
                    continue
                common = solid & box
                vol = sum(x.volume for x in common.solids()) if common is not None else 0.0
                if vol > VOLUME_TOL_MM3:
                    row["overlaps"].append({"arm": role, "head": hid, "part": part,
                                            "volume_mm3": round(vol, 1)})
        row["max_overlap_mm3"] = max((o["volume_mm3"] for o in row["overlaps"]), default=0.0)
        if out_dir is None:
            continue
        name = f"Scara_{pascal(sid.removeprefix('scara_'))}"
        x0 = spec.stations[st].pose[0]
        heads = {f"GangHead_{pascal(h.removeprefix('gh_'))}" for h in envs}
        shown = []
        for it in items:
            if it.group == "Reference" or it.shape is None:
                continue
            if it.subgroup == name or it.name in heads:
                continue                     # this arm folded, the head retracted: redrawn below
            if abs(it.centre[0].value - x0) > PARK_WINDOW:
                continue
            shown.append(it)
        for role, solid in arm:
            solid.label, solid.color = f"{name}_{role}_Park", Color(*colour_of("robot"), OPAQUE)
            shown.append(Item(f"{name}_{role}_Park", "robot", "Robots", (Dim(0.0),) * 3,
                              (Dim(0.0),) * 3, subgroup=f"{name} (parked)", shape=solid))
        for hid, part, box in boxes:
            if part.startswith("head"):
                box.label = f"GangHead_{pascal(hid.removeprefix('gh_'))} (working)"
                shown.append(Item(box.label, "gang_head", "Robots", (Dim(0.0),) * 3,
                                  (Dim(0.0),) * 3, shape=box))
        clear = pc["clearance_m"]
        title = (f"{st}: {sid} parked j1 {math.degrees(joints[0]):.1f} deg, j2 "
                 f"{math.degrees(joints[1]):.1f} deg; capsule clearance "
                 + (f"{clear * 1000:.0f} mm" if clear is not None else "unknown")
                 + f"; STEP overlap {row['max_overlap_mm3']:g} mm3")
        shot = render_png.render_panels(shown, colour_of, out_dir / f"{st}_park.png", title,
                                        window=((x0 - PARK_WINDOW) * 1000.0,
                                                (x0 + PARK_WINDOW) * 1000.0))
        row["render"] = shot.get("png") or shot.get("error")
    return out


# ---------------------------------------------------------------- checks

def allowed(a: str, b: str, owner_a: str | None = None, owner_b: str | None = None) -> bool:
    for x, y in ALLOWED_OVERLAPS:
        if {a, b} == {x, y} or (y == "*" and x in (a, b)):
            return True
    if owner_a is not None and owner_a == owner_b:
        return any(sorted((a, b)) == sorted(pair) for pair in ALLOWED_SAME_OWNER)
    return False


def interference(items: list[Item]) -> dict:
    """Every pair of solids: bbox prefilter, then the boolean. Returns the report section."""
    boxes = [it.shape.bounding_box() for it in items]
    out = {"pairs": 0, "bbox_candidates": 0, "allowed_overlaps": 0, "failures": [],
           "volume_tol_mm3": VOLUME_TOL_MM3,
           "allowed_rules": [f"{a} ∩ {b}" for a, b in ALLOWED_OVERLAPS]
           + [f"{a} ∩ {b} (same gang head)" for a, b in ALLOWED_SAME_OWNER]}
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            out["pairs"] += 1
            a, b = boxes[i], boxes[j]
            if (a.min.X >= b.max.X or b.min.X >= a.max.X or a.min.Y >= b.max.Y
                    or b.min.Y >= a.max.Y or a.min.Z >= b.max.Z or b.min.Z >= a.max.Z):
                continue
            out["bbox_candidates"] += 1
            if allowed(items[i].category, items[j].category, items[i].owner, items[j].owner):
                out["allowed_overlaps"] += 1
                continue
            common = items[i].shape & items[j].shape
            volume = sum(s.volume for s in common.solids()) if common is not None else 0.0
            if volume > VOLUME_TOL_MM3:
                out["failures"].append({"a": items[i].label, "b": items[j].label,
                                        "volume_mm3": round(volume, 1)})
    out["result"] = "FAIL" if out["failures"] else "PASS"
    return out


# ---------------------------------------------------------------- assembly, export, verify

def assemble(items: list[Item], groups: tuple[str, ...] = GROUPS,
            copy_shapes: bool = False) -> Compound:
    """The group/subgroup tree, in `groups` order. A group not listed is left out entirely — used
    to build the physical-only assembly, which has no Reference group at all.

    `copy_shapes`: building a Compound reparents each child shape to it, silently dropping it out
    of any Compound it was already a child of — so a second assemble() call over the same `items`
    (physical, after the full one) must work from copies, never the same `it.shape` objects, or
    the first tree's groups come back empty. The copy is `relocated()`: a new wrapper over the same
    OCC geometry, with no parent.
    """
    out = []
    for gname in groups:
        children, subs = [], {}
        for it in items:
            if it.group != gname:
                continue
            shape = relocated(it.shape) if copy_shapes else it.shape
            if it.subgroup:
                subs.setdefault(it.subgroup, []).append(shape)
            else:
                children.append(shape)
        for sname, shapes in subs.items():
            children.append(Compound(children=shapes, label=sname))
        if children:            # a milestone may have no feeder, say: an empty group is no group
            out.append(Compound(children=children, label=gname))
    return Compound(children=out, label="Line")


def rotate_tree(node: Compound) -> Compound:
    """A copy of `node` rotated EXPORT_ROT_X_DEG about X, for STEP export only. Leaves keep their
    label and colour; the items' own `.shape`s (used for interference, the SVG, layout.json) are
    never touched."""
    kids = list(getattr(node, "children", ()) or ())
    if not kids:
        return relocated(node, Rot(X=EXPORT_ROT_X_DEG))
    return Compound(children=[rotate_tree(k) for k in kids], label=node.label)


def tree(node, path=()) -> dict[str, tuple[str, ...]]:
    """{leaf label: (ancestor labels, root first)} of an assembly."""
    kids = list(getattr(node, "children", ()) or ())
    if not kids:
        return {node.label: path}
    out = {}
    for k in kids:
        out.update(tree(k, path + (node.label,)))
    return out


def count_leaves(node) -> int:
    kids = list(getattr(node, "children", ()) or ())
    return 1 if not kids else sum(count_leaves(k) for k in kids)


def leaves(node) -> list:
    kids = list(getattr(node, "children", ()) or ())
    return [node] if not kids else [leaf for k in kids for leaf in leaves(k)]


def write_step(assembly: Compound, path: Path) -> None:
    """export_step, with every solid's opacity written out as fully opaque.

    Each solid is exported at PROBE_ALPHA so OCCT emits SURFACE_STYLE_RENDERING_WITH_PROPERTIES +
    SURFACE_STYLE_TRANSPARENT (it writes neither at alpha 1.0), then every transparency value in
    the file is set to 0 — opaque — and the solids are put back at OPAQUE.
    """
    solids = leaves(assembly)
    keep = [s.color for s in solids]
    try:
        for s in solids:
            s.color = Color(*tuple(s.color)[:3], PROBE_ALPHA)
        export_step(assembly, str(path))
    finally:
        for s, c in zip(solids, keep):
            s.color = c
    text = path.read_text()
    text, n = TRANSPARENT_ENTITY.subn("SURFACE_STYLE_TRANSPARENT(0.)", text)
    if n < len(solids):
        raise OSError(f"expected a transparency entity per solid, wrote {n} for "
                      f"{len(solids)} solids")
    path.write_text(text)


def opacity(step: Path, back: Compound) -> dict:
    """Is every solid opaque? Two views of the same file, and `opaque` needs both.

    * the file itself: a SURFACE_STYLE_TRANSPARENT for every solid, every one 0. This is the
      authoritative check — it is what a viewer reads.
    * the re-import: alpha of every solid. build123d's import_step does NOT read transparency
      back (a solid written at 0.5 comes back at 1.0), so this alone cannot tell an opaque file
      from a translucent one; it only catches a colour lost on the way.
    """
    solids = leaves(back)
    alphas = {s.label: tuple(s.color)[3] if s.color else None for s in solids}
    values = [float(v) for v in TRANSPARENT_ENTITY.findall(step.read_text())]
    not_opaque = sorted(n for n, a in alphas.items() if a is None or abs(a - OPAQUE) > ALPHA_TOL)
    nonzero = sum(v != 0.0 for v in values)
    states = len(values) >= len(solids)
    return {"opaque": not not_opaque and states and not nonzero,
            "all_alpha_1": not not_opaque, "not_opaque": not_opaque,
            "min_alpha": min((a for a in alphas.values() if a is not None), default=None),
            "file_states_opacity": states, "transparent_entities": len(values),
            "transparent_entities_nonzero": nonzero}


def verify(step: Path, built: Compound) -> dict:
    back = import_step(str(step))
    want, got = tree(built), tree(back)
    return {"file": str(step), "built_solids": count_leaves(built),
            "reimported_solids": count_leaves(back),
            "names_match": sorted(want) == sorted(got),
            "hierarchy_preserved": want == got,
            "missing": sorted(set(want) - set(got)), "unexpected": sorted(set(got) - set(want)),
            "opacity": opacity(step, back)}


def write_svg(items: list[Item], path: Path) -> None:
    """Top view: one rectangle per solid footprint, in mm, +y up.

    Unlike the STEP, fill stays the category colour and a placeholder gets an orange outline:
    under --dev every station pose is a placeholder, so an all-orange plan would show nothing.
    """
    order = ["floor", "aisle", "guard", "envelope", "conveyor", "station", "pallet", "carrier",
             "robot", "gang_head", "feeder", "buffer", "return"]
    boxes = [(it, it.shape.bounding_box()) for it in items]
    fl = next(b for it, b in boxes if it.category == "floor")
    x0, x1, y0, y1 = fl.min.X, fl.max.X, fl.min.Y, fl.max.Y
    w, h = x1 - x0, y1 - y0
    font = max(w, h) / 250.0
    rows = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{x0:.1f} {-y1:.1f} {w:.1f} '
            f'{h + 6 * font:.1f}" width="1800" height="{1800 * (h + 6 * font) / w:.0f}">',
            f"<title>line layout, top view (mm, line frame, +y up)</title>"]
    for cat in order:
        for it, b in boxes:
            if it.category != cat:
                continue
            fill = hexrgb(colour_of(cat))
            edge = hexrgb(PLACEHOLDER_COLOUR) if it.placeholder else "#222"
            hollow = it.hollow is not None or cat == "envelope"
            if it.cylinder:
                c = b.center()
                ring = [it.size[0].value * 500.0] + ([it.hollow[0].value * 500.0]
                                                      if it.hollow else [])
                for r in ring:
                    style = (f'fill="none" stroke="{fill}" stroke-width="{font / 2:.1f}" '
                             f'stroke-dasharray="{font:.1f}"' if hollow else
                             f'fill="{fill}" stroke="{edge}" stroke-width="{font / 10:.1f}"')
                    rows.append(f'<circle cx="{c.X:.1f}" cy="{-c.Y:.1f}" r="{r:.1f}" {style}>'
                                f'<title>{it.label}</title></circle>')
                continue
            if hollow:
                style = (f'fill="none" stroke="{fill}" stroke-width="{font / 2:.1f}"'
                         + (f' stroke-dasharray="{font:.1f}"' if cat == "envelope" else ""))
            else:
                style = (f'fill="{fill}" stroke="{edge}" '
                         f'stroke-width="{font / (4 if it.placeholder else 10):.1f}"'
                         + (' fill-opacity="0.35"' if cat in ("floor", "aisle") else ""))
            rows.append(f'<rect x="{b.min.X:.1f}" y="{-b.max.Y:.1f}" width="{b.size.X:.1f}" '
                        f'height="{b.size.Y:.1f}" {style}><title>{it.label}</title></rect>')
    for it, b in boxes:
        if it.category in ("station", "feeder", "buffer") or it.name.endswith("_Envelope"):
            rows.append(f'<text x="{b.center().X:.1f}" y="{-b.max.Y - font / 2:.1f}" '
                        f'font-size="{font:.1f}" text-anchor="middle" font-family="sans-serif">'
                        f'{it.label}</text>')
    legend_y = -y0 + font * 2
    lx = x0 + font
    for cat in order + ["placeholder (outline, _PH)"]:
        fill = hexrgb(colour_of(cat)) if cat in CATEGORY_COLOUR else hexrgb(PLACEHOLDER_COLOUR)
        rows.append(f'<rect x="{lx:.1f}" y="{legend_y:.1f}" width="{font:.1f}" '
                    f'height="{font:.1f}" fill="{fill}"/><text x="{lx + font * 1.3:.1f}" '
                    f'y="{legend_y + font * 0.9:.1f}" font-size="{font:.1f}" '
                    f'font-family="sans-serif">{cat}</text>')
        lx += font * (len(cat) * 0.7 + 3)
    rows.append("</svg>")
    path.write_text("\n".join(rows) + "\n")


def hexrgb(rgb) -> str:
    return "#%02x%02x%02x" % tuple(int(c * 255) for c in rgb)


def solid_doc(it: Item) -> dict:
    b = it.shape.bounding_box()
    dims = {f"centre_{a}": it.centre[i].doc() for i, a in enumerate("xyz")}
    dims.update({f"size_{a}": it.size[i].doc() for i, a in enumerate("xyz")})
    if it.yaw is not None:
        dims["yaw"] = {**it.yaw.doc(), "value_mm": None, "value_rad": it.yaw.value}
    if it.hollow:
        dims.update({f"inner_{a}": it.hollow[i].doc() for i, a in enumerate("xy")})
    return {"name": it.name, "label": it.label, "group": it.group, "subgroup": it.subgroup,
            "category": it.category, "placeholder": it.placeholder,
            "bbox_mm": {"min": [round(c, 3) for c in tuple(b.min)],
                        "max": [round(c, 3) for c in tuple(b.max)],
                        "size": [round(c, 3) for c in tuple(b.size)]},
            "dims": dims}


def warnings_of(v: Values) -> list[str]:
    out = []
    top = (lookup(v.raw, "layout.conveyor.top_height") or 0.0) \
        + (lookup(v.raw, "layout.pallet_block.thickness") or 0.0)
    wh = lookup(v.raw, "line.work_height") or 0.0
    if abs(top - wh) > PALLET_DATUM_TOL:
        out.append(f"pallet top is {top:.3f} m above the floor (conveyor.top_height + "
                   f"pallet_block.thickness) but line.work_height puts the datum plane at "
                   f"{wh:.3f} m")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate the line layout STEP from the spec")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help=f"default {DEFAULT_OUT}")
    ap.add_argument("--report", type=Path, default=None,
                    help="default reports/layout.json (layout.dev.json with --dev)")
    ap.add_argument("--milestone", default=None,
                    help="draw only this milestone (D-012) into <out dir>/<NAME>/")
    ap.add_argument("--cad-reports", type=Path, default=None,
                    help=f"where cad_<part>.json live (default {rg.CAD_REPORTS})")
    ap.add_argument("--no-render", action="store_true",
                    help="skip the reports/render/<name>_{top,iso}.png renders")
    ap.add_argument("--no-reference", action="store_true",
                    help=f"the primary STEP ({DEFAULT_OUT.name}) drops the Reference group too; "
                         f"{PHYSICAL_NAME} (always written) is unaffected")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    out_dir = args.out.parent / "dev" if args.dev else args.out.parent
    if args.milestone:
        out_dir = out_dir / args.milestone
    step = out_dir / args.out.name
    report = args.report or report_path(
        f"layout.{args.milestone}" if args.milestone else "layout", args.dev)
    doc: dict = {"passed": False, "dev": args.dev, "generator": "tools/cad/layout.py",
                 "units": "mm", "frame": "line frame: origin = pallet datum at st_load_pallet, "
                 "Z up; floor at z = -line.work_height. Exported STEP only: rotated "
                 f"{EXPORT_ROT_X_DEG:g} deg about X so spec (x, y, z) -> STEP (x, z, -y) — "
                 "SolidWorks is Y-up, so its Top view is the plan view. layout.json, bbox_mm and "
                 "every check stay in the spec/line frame.",
                 "milestone": args.milestone,
                 "outputs": {}, "counts": {}, "interference": {}, "verify": {},
                 "verify_physical": {},
                 "constants_m": {k: globals()[k] for k in (
                     "FLOOR_T", "FLOOR_MARGIN", "AISLE_T", "GUARD_T", "GUARD_H", "TUNNEL_CLEAR",
                     "OUTLINE_RIM", "OUTLINE_H", "SINK_SIZE", "SINK_H")},
                 "placeholders": [], "missing": [], "warnings": [], "errors": [], "solids": []}

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
        for e in doc["errors"]:
            print(f"layout: FAIL — {e}", file=sys.stderr)
        return code

    try:
        merged = load_merged(args.spec, args.dev)
        if args.milestone:
            merged.raw = milestone_subset(merged.raw, args.milestone)
        spec = parse_spec(merged.raw)
    except SpecError as exc:
        doc["errors"] = exc.errors
        return finish(1)
    if spec.layout is None:
        doc["errors"].append(f"{args.spec.name} has no `layout:` section")
        return finish(1)

    v = Values(merged.raw, {p["path"] for p in merged.placeholders})
    notes: dict = {"warnings": [], "vendor": {}}
    items = plan(v, spec, rg.CadHeights(args.cad_reports, args.dev), notes)
    doc["vendor_steps"] = notes["vendor"]
    doc["placeholders"] = sorted(set(v.placeholders))
    if items is None:
        doc["missing"] = sorted(set(v.missing))
        doc["errors"].append(f"no value for: {', '.join(doc['missing'])} — fill the key(s) in, "
                             "or re-run with --dev")
        return finish(1)
    doc["warnings"] = warnings_of(v) + notes["warnings"]

    timing = doc.setdefault("timing_s", {})
    peak = doc.setdefault("peak_rss_mb", {})

    def rss(phase):
        peak[phase] = round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0)
        print(f"layout: {phase} done, peak {peak[phase]} MB", file=sys.stderr, flush=True)
    rss("plan")
    t0 = time.monotonic()
    for it in items:
        it.shape = build_shape(it)
    line = assemble(items, GROUPS)                                   # full, incl. Reference
    physical = assemble(items, PHYSICAL_GROUPS, copy_shapes=True)    # no Reference group at all
    primary = physical if args.no_reference else line
    doc["solids"] = [solid_doc(it) for it in items]
    doc["counts"] = {"solids": len(items),
                     "placeholder_solids": sum(it.placeholder for it in items),
                     "placeholder_values": len(doc["placeholders"]),
                     "by_group": {g: sum(it.group == g for it in items) for g in GROUPS}}

    timing["build"] = round(time.monotonic() - t0, 2)
    rss("build")
    t0 = time.monotonic()
    doc["interference"] = interference(items)
    timing["interference"] = round(time.monotonic() - t0, 2)
    rss("interference")
    t0 = time.monotonic()
    for f in doc["interference"]["failures"]:
        doc["errors"].append(f"interference: {f['a']} ∩ {f['b']} = {f['volume_mm3']} mm3")

    physical_step = out_dir / PHYSICAL_NAME
    try:
        out_dir.mkdir(parents=True, exist_ok=True)
        write_step(rotate_tree(primary), step)
        write_step(rotate_tree(physical), physical_step)
        write_svg(items, out_dir / SVG_NAME)
    except OSError as exc:
        doc["errors"].append(f"cannot write {out_dir}: {exc}")
        return finish(1)
    doc["outputs"] = {"step": str(step), "step_physical": str(physical_step),
                      "svg": str(out_dir / SVG_NAME)}
    timing["write"] = round(time.monotonic() - t0, 2)
    rss("write")
    t0 = time.monotonic()
    if not args.no_render:
        shots = render_png.render(items, colour_of, REPORTS / "render", args.milestone or "line")
        if "error" in shots:
            doc["warnings"].append(f"render: {shots['error']}")
        else:
            doc["outputs"]["render"] = shots

    doc["park"] = park(v, spec, rg.CadHeights(args.cad_reports, args.dev), items,
                       None if args.no_render else REPORTS / "render")
    for st, row in doc["park"].items():
        for o in row["overlaps"]:
            doc["errors"].append(f"park: {row['scara']} parked at {st}: {o['arm']} ∩ {o['head']} "
                                 f"{o['part']} = {o['volume_mm3']} mm3")
        if "error" in row:
            doc["warnings"].append(f"park {st}: {row['error']}")
    timing["render"] = round(time.monotonic() - t0, 2)
    rss("render")
    t0 = time.monotonic()
    doc["verify"] = verify(step, primary)
    if not doc["verify"]["hierarchy_preserved"]:
        manifest, files = out_dir / "manifest.txt", []
        for group in primary.children:
            path = out_dir / f"{step.stem}_{group.label}.step"
            write_step(rotate_tree(group), path)
            files.append({"group": group.label, **verify(path, group)})
        manifest.write_text("".join(f"{f['group']}\t{Path(f['file']).name}\t"
                                    f"{f['reimported_solids']} solids\n" for f in files))
        doc["verify"]["fallback"] = {"reason": "hierarchy did not survive the STEP round trip",
                                     "manifest": str(manifest), "files": files}
        doc["outputs"]["manifest"] = str(manifest)
        if not all(f["names_match"] for f in files):
            doc["errors"].append("per-group STEP files lost solids or names too; see verify")
    elif not doc["verify"]["names_match"] or \
            doc["verify"]["reimported_solids"] != doc["verify"]["built_solids"]:
        doc["errors"].append("re-imported STEP does not have the solids that were built")

    doc["verify_physical"] = verify(physical_step, physical)
    if not doc["verify_physical"]["hierarchy_preserved"] or \
            not doc["verify_physical"]["names_match"] or \
            doc["verify_physical"]["reimported_solids"] != doc["verify_physical"]["built_solids"]:
        doc["errors"].append(f"{PHYSICAL_NAME}: re-imported STEP does not have the solids that "
                             "were built, or lost the hierarchy")

    timing["verify"] = round(time.monotonic() - t0, 2)
    rss("verify")
    checked = ([doc["verify"]] + [f for f in doc["verify"].get("fallback", {}).get("files", [])]
              + [doc["verify_physical"]])
    for v in checked:
        op = v["opacity"]
        if not op["opaque"]:
            doc["errors"].append(
                f"{Path(v['file']).name}: not fully opaque — {len(op['not_opaque'])} solid(s) "
                f"re-import below alpha 1.0 (min {op['min_alpha']}), "
                f"{op['transparent_entities_nonzero']} non-zero transparency entit(ies), "
                f"{op['transparent_entities']} written"
                + ("" if op["file_states_opacity"] else " (fewer than one per solid)"))

    for w in doc["warnings"]:
        print(f"layout: WARN {w}", file=sys.stderr)
    if args.dev:
        print(f"layout: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{out_dir} is not a deliverable")
    i = doc["interference"]
    print(f"layout: {len(items)} solids ({doc['counts']['placeholder_solids']} placeholder), "
          f"{i['pairs']} pairs, {i['bbox_candidates']} bbox overlaps, "
          f"{i['allowed_overlaps']} allowed, {len(i['failures'])} interference(s) -> {step}")
    if doc["errors"]:
        return finish(1)
    op = doc["verify"]["opacity"]
    print(f"layout: OK — STEP re-imported with {doc['verify']['reimported_solids']} solids, "
          f"hierarchy preserved, all opaque ({op['transparent_entities']} transparency "
          f"entities, all 0; re-import alpha 1.0 on every solid) -> {report}")
    print(f"layout: {PHYSICAL_NAME} ({doc['verify_physical']['reimported_solids']} solids, no "
          f"Reference group) -> {physical_step}")
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
