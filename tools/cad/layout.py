"""Layout-level CAD assembly of the main line, from line_spec.yaml, for review in SolidWorks.

    tools/.venv/bin/python tools/cad/layout.py [--dev] [--no-reference] [--spec PATH] [--out FILE.step]

Simple solids in the line frame (origin = pallet datum at st_load_pallet, Z up; the floor is at
z = -line.work_height), exported in millimetres:

  Conveyor   Conveyor, Pallet_<n> (one block at station n's pose, n = its index), and
             Pallet_Slot_<n>_<k> (queued-pallet outlines between station n and the next)
  Stations   Station_<Name> (stop_unit_length x conveyor width, floor to belt top), and for a
             station with count > 1 one Station_<Name>_Cell<k> per cell on the test branch
  Gantries   Gantry_<n> > 4 posts and 2 beams along x — the physical frame only
  Feeders    Feeder_<Name> at its pose, sized by layout.feeder_footprints (a feeder with no pose —
             a blow feeder riding its gang head — has nothing to place)
  Buffers    Buffer_<Name>, the same
  Reference  non-physical solids: Floor (bbox of everything + 1 m), OperatorAisle (outside the
             outermost guard on its side), Guard_Gantry_<n> (a wall guard_offset outside each
             envelope, with a tunnel where the conveyor passes), Gantry_<n>_Envelope (the full
             axis travel)

Every dimension carries the spec keys it came from; a solid that used any placeholder is coloured
by its category exactly as a real one and its name ends in `_PH` — placeholder status never
changes colour. The numbers typed below are envelopes (floor slab, guard wall, outline rim),
listed in the report as constants — no spec value comes from them.

Interference: every pair of solids is intersected (bbox first, then the boolean). Only the pairs in
ALLOWED_OVERLAPS may share volume; anything else above VOLUME_TOL_MM3 is a FAIL naming both solids.

Axes: all of the above — layout.json, bbox_mm, the interference/straddle checks — stay in the spec
(line) frame, Z up. Only the exported STEP is rotated: SolidWorks is Y-up, so the whole assembly is
rotated EXPORT_ROT_X_DEG (-90 deg about X) right before writing, mapping spec (x, y, z) to STEP
(x, z, -y) so SolidWorks' Top view is the plan view.

Outputs: two STEP assemblies next to each other — line_layout.step (Line > Conveyor, Stations,
Gantries, Feeders, Buffers, Reference; `--no-reference` drops Reference from this one too) and
line_layout_physical.step (always written, always without Reference) — a top-view SVG, and
reports/layout.json with every solid's bbox and dimension provenance. Each STEP is re-imported and
its names and hierarchy compared with what was built; if line_layout.step's hierarchy does not
survive, one STEP per group and a manifest.txt are written instead.
`--dev` writes to <out dir>/dev/ and reports/layout.dev.json. The STEP is generated: edit the
spec, never the SolidWorks file.
"""

from __future__ import annotations

import argparse
import copy
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

from build123d import Box, Color, Compound, Pos, Rot, export_step, import_step

sys.path.insert(0, str(Path(__file__).resolve().parent))         # tools/cad: common
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: spec

from common import Values, lookup, pascal  # noqa: E402
from spec import (DEFAULT_SPEC, SpecError, add_dev_argument, gantry_ids,  # noqa: E402
                  load_merged, parse_spec, report_path)

DEFAULT_OUT = Path("/mnt/c/Users/Admin/Documents/drone-line-cad/60_layout/line_layout.step")
PHYSICAL_NAME = "line_layout_physical.step"   # the same assembly, Reference group dropped
SVG_NAME = "line_layout_top.svg"
GROUPS = ("Conveyor", "Stations", "Gantries", "Feeders", "Buffers", "Reference")
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
PALLET_DATUM_TOL = 0.001  # pallet top vs the datum plane: WARN beyond this
VOLUME_TOL_MM3 = 1.0     # shared volume below this is two faces touching, not an overlap

# Pairs of categories that may share volume. Everything else is a FAIL.
ALLOWED_OVERLAPS = (
    ("pallet", "conveyor"),      # a pallet (or a queued outline) rides on the belt
    ("station", "conveyor"),     # a station footprint straddles the belt
    ("envelope", "*"),           # a travel envelope is a volume the gantry sweeps, not a body
    ("floor", "*"),
)

# Every solid is always coloured by category, placeholder or not: placeholder status shows only
# in the `_PH` name suffix and in layout.json, never as a colour override.
CATEGORY_COLOUR = {
    "floor": "reference", "aisle": "reference", "guard": "reference", "envelope": "reference",
    "conveyor": "conveyor", "pallet": "pallet", "station": "station", "gantry": "gantry",
    "feeder": "feeder", "buffer": "buffer",
}
COLOURS = {                      # by colour bucket, RGB 0-1
    "reference": (0.80, 0.80, 0.82), "conveyor": (0.45, 0.45, 0.47), "pallet": (0.20, 0.45, 0.85),
    "station": (0.25, 0.65, 0.35), "gantry": (0.25, 0.25, 0.27), "feeder": (0.95, 0.80, 0.20),
    "buffer": (0.55, 0.25, 0.70),
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
    subgroup: str | None = None                 # Gantry_<n> inside Gantries
    yaw: Dim | None = None                      # radians about z
    hollow: tuple[Dim, Dim] | None = None       # inner x, y of a ring (outline, guard)
    cuts: list[tuple[tuple[Dim, Dim, Dim], tuple[Dim, Dim, Dim]]] = field(default_factory=list)
    shape: object = None

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


def box_mm(centre, size):
    return Pos(*(c.value * 1000.0 for c in centre)) * Box(*(s.value * 1000.0 for s in size))


def build_shape(it: Item):
    solid = Box(*(s.value * 1000.0 for s in it.size))
    if it.hollow:
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


def plan(v: Values) -> list[Item] | None:
    """Every solid, as Dims. None when a value the layout needs is null."""
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
    slots = v.plain(f"{L}.buffer_slots_between_stations")
    for a, b in zip(stations, stations[1:]):
        for k in range(1, int(slots or 0) + 1):
            f = k / (int(slots) + 1)
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

    # Gantries: the travel envelope, posts outside its corners, a beam along x on each side.
    post = q(f"{L}.gantry_frame.post_size")
    beam_h, beam_d = q(f"{L}.gantry_frame.beam_height"), q(f"{L}.gantry_frame.beam_depth")
    envelopes = {}
    for gid in gantry_ids(v.raw):
        g, n = f"gantries.{gid}", gid.removeprefix("gantry_")
        sub = f"Gantry_{n}"
        env = [(q(f"{g}.mount[{i}]") + q(f"{g}.axes.{a}.min"),
                q(f"{g}.mount[{i}]") + q(f"{g}.axes.{a}.max")) for i, a in enumerate("xyz")]
        envelopes[gid] = env
        ec = [span(*e) for e in env]
        items.append(Item(f"{sub}_Envelope", "envelope", "Reference",
                          tuple(c for c, _ in ec), tuple(s for _, s in ec)))
        pz = span(floor_z, env[2][1])
        corners = [(env[0][0] - post / 2.0, env[1][0] - post / 2.0),
                   (env[0][1] + post / 2.0, env[1][0] - post / 2.0),
                   (env[0][1] + post / 2.0, env[1][1] + post / 2.0),
                   (env[0][0] - post / 2.0, env[1][1] + post / 2.0)]
        for k, (px, py) in enumerate(corners, start=1):
            items.append(Item(f"{sub}_Post_{k}", "gantry", "Gantries", (px, py, pz[0]),
                              (post, post, pz[1]), subgroup=sub))
        bx = span(env[0][0] - post, env[0][1] + post)
        for k, py in enumerate((env[1][0] - post / 2.0, env[1][1] + post / 2.0), start=1):
            items.append(Item(f"{sub}_Beam_{k}", "gantry", "Gantries",
                              (bx[0], py, env[2][1] + beam_h / 2.0), (bx[1], beam_d, beam_h),
                              subgroup=sub))

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

    # Guards: a wall guard_offset outside each envelope, open where the belt passes through.
    offset = q(f"{L}.guard_offset")
    tunnel_top = belt_top + pallet[2] + const("TUNNEL_CLEAR")
    wall = const("GUARD_T")
    gz = span(floor_z, floor_z + const("GUARD_H"))
    guards = []
    for gid, env in envelopes.items():
        inner = [(env[i][0] - offset, env[i][1] + offset) for i in range(2)]
        gx, gy = span(inner[0][0] - wall, inner[0][1] + wall), span(inner[1][0] - wall,
                                                                   inner[1][1] + wall)
        tz = span(floor_z, tunnel_top)
        tunnel = ((gx[0], cy, tz[0]), (gx[1] + wall * 4.0, sy, tz[1]))
        it = Item(f"Guard_Gantry_{gid.removeprefix('gantry_')}", "guard", "Reference",
                  (gx[0], gy[0], gz[0]), (gx[1], gy[1], gz[1]),
                  hollow=(inner[0][1] - inner[0][0], inner[1][1] - inner[1][0]), cuts=[tunnel])
        guards.append(it)
        items.append(it)

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


# ---------------------------------------------------------------- checks

def allowed(a: str, b: str) -> bool:
    for x, y in ALLOWED_OVERLAPS:
        if {a, b} == {x, y} or (y == "*" and x in (a, b)):
            return True
    return False


def interference(items: list[Item]) -> dict:
    """Every pair of solids: bbox prefilter, then the boolean. Returns the report section."""
    boxes = [it.shape.bounding_box() for it in items]
    out = {"pairs": 0, "bbox_candidates": 0, "allowed_overlaps": 0, "failures": [],
           "volume_tol_mm3": VOLUME_TOL_MM3,
           "allowed_rules": [f"{a} ∩ {b}" for a, b in ALLOWED_OVERLAPS]}
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            out["pairs"] += 1
            a, b = boxes[i], boxes[j]
            if (a.min.X >= b.max.X or b.min.X >= a.max.X or a.min.Y >= b.max.Y
                    or b.min.Y >= a.max.Y or a.min.Z >= b.max.Z or b.min.Z >= a.max.Z):
                continue
            out["bbox_candidates"] += 1
            if allowed(items[i].category, items[j].category):
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
    (physical, after the full one) must work from independent deep copies, never the same
    `it.shape` objects, or the first tree's groups come back empty.
    """
    out = []
    for gname in groups:
        children, subs = [], {}
        for it in items:
            if it.group != gname:
                continue
            shape = copy.deepcopy(it.shape) if copy_shapes else it.shape
            if it.subgroup:
                subs.setdefault(it.subgroup, []).append(shape)
            else:
                children.append(shape)
        for sname, shapes in subs.items():
            children.append(Compound(children=shapes, label=sname))
        out.append(Compound(children=children, label=gname))
    return Compound(children=out, label="Line")


def rotate_tree(node: Compound) -> Compound:
    """A copy of `node` rotated EXPORT_ROT_X_DEG about X, for STEP export only. Leaves keep their
    label and colour; the items' own `.shape`s (used for interference, the SVG, layout.json) are
    never touched."""
    kids = list(getattr(node, "children", ()) or ())
    if not kids:
        rotated = Rot(X=EXPORT_ROT_X_DEG) * node
        rotated.label = node.label
        return rotated
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
    order = ["floor", "aisle", "guard", "envelope", "conveyor", "station", "pallet", "gantry",
             "feeder", "buffer"]
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
    ap.add_argument("--no-reference", action="store_true",
                    help=f"the primary STEP ({DEFAULT_OUT.name}) drops the Reference group too; "
                         f"{PHYSICAL_NAME} (always written) is unaffected")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    out_dir = args.out.parent / "dev" if args.dev else args.out.parent
    step = out_dir / args.out.name
    report = args.report or report_path("layout", args.dev)
    doc: dict = {"passed": False, "dev": args.dev, "generator": "tools/cad/layout.py",
                 "units": "mm", "frame": "line frame: origin = pallet datum at st_load_pallet, "
                 "Z up; floor at z = -line.work_height. Exported STEP only: rotated "
                 f"{EXPORT_ROT_X_DEG:g} deg about X so spec (x, y, z) -> STEP (x, z, -y) — "
                 "SolidWorks is Y-up, so its Top view is the plan view. layout.json, bbox_mm and "
                 "every check stay in the spec/line frame.",
                 "outputs": {}, "counts": {}, "interference": {}, "verify": {},
                 "verify_physical": {},
                 "constants_m": {k: globals()[k] for k in (
                     "FLOOR_T", "FLOOR_MARGIN", "AISLE_T", "GUARD_T", "GUARD_H", "TUNNEL_CLEAR",
                     "OUTLINE_RIM", "OUTLINE_H")},
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
        spec = parse_spec(merged.raw)
    except SpecError as exc:
        doc["errors"] = exc.errors
        return finish(1)
    if spec.layout is None:
        doc["errors"].append(f"{args.spec.name} has no `layout:` section")
        return finish(1)

    v = Values(merged.raw, {p["path"] for p in merged.placeholders})
    items = plan(v)
    doc["placeholders"] = sorted(set(v.placeholders))
    if items is None:
        doc["missing"] = sorted(set(v.missing))
        doc["errors"].append(f"no value for: {', '.join(doc['missing'])} — fill the key(s) in, "
                             "or re-run with --dev")
        return finish(1)
    doc["warnings"] = warnings_of(v)

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

    doc["interference"] = interference(items)
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
