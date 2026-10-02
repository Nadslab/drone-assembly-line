"""Where the robots stand, what they reach, and how high they must travel (D-012). Plain floats,
no ROS, no CAD.

Shared by check_line.py (`scara_reach`, `scara_clearance`, `safe_z`), estimate_moves.py and
tools/cad/layout.py, so the checks, the estimate and the drawing cannot disagree about a base, a
footprint or a height.

Line frame throughout: origin at the pallet datum of station 1, z = 0 on the datum plane.

Footprints (plan, metres): a SCARA base is a circle of `layout.scara_base.diameter` at
`spec.scara_base()`; a station is the pallet block at its pose; a feeder or buffer is its
`layout.feeder_footprints` rectangle at its pose; a gang head is a `layout.gang_head_box` at its
station — or, when it rides a slide (D-012), `slide.stroke` further out on the side opposite the
station's SCARA, where it waits while the SCARA works. A footprint whose size is null falls back
to its pose point.

Heights. A part's top in the stack is `z_in_stack + height`, height from its `envelope[2]` or
`dims` (box z, cylinder height). A build123d part (pallet, carriers) has no height in the spec:
its generator is the authority on that geometry (D-003), so it is read from the generator's report,
reports/cad_<part>.json (`.dev.json` under --dev), bbox z in the part's own frame, whose z = 0 is
the rest plane. A missing report makes that height UNRESOLVED, never a guess.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from spec import REPO, LineSpec, robot_served, scara_base, scara_id

CAD_REPORTS = REPO / "reports"
EPS = 1e-9


@dataclass(frozen=True)
class Rect:
    """An axis-aligned plan footprint, metres."""

    x0: float
    x1: float
    y0: float
    y1: float

    def gap(self, o: Rect) -> float:
        """Clear distance to `o`; negative when they overlap (the overlap depth)."""
        dx = max(o.x0 - self.x1, self.x0 - o.x1)
        dy = max(o.y0 - self.y1, self.y0 - o.y1)
        if dx >= 0 and dy >= 0:
            return (dx * dx + dy * dy) ** 0.5       # apart diagonally: corner to corner
        return max(dx, dy)                          # apart along one axis, or overlapping (< 0)

    def dist_from(self, x: float, y: float) -> float:
        """Distance from a point to the nearest point of the rectangle (0 inside)."""
        dx = max(self.x0 - x, 0.0, x - self.x1)
        dy = max(self.y0 - y, 0.0, y - self.y1)
        return math.hypot(dx, dy)

    def doc(self) -> list[float]:
        return [round(v, 6) for v in (self.x0, self.x1, self.y0, self.y1)]


def centred(x: float, y: float, w: float | None, d: float | None) -> Rect:
    """A w x d rectangle about (x, y); a point when the size is unknown."""
    w, d = w or 0.0, d or 0.0
    return Rect(x - w / 2.0, x + w / 2.0, y - d / 2.0, y + d / 2.0)


def conveyor_rect(spec: LineSpec) -> Rect | None:
    """The belt in plan: along every station pose, pitch/2 beyond the end stations (layout.py)."""
    if spec.layout is None or spec.layout.conveyor.width is None or spec.conveyor.pitch is None:
        return None
    poses = [s.pose for s in spec.stations.values()]
    if not poses or any(p[0] is None or p[1] is None for p in poses):
        return None
    w, half = spec.layout.conveyor.width / 2.0, spec.conveyor.pitch / 2.0
    return Rect(min(p[0] for p in poses) - half, max(p[0] for p in poses) + half,
                min(p[1] for p in poses) - w, max(p[1] for p in poses) + w)


def base_radius(spec: LineSpec) -> float | None:
    d = spec.layout.scara_base.diameter if spec.layout else None
    return None if d is None else d / 2.0


def station_rect(spec: LineSpec, sid: str) -> Rect | None:
    p = spec.stations[sid].pose
    if p[0] is None or p[1] is None:
        return None
    pb = spec.layout.pallet_block if spec.layout else None
    return centred(p[0], p[1], pb and pb.length, pb and pb.width)


def source_rect(spec: LineSpec, fid: str) -> Rect | None:
    """A feeder's or buffer's floor footprint at its pose."""
    f = spec.feeders.get(fid) or spec.buffers.get(fid)
    if f is None or f.pose is None or f.pose[0] is None or f.pose[1] is None:
        return None
    fp = spec.layout.feeder_footprints.get(fid) if spec.layout else None
    return centred(f.pose[0], f.pose[1], fp and fp[0], fp and fp[1])


def source_pose(spec: LineSpec, source: str | None):
    if source in spec.feeders:
        return spec.feeders[source].pose
    if source in spec.buffers:
        return spec.buffers[source].pose
    return None


# ---------------------------------------------------------------- heights

class CadHeights:
    """bbox of a build123d part, from its generator's report."""

    def __init__(self, directory: Path | None = None, dev: bool = False):
        self.directory = Path(directory) if directory else CAD_REPORTS
        self.dev = dev
        self._cache: dict[str, dict | None] = {}

    def path(self, part: str) -> Path:
        return self.directory / f"cad_{part}{'.dev' if self.dev else ''}.json"

    def bbox(self, part: str) -> dict | None:
        if part not in self._cache:
            try:
                box = json.loads(self.path(part).read_text()).get("bbox_mm") or None
            except (OSError, ValueError):
                box = None
            self._cache[part] = box if box and box.get("max") and box.get("size") else None
        return self._cache[part]


def part_extent(spec: LineSpec, pid: str, cad: CadHeights) -> tuple[float, float] | None:
    """(top above the rest plane, own height) of a part, metres; None when unknown."""
    p = spec.parts.get(pid)
    if p is None:
        return None
    if p.source == "build123d":
        box = cad.bbox(pid)
        return (box["max"][2] / 1000.0, box["size"][2] / 1000.0) if box else None
    if p.envelope is not None:
        h = p.envelope[2]
    elif p.dims is not None and p.shape == "box":
        h = p.dims[2]
    elif p.dims is not None and p.shape == "cylinder":
        h = p.dims[1]
    else:
        h = None
    if h is None:
        return None
    return (p.z_in_stack or 0.0) + h, h


def unknown_extent(spec: LineSpec, pid: str, cad: CadHeights) -> str:
    p = spec.parts.get(pid)
    if p is not None and p.source == "build123d":
        return f"height of {pid}: no bbox in {cad.path(pid).name} (run tools/cad/build_all.py)"
    return f"height of {pid}: envelope/dims null"


def stack_parts(spec: LineSpec, station: str, own: bool = True) -> list[str]:
    """What stands on the pallet at `station`: the pallet, and every part an active step places
    at an earlier station (by index) — and at this one too when `own`. Nothing is ever taken off:
    conservative. `own=False` is the stack a SCARA brings its part in over (D-012)."""
    idx = spec.stations[station].index
    out = ["pallet"] if "pallet" in spec.parts else []
    for s in spec.sequence:
        if s.status != "active" or not s.part or s.station not in spec.stations:
            continue
        at = spec.stations[s.station].index
        if (at < idx or (own and at == idx)) and s.part not in out:
            out.append(s.part)
    return out


def stack_top(spec: LineSpec, station: str, cad: CadHeights, own: bool = True
              ) -> tuple[float | None, list[str]]:
    """Top of the stack at `station` above its pose z, and what made it unknown."""
    tops, missing = [], []
    for pid in stack_parts(spec, station, own):
        ext = part_extent(spec, pid, cad)
        if ext is None:
            missing.append(f"{station}: {unknown_extent(spec, pid, cad)}")
        else:
            tops.append(ext[0])
    return (None if missing else max(tops, default=0.0)), missing


# ---------------------------------------------------------------- gang heads

@dataclass
class HeadBox:
    head: str
    station: str | None = None
    rect: Rect | None = None              # where it waits while a SCARA works (retracted)
    working: Rect | None = None           # over the station, driving
    z0: float | None = None               # line frame
    z1: float | None = None
    slide_from: str | None = None         # "+y"/"-y": the side it comes in from, if on a slide
    unresolved: list[str] = field(default_factory=list)


def head_box(spec: LineSpec, hid: str, cad: CadHeights) -> HeadBox:
    """The gang head body as an envelope: `layout.gang_head_box` over its station, its underside a
    ram stroke above the finished stack there (a horizontal head: on the stack). A head on a slide
    waits `slide.stroke` out on the side opposite the station's SCARA (D-012)."""
    out = HeadBox(hid)
    where = {s.station for s in spec.sequence if s.head == hid}
    station = where.pop() if len(where) == 1 else None
    out.station = station
    if station not in spec.stations:
        out.unresolved.append(f"gang_heads.{hid}: no single station drives it")
        return out
    h = spec.gang_heads[hid]
    pose = spec.stations[station].pose
    box = spec.layout.gang_head_box if spec.layout else None
    if box is None or None in box:
        out.unresolved.append("layout.gang_head_box is null")
    if None in pose[:3]:
        out.unresolved.append(f"stations.{station}.pose is null")
    top, missing = stack_top(spec, station, cad)
    out.unresolved += missing
    lift = h.stroke if h.direction == "down" else 0.0
    if lift is None:
        out.unresolved.append(f"gang_heads.{hid}.stroke is null")
    dy = 0.0
    if h.slide is not None:
        sc = spec.scaras.get(scara_id(station))
        if sc is None or sc.base_side is None:
            out.unresolved.append(f"gang_heads.{hid}.slide: the side of {station}'s SCARA is "
                                  "not known")
        elif h.slide.stroke is None:
            out.unresolved.append(f"gang_heads.{hid}.slide.stroke is null")
        else:
            out.slide_from = "-y" if sc.base_side == "+y" else "+y"
            dy = h.slide.stroke * (-1.0 if sc.base_side == "+y" else 1.0)
    if out.unresolved:
        return out
    out.working = centred(pose[0], pose[1], box[0], box[1])
    out.rect = centred(pose[0], pose[1] + dy, box[0], box[1])
    out.z0 = pose[2] + top + lift
    out.z1 = out.z0 + box[2]
    return out


# ---------------------------------------------------------------- safe_z

@dataclass
class SafeZ:
    scara: str
    value: float | None = None          # line frame z the SCARA's horizontal moves travel at
    tallest: tuple[str, float] | None = None
    hang: tuple[str, float] | None = None
    items: list[tuple[str, float]] = field(default_factory=list)   # (label, top z) in reach
    unresolved: list[str] = field(default_factory=list)

    def doc(self) -> dict:
        return {"safe_z": None if self.value is None else round(self.value, 4),
                "tallest_item": self.tallest and [self.tallest[0], round(self.tallest[1], 4)],
                "hang": self.hang and [self.hang[0], round(self.hang[1], 4)],
                "items": [[n, round(z, 4)] for n, z in self.items],
                "unresolved": self.unresolved}


def in_reach(spec: LineSpec, sid: str, rect: Rect) -> bool | None:
    """Does any of `rect` lie within r_max of the base? None while the base or reach is null."""
    base, r = scara_base(spec, sid), spec.scaras[sid].reach.r_max
    if base is None or r is None:
        return None
    return rect.dist_from(base[0], base[1]) <= r + EPS


def safe_z(spec: LineSpec, sid: str, cad: CadHeights) -> SafeZ:
    """Tallest item in the SCARA's reach envelope + tallest part it carries + z_margin.

    Items in reach (any of their footprint within r_max of the base): every station — at its pose
    z plus the stack standing there, its own station's stack as it is *before* the SCARA places on
    it (the part it brings in is the hang, not an item) — every feeder and buffer at its pick
    plane (pose z, the top of what it presents, D-004), and every gang head body where it waits
    while a SCARA works (retracted on its slide, D-012).
    """
    out = SafeZ(sid)
    sc = spec.scaras[sid]
    if scara_base(spec, sid) is None or sc.reach.r_max is None:
        out.unresolved.append(f"{sid}: base or reach.r_max is null")
        return out
    for st_id, st in spec.stations.items():
        rect = station_rect(spec, st_id)
        if rect is None or not in_reach(spec, sid, rect):
            continue
        if st.pose[2] is None:
            out.unresolved.append(f"stations.{st_id}.pose[2] is null")
            continue
        own = st_id != sc.station
        top, missing = stack_top(spec, st_id, cad, own=own)
        out.unresolved += missing
        if top is not None:
            out.items.append((f"station {st_id}" + ("" if own else " (before its place)"),
                              st.pose[2] + top))
    for coll, kind in ((spec.feeders, "feeder"), (spec.buffers, "buffer")):
        for fid, f in coll.items():
            rect = source_rect(spec, fid)
            if rect is None or not in_reach(spec, sid, rect):
                continue
            if f.pose[2] is None:
                out.unresolved.append(f"{kind} {fid}: pose z is null")
                continue
            out.items.append((f"{kind} {fid}", f.pose[2]))
    for hid in spec.gang_heads:
        hb = head_box(spec, hid, cad)
        if hb.rect is None:
            # Only a head that could be in reach matters; without its box, judge by its station.
            st = hb.station and spec.stations.get(hb.station)
            near = st and station_rect(spec, hb.station)
            if near is not None and in_reach(spec, sid, near) is not False:
                out.unresolved += hb.unresolved
            continue
        if in_reach(spec, sid, hb.rect):
            where = "retracted on its slide" if hb.slide_from else "over its station"
            out.items.append((f"gang head {hid} ({where})", hb.z1))
    hangs = []
    for s in spec.sequence:
        if s.status == "active" and robot_served(s) and s.station == sc.station and s.part:
            ext = part_extent(spec, s.part, cad)
            if ext is None:
                out.unresolved.append(f"{s.id}: {unknown_extent(spec, s.part, cad)}")
            else:
                hangs.append((s.part, ext[1]))
    margin = spec.scara_defaults.z_margin
    if margin is None:
        out.unresolved.append("scara_defaults.z_margin is null")
    if not out.items:
        out.unresolved.append(f"{sid}: nothing in its reach envelope")
    if out.unresolved:
        return out
    out.tallest = max(out.items, key=lambda it: it[1])
    out.hang = max(hangs, key=lambda h: h[1]) if hangs else ("nothing", 0.0)
    out.value = out.tallest[1] + out.hang[1] + margin
    return out
