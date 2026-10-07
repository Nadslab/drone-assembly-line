"""Where the robots stand, what they reach, and how high they must travel (D-012, D-013). Plain
floats, no ROS, no CAD.

Shared by check_line.py (`scara_reach`, `scara_clearance`, `safe_z`, `park_clearance`,
`slide_clearance`, `path_clearance`, `refill_clearance`, `payload`, `wrist_inertia`),
estimate_moves.py, choose_park.py and
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

The arm (D-014). The parked arm is checked as capsules sized from the vendor STEP: one per link
(J1 -> J2, J2 -> the Z shaft), a disc for the shaft and one for the end-effector's tools, each
over its z range. tools/cad/vendor.py measures them into reports/vendor_<robot model>.json (no
`.dev`: the STEP is not a placeholder); a missing report makes the park clearance UNRESOLVED.
D-015: the parked arm is held clear in plan; the same capsules, the shaft and tools following the
TCP's z, are swept along every joint path against the waiting heads and their slide frames
(`path_clearance`), and the parked arm and its park legs against the refill column of every
buffer a source refills at the station (`refill_clearance`); D-016: of every magazine or nest an
operator refills there too, in plan, and the station's gang head with them.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from spec import (REPO, LineSpec, ParkPose, robot_served, scara_base, scara_id, scara_model,  # noqa: F401
                  scara_side, scara_yaw)

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


def return_rect(spec: LineSpec, rid: str) -> Rect | None:
    """A return path's plan footprint: `width` across, from its put-down point `length` along
    its yaw (D-013). Axis-aligned bounds of the rotated strip."""
    rp = spec.return_paths.get(rid)
    if rp is None or rp.pose is None or None in rp.pose or rp.length is None \
            or rp.width is None:
        return None
    x, y, _, yaw = rp.pose
    ux, uy = math.cos(yaw), math.sin(yaw)
    w = rp.width / 2.0
    pts = [(x + a * ux * rp.length - b * uy * w, y + a * uy * rp.length + b * ux * w)
           for a in (0.0, 1.0) for b in (-1.0, 1.0)]
    return Rect(min(p[0] for p in pts), max(p[0] for p in pts),
                min(p[1] for p in pts), max(p[1] for p in pts))


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

    def vendor(self, model: str) -> dict | None:
        """The arm capsules tools/cad/vendor.py measured in a robot model's STEP (D-014)."""
        key = f"vendor:{model}"
        if key not in self._cache:
            try:
                doc = json.loads((self.directory / f"vendor_{model}.json").read_text())
            except (OSError, ValueError):
                doc = None
            self._cache[key] = doc if doc and doc.get("capsules") else None
        return self._cache[key]

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


def stack_parts(spec: LineSpec, station: str, own: bool = True,
                before_step: str | None = None) -> list[str]:
    """What stands on the pallet at `station`: the pallet, and every part an active step places
    at an earlier station (by index) — and at this one too when `own` — less every part a
    `remove` step lifts off before it (D-013: a carrier rides along until it is removed).
    `before_step` (with `own`) stops at that step of this station: the stack a SCARA brings that
    step's part in over. `own=False` is the stack as the pallet arrives. A milestone subset starts
    from what the pallet brings in (`pallet_in`)."""
    idx = spec.stations[station].index
    out = ["pallet"] if "pallet" in spec.parts else []
    out += [p for p in spec.pallet_in or [] if p not in out]   # brought in from upstream
    for s in spec.sequence:
        if s.status != "active" or not s.part or s.station not in spec.stations:
            continue
        at = spec.stations[s.station].index
        if at == idx and s.id == before_step:
            break
        if not (at < idx or (own and at == idx)):
            continue
        if s.capability == "remove":
            if s.part in out:
                out.remove(s.part)
        elif s.part not in out:
            out.append(s.part)
    return out


def stack_top(spec: LineSpec, station: str, cad: CadHeights, own: bool = True,
              before_step: str | None = None) -> tuple[float | None, list[str]]:
    """Top of the stack at `station` above its pose z, and what made it unknown."""
    tops, missing = [], []
    for pid in stack_parts(spec, station, own, before_step):
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
    # The slide frame (D-014): [(name, rect, z0, z1)] for "Rail" and "Post"; frame_unresolved says
    # why it is missing (the head itself can still be placed without it).
    frame: list = field(default_factory=list)
    frame_unresolved: list[str] = field(default_factory=list)

    def working_envelope(self) -> list:
        """[(name, rect, z0, z1)]: the head at its working position plus its slide frame."""
        if self.working is None:
            return []
        return [("head (working)", self.working, self.z0, self.z1)] + list(self.frame)


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
    pose = h.pose or spec.stations[station].pose      # its working pose (D-013 offset)
    box = spec.layout.gang_head_box if spec.layout else None
    if box is None or None in box:
        out.unresolved.append("layout.gang_head_box is null")
    if None in pose[:3]:
        out.unresolved.append(f"gang_heads.{hid}.pose (its station {station} and offset) is null")
    top, missing = stack_top(spec, station, cad)
    out.unresolved += missing
    lift = h.stroke if h.direction == "down" else 0.0
    if lift is None:
        out.unresolved.append(f"gang_heads.{hid}.stroke is null")
    dy = 0.0
    if h.slide is not None:
        sid = scara_id(station)
        side = scara_side(spec, sid) if sid in spec.scaras else None
        if side is None:
            out.unresolved.append(f"gang_heads.{hid}.slide: the side of {station}'s SCARA is "
                                  "not known")
        elif h.slide.stroke is None:
            out.unresolved.append(f"gang_heads.{hid}.slide.stroke is null")
        else:
            out.slide_from = "-y" if side == "+y" else "+y"
            dy = h.slide.stroke * (-1.0 if side == "+y" else 1.0)
    if out.unresolved:
        return out
    out.working = centred(pose[0], pose[1], box[0], box[1])
    out.rect = centred(pose[0], pose[1] + dy, box[0], box[1])
    out.z0 = pose[2] + top + lift
    out.z1 = out.z0 + box[2]
    if h.slide is not None:
        _slide_frame(spec, hid, out, pose, box, dy)
    return out


def _slide_frame(spec: LineSpec, hid: str, out: HeadBox, pose, box, dy: float) -> None:
    """The rail and post that carry a head on its slide (D-014). The rail runs along the slide
    axis from the head's outer face at its working position to the far side of the post, centred
    on the head's top face (the head's carriage runs in it); the post stands beyond the retracted
    head, from the floor to the rail's top."""
    fr = spec.gang_heads[hid].slide.frame
    wh = spec.line.work_height
    if fr is None or None in fr.post or None in fr.rail:
        out.frame_unresolved.append(f"gang_heads.{hid}.slide.frame (post, rail) is null")
        return
    if wh is None:
        out.frame_unresolved.append("line.work_height is null (the post stands on the floor)")
        return
    sign = 1.0 if dy >= 0 else -1.0
    y_work = pose[1] + sign * box[1] / 2.0             # the head's outer face, working
    y_back = y_work + dy                               # the same face, retracted
    y_end = y_back + sign * fr.post[1]                 # the post's far face
    rail_z0, rail_z1 = out.z1 - fr.rail[1] / 2.0, out.z1 + fr.rail[1] / 2.0
    out.frame = [
        ("rail", Rect(pose[0] - fr.rail[0] / 2.0, pose[0] + fr.rail[0] / 2.0,
                      min(y_work, y_end), max(y_work, y_end)), rail_z0, rail_z1),
        ("post", Rect(pose[0] - fr.post[0] / 2.0, pose[0] + fr.post[0] / 2.0,
                      min(y_back, y_end), max(y_back, y_end)), -wh, rail_z1)]


# ---------------------------------------------------------------- kinematics (D-013)

@dataclass
class Kin:
    """One SCARA's arm in the line frame: base, the direction j1 = 0 points, the links and the
    joint limits of its robot model. `links` is None while link1/link2 are null."""

    sid: str
    base: tuple[float, float, float]
    yaw: float
    reach: float
    z_lo: float                       # TCP z range, line frame: base z .. base z + z_stroke
    z_hi: float
    j1_lim: float
    j2_lim: float
    links: tuple[float, float] | None

    @property
    def r_min(self) -> float | None:
        """Closest the TCP comes to the base: the elbow folded to its j2 limit."""
        if self.links is None:
            return None
        l1, l2 = self.links
        return math.sqrt(max(l1 * l1 + l2 * l2 + 2.0 * l1 * l2 * math.cos(self.j2_lim), 0.0))


def kin(spec: LineSpec, sid: str) -> Kin | None:
    """None while the base pose is null."""
    base, yaw = scara_base(spec, sid), scara_yaw(spec, sid)
    if base is None or yaw is None:
        return None
    m = scara_model(spec, sid)
    links = (m.link1, m.link2) if m.link1 is not None and m.link2 is not None else None
    return Kin(sid, base, yaw, m.reach, base[2], base[2] + m.z_stroke, m.joint_limits.j1,
               m.joint_limits.j2, links)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ik(k: Kin, x: float, y: float) -> tuple[list[tuple[float, float]], str | None]:
    """Joint solutions (j1, j2) that put the TCP at plan point (x, y) inside the joint limits,
    elbow either way; and why there are none. Needs `k.links`."""
    l1, l2 = k.links
    dx, dy = x - k.base[0], y - k.base[1]
    r = math.hypot(dx, dy)
    if r > l1 + l2 + EPS:
        return [], f"{r:.3f} m from the base, beyond reach {l1 + l2:g}"
    c = max(-1.0, min(1.0, (r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)))
    theta = _wrap(math.atan2(dy, dx) - k.yaw)
    sols, why = [], []
    for j2 in {math.acos(c), -math.acos(c)}:
        j1 = _wrap(theta - math.atan2(l2 * math.sin(j2), l1 + l2 * math.cos(j2)))
        if abs(j2) > k.j2_lim + EPS:
            why.append(f"j2 {math.degrees(j2):.1f}° outside ±{math.degrees(k.j2_lim):g}° "
                       f"({r:.3f} m from the base, inside r_min {k.r_min:.3f})")
        elif abs(j1) > k.j1_lim + EPS:
            why.append(f"j1 {math.degrees(j1):.1f}° outside ±{math.degrees(k.j1_lim):g}°")
        else:
            sols.append((j1, j2))
    return sols, (None if sols else "; ".join(dict.fromkeys(why)))


def fk(k: Kin, j1: float, j2: float) -> tuple[float, float]:
    l1, l2 = k.links
    a = k.yaw + j1
    return (k.base[0] + l1 * math.cos(a) + l2 * math.cos(a + j2),
            k.base[1] + l1 * math.sin(a) + l2 * math.sin(a + j2))


PATH_SAMPLES = 48


def leg_ends(k: Kin, a, b, ja=None, jb=None
             ) -> tuple[tuple[float, float], tuple[float, float]] | None:
    """A leg's joint poses at its two ends: from joint pose `ja` (else plan point `a`) to `jb`
    (else `b`). An end given as joints is held; the other end takes its IK solution on the same
    elbow side with the least j1 travel (both plan points: the elbow solution valid at both ends
    with the least joint travel). None when no such solution exists."""
    if ja is not None and jb is not None:
        return ja, jb
    if ja is None and jb is None:
        sa, _ = ik(k, *a)
        sb, _ = ik(k, *b)
        pairs = [(p, q) for p in sa for q in sb if (p[1] >= 0) == (q[1] >= 0)]
        if not pairs:
            return None
        return min(pairs, key=lambda pq: abs(pq[0][0] - pq[1][0]) + abs(pq[0][1] - pq[1][1]))
    fixed = ja if ja is not None else jb
    other, _ = ik(k, *(b if ja is not None else a))
    other = [o for o in other if (o[1] >= 0) == (fixed[1] >= 0)]
    if not other:
        return None
    o = min(other, key=lambda o: abs(o[0] - fixed[0]))
    return (ja, o) if ja is not None else (o, jb)


def leg_joints(k: Kin, a, b, ja=None, jb=None) -> list[tuple[float, float]] | None:
    """The joint poses along a leg (`leg_ends`, j1 and j2 interpolated together)."""
    ends = leg_ends(k, a, b, ja, jb)
    if ends is None:
        return None
    p, q = ends
    return [(p[0] + (q[0] - p[0]) * t / PATH_SAMPLES, p[1] + (q[1] - p[1]) * t / PATH_SAMPLES)
            for t in range(PATH_SAMPLES + 1)]


def leg_path(k: Kin, a, b, ja=None, jb=None) -> list[tuple[float, float]] | None:
    """A leg's plan path (TCP points along `leg_joints`)."""
    js = leg_joints(k, a, b, ja, jb)
    return None if js is None else [fk(k, *j) for j in js]


def joint_limits_from_tcp(k: Kin, v: float, a: float) -> dict[str, tuple[float, float]]:
    """Angular speed and acceleration limits of J1 and J2 (D-015), from the TCP's v_max/a_max at
    each joint's link radius: J1 turns the whole arm, so its radius is link1 + link2; J2 turns
    link2 alone. {"j1": (w, alpha), "j2": (w, alpha)}, rad/s and rad/s^2."""
    l1, l2 = k.links
    return {"j1": (v / (l1 + l2), a / (l1 + l2)), "j2": (v / l2, a / l2)}


def home(k: Kin, station_xy: tuple[float, float]) -> tuple[float, float] | None:
    """The TCP folded in to r_min, straight out from the base towards its station."""
    if k.r_min is None:
        return None
    dx, dy = station_xy[0] - k.base[0], station_xy[1] - k.base[1]
    n = math.hypot(dx, dy) or 1.0
    return k.base[0] + dx / n * k.r_min, k.base[1] + dy / n * k.r_min


# ---------------------------------------------------------------- end-effector and carried parts

def tool_radius(spec: LineSpec, tid: str) -> float | None:
    """How far a tool reaches out from the J4 axis in plan: offset_r + its half-diagonal."""
    t = spec.tools.get(tid)
    if t is None or None in t.dims[:2] or t.offset_r is None:
        return None
    return t.offset_r + math.hypot(t.dims[0], t.dims[1]) / 2.0


def end_effector_radius(spec: LineSpec, sid: str) -> float | None:
    """The disc the whole end-effector sweeps turning about J4 (the dual-tool switch); 0 for a
    bare flange (no tool listed)."""
    rs = [tool_radius(spec, t) for t in spec.scaras[sid].tools]
    return None if None in rs else max(rs, default=0.0)


def carried_parts(spec: LineSpec, pid: str) -> list[str]:
    """The part and everything that rides in it (`carried_by`), e.g. carrier_spider + spider."""
    out = [pid]
    for name, p in spec.parts.items():
        if p.carried_by == pid and name not in out:
            out += carried_parts(spec, name)
    return out


def part_plan(spec: LineSpec, pid: str, cad: CadHeights) -> tuple[float, float] | None:
    """A part's plan size [x, y], metres: envelope, box dims, cylinder diameter, or the build123d
    generator's bbox."""
    p = spec.parts.get(pid)
    if p is None:
        return None
    if p.source == "build123d":
        box = cad.bbox(pid)
        return (box["size"][0] / 1000.0, box["size"][1] / 1000.0) if box else None
    if p.envelope is not None and None not in p.envelope[:2]:
        return p.envelope[0], p.envelope[1]
    if p.dims is not None and p.shape == "box" and None not in p.dims[:2]:
        return p.dims[0], p.dims[1]
    if p.dims is not None and p.shape == "cylinder" and p.dims[0] is not None:
        return 2.0 * p.dims[0], 2.0 * p.dims[0]
    return None


def part_radius(spec: LineSpec, pid: str, cad: CadHeights) -> float | None:
    sizes = [part_plan(spec, q, cad) for q in carried_parts(spec, pid)]
    return None if None in sizes else max(math.hypot(*s) / 2.0 for s in sizes)


def part_mass(spec: LineSpec, pid: str) -> float | None:
    """Mass lifted with the part: it and everything it carries."""
    ms = [spec.parts[q].mass for q in carried_parts(spec, pid)]
    return None if None in ms else sum(ms)


# ---------------------------------------------------------------- safe_z (path based, D-013)

@dataclass
class SafeZ:
    scara: str
    value: float | None = None          # line frame z the SCARA's horizontal moves travel at
    tallest: tuple[str, float] | None = None
    hang: tuple[str, float] | None = None
    items: list[tuple[str, float]] = field(default_factory=list)   # (label, top z) under a path
    unresolved: list[str] = field(default_factory=list)
    steps: dict = field(default_factory=dict)       # step id -> its own SafeZ doc
    switch: bool = False                            # a dual-tool switch precedes the step

    def doc(self) -> dict:
        out = {"safe_z": None if self.value is None else round(self.value, 4),
               "tallest_item": self.tallest and [self.tallest[0], round(self.tallest[1], 4)],
               "hang": self.hang and [self.hang[0], round(self.hang[1], 4)],
               "items": [[n, round(z, 4)] for n, z in self.items],
               "unresolved": self.unresolved}
        if self.steps:
            out["steps"] = self.steps
        return out


def in_reach(spec: LineSpec, sid: str, rect: Rect) -> bool | None:
    """Does any of `rect` lie within the model's reach of the base? None while the base is null."""
    base = scara_base(spec, sid)
    if base is None:
        return None
    return rect.dist_from(base[0], base[1]) <= scara_model(spec, sid).reach + EPS


def _plan_items(spec: LineSpec, sid: str, step, cad: CadHeights
                ) -> tuple[list[tuple[str, Rect, float]], list[str]]:
    """Everything a SCARA's tool could pass over, with its plan footprint and top z: every
    station's stack (its own as it stands before `step` places), every feeder and buffer at its
    pick plane, and every gang head where it waits while a SCARA works."""
    out, missing = [], []
    sc = spec.scaras[sid]
    for st_id, st in spec.stations.items():
        rect = station_rect(spec, st_id)
        if rect is None or not in_reach(spec, sid, rect):
            continue
        if st.pose[2] is None:
            missing.append(f"stations.{st_id}.pose[2] is null")
            continue
        own = st_id == sc.station
        top, miss = stack_top(spec, st_id, cad, own=True,
                              before_step=step.id if own else None)
        missing += miss
        if top is not None:
            out.append((f"station {st_id}" + (f" (before {step.id})" if own else ""), rect,
                        st.pose[2] + top))
    for coll, kind in ((spec.feeders, "feeder"), (spec.buffers, "buffer")):
        for fid, f in coll.items():
            rect = source_rect(spec, fid)
            if rect is None or not in_reach(spec, sid, rect):
                continue
            if f.pose[2] is None:
                missing.append(f"{kind} {fid}: pose z is null")
                continue
            out.append((f"{kind} {fid}", rect, f.pose[2]))
    for rid, rp in spec.return_paths.items():
        rect = return_rect(spec, rid)
        if rect is None or not in_reach(spec, sid, rect):
            continue
        out.append((f"return path {rid}", rect, rp.pose[2]))
    for hid in spec.gang_heads:
        hb = head_box(spec, hid, cad)
        if hb.rect is None:
            st = hb.station and spec.stations.get(hb.station)
            near = st and station_rect(spec, hb.station)
            if near is not None and in_reach(spec, sid, near) is not False:
                missing += hb.unresolved
            continue
        if in_reach(spec, sid, hb.rect):
            where = "retracted on its slide" if hb.slide_from else "over its station"
            out.append((f"gang head {hid} ({where})", hb.rect, hb.z1))
    return out, missing


def scara_steps(spec: LineSpec, sid: str) -> list:
    """The active steps a SCARA performs at its station, in sequence order."""
    st = spec.scaras[sid].station
    return [s for s in spec.sequence if s.status == "active" and s.station == st
            and robot_served(s)]


def tool_switches(spec: LineSpec, sid: str) -> list[str]:
    """Ids of the steps that start with a dual-tool switch: the tool differs from the previous
    SCARA step's, read cyclically (the next drone's first step follows this drone's last)."""
    steps = [s for s in scara_steps(spec, sid) if s.tool]
    out = []
    for i, s in enumerate(steps):
        prev = steps[i - 1]
        if len(steps) > 1 and prev.tool != s.tool:
            out.append(s.id)
    return out


def park_steps(spec: LineSpec, sid: str) -> tuple[set[str], set[str]]:
    """(steps that end at park_pose, steps that start from it), D-014: while a gang head at the
    SCARA's station is in its working position the SCARA is parked, so the SCARA step right
    before each active drive step there ends at park, and the next SCARA step (read cyclically:
    the next drone's first) starts from it."""
    st = spec.scaras[sid].station
    seq = [s for s in spec.sequence if s.status == "active" and s.station == st]
    mine = [s.id for s in seq if robot_served(s)]
    ends, starts = set(), set()
    if not mine:
        return ends, starts
    for i, s in enumerate(seq):
        if s.capability != "drive" or s.head is None:
            continue
        before = [x.id for x in seq[:i] if x.id in mine]
        after = [x.id for x in seq[i + 1:] if x.id in mine]
        ends.add(before[-1] if before else mine[-1])
        starts.add(after[0] if after else mine[0])
    return ends, starts


def park_joints(spec: LineSpec, sid: str) -> tuple[float, float] | None:
    pp = spec.scaras[sid].park_pose
    return None if pp is None or pp.j1 is None or pp.j2 is None else (pp.j1, pp.j2)


def step_legs(spec: LineSpec, sid: str, step, k: Kin):
    """[(label, a, b, carrying, ja, jb)] of one pick-and-place, or a `remove` (lift off the
    pallet, put down on the return path): start -> pick, pick -> place carrying, place -> end.
    Start and end are home, or park_pose where `park_steps` says (ja / jb: its joints)."""
    pick, place = step_points(spec, step)
    hm = home(k, spec.stations[spec.scaras[sid].station].pose[:2])
    if pick is None or place is None or hm is None:
        return None
    ends, starts = park_steps(spec, sid)
    pj = park_joints(spec, sid) if step.id in ends | starts else None
    if step.id in ends | starts and pj is None:
        return None
    park = fk(k, *pj) if pj is not None else None
    s_pt, s_j, s_lab = (park, pj, "park") if step.id in starts else (hm, None, "home")
    e_pt, e_j, e_lab = (park, pj, "park") if step.id in ends else (hm, None, "home")
    return [(f"{s_lab}_to_pick", s_pt, pick[:2], False, s_j, None),
            ("pick_to_place", pick[:2], place[:2], True, None, None),
            (f"place_to_{e_lab}", place[:2], e_pt, False, None, e_j)]


def step_points(spec: LineSpec, step):
    """(pick, place) poses of a SCARA step: from its `from` source to its station's pallet; a
    `remove` lifts from the pallet and puts down at its `to` return path (D-013)."""
    st = spec.stations[step.station].pose
    if step.capability == "remove":
        rp = spec.return_paths.get(step.to)
        dest = rp.pose if rp is not None else None
        return (st, dest) if dest is not None and None not in dest[:3] and None not in st[:3] \
            else (None, None)
    src = source_pose(spec, step.from_)
    if src is None or None in src[:3] or None in st[:3]:
        return None, None
    return src, st


def step_safe_z(spec: LineSpec, sid: str, step, cad: CadHeights, switch: bool = False) -> SafeZ:
    """How high one SCARA step must travel: over everything under the band its tool (and the part
    it carries) sweeps along each leg's joint-space path, plus — when the step starts with a
    dual-tool switch — under the end-effector's disc at home. Per leg: tallest item under the band
    + the carried part's height on the carrying leg + z_margin."""
    out = SafeZ(sid, switch=switch)
    k = kin(spec, sid)
    if k is None:
        out.unresolved.append(f"{sid}: base pose is null")
        return out
    if k.links is None:
        out.unresolved.append(f"robot_models.{spec.scaras[sid].model}.link1/link2 null: the "
                              f"swept path of {sid} is not known")
        return out
    margin = spec.scara_defaults.z_margin
    if margin is None:
        out.unresolved.append("scara_defaults.z_margin is null")
    ee = tool_radius(spec, step.tool) if step.tool else end_effector_radius(spec, sid)
    if ee is None:
        out.unresolved.append(f"{step.id}: tool {step.tool} dims/offset_r null (its swept band)")
    hang = None
    if step.part:
        ext = part_extent(spec, step.part, cad)
        pr = part_radius(spec, step.part, cad)
        if ext is None:
            out.unresolved.append(f"{step.id}: {unknown_extent(spec, step.part, cad)}")
        if pr is None:
            out.unresolved.append(f"{step.id}: plan size of {step.part} unknown")
        if ext is not None:
            hang = (step.part, ext[1])
        if pr is not None and ee is not None:
            t = spec.tools.get(step.tool)
            pr_c = pr + ((t.offset_r or 0.0) if t else 0.0)
    legs = step_legs(spec, sid, step, k)
    if legs is None:
        out.unresolved.append(f"{step.id}: pick, place, home or park pose not known"
                              + (f" (scaras.{sid}.park_pose is null)"
                                 if step.id in set.union(*park_steps(spec, sid))
                                 and park_joints(spec, sid) is None else ""))
    items, missing = _plan_items(spec, sid, step, cad)
    out.unresolved += missing
    if out.unresolved:
        return out
    need: list[tuple[float, str, float, tuple[str, float]]] = []   # (z, item, top, hang)
    under: dict[str, float] = {}
    for label, a, b, carrying, ja, jb in legs:
        path = leg_path(k, a, b, ja, jb)
        if path is None:
            out.unresolved.append(f"{step.id}: {label} has no joint path inside the limits "
                                  "(see scara_reach)")
            continue
        band = max(ee, pr_c) if carrying and step.part else ee
        h = hang if carrying and hang else ("nothing", 0.0)
        for name, rect, top in items:
            if any(rect.dist_from(x, y) <= band + EPS for x, y in path):
                under[name] = top
                need.append((top + h[1] + margin, name, top, h))
    if switch:
        hm = legs[0][1]
        disc = end_effector_radius(spec, sid)
        if disc is None:
            out.unresolved.append(f"{sid}: end-effector radius unknown (tool dims/offset_r)")
        else:
            for name, rect, top in items:
                if rect.dist_from(*hm) <= disc + EPS:
                    under[name] = top
                    need.append((top + margin, f"{name} (tool switch at home)", top,
                                 ("nothing", 0.0)))
    if out.unresolved:
        return out
    out.items = sorted(under.items(), key=lambda it: -it[1])
    if not need:
        out.value, out.tallest, out.hang = margin, ("nothing", 0.0), ("nothing", 0.0)
        return out
    z, name, top, h = max(need, key=lambda n: n[0])
    out.value, out.tallest, out.hang = z, (name, top), h
    return out


def safe_z(spec: LineSpec, sid: str, cad: CadHeights) -> SafeZ:
    """The SCARA's travel height: the highest of its steps' path-based safe_z (D-013). Each
    step's own value is in `steps` (estimate_moves.py times every step at its own)."""
    out = SafeZ(sid)
    switches = set(tool_switches(spec, sid))
    best = None
    for s in scara_steps(spec, sid):
        one = step_safe_z(spec, sid, s, cad, switch=s.id in switches)
        out.steps[s.id] = one.doc() | {"tool_switch": one.switch}
        out.unresolved += one.unresolved
        for name, top in one.items:
            if name not in dict(out.items):
                out.items.append((name, top))
        if one.value is not None and (best is None or one.value > best.value):
            best = one
    if not out.steps:
        out.unresolved.append(f"{sid}: no step to travel for")
    if out.unresolved or best is None:
        return out
    out.value, out.tallest, out.hang = best.value, best.tallest, best.hang
    return out


# ---------------------------------------------------------------- payload and wrist inertia

def plate_inertia(m: float, x: float, y: float) -> float:
    """A uniform plate's moment about the vertical axis through its centre."""
    return m * (x * x + y * y) / 12.0


def wrist_load(spec: LineSpec, sid: str, cad: CadHeights) -> dict:
    """Payload and J4 inertia of a SCARA's end-effector with its heaviest carried part.

    Payload = every tool's mass + the heaviest part a step lifts (with what rides in it).
    Inertia about J4 = sum of (I_cm + m r^2): each tool a plate of its dims[x, y] at offset_r; the
    carried part (and what rides in it) a plate of its plan size under the tool that lifts it, at
    that tool's offset_r. The worst step is reported. Nulls are listed, not guessed."""
    sc = spec.scaras[sid]
    missing, tools_kg, tools_i = [], 0.0, 0.0
    for tid in sc.tools:
        t = spec.tools[tid]
        if t.mass is None:
            missing.append(f"tools.{tid}.mass")
        if t.offset_r is None:
            missing.append(f"tools.{tid}.offset_r")
        if None in t.dims[:2]:
            missing.append(f"tools.{tid}.dims")
        if not missing:
            tools_kg += t.mass
            tools_i += plate_inertia(t.mass, t.dims[0], t.dims[1]) + t.mass * t.offset_r ** 2
    worst_kg, worst_i, worst_part = 0.0, 0.0, None
    for s in scara_steps(spec, sid):
        if not s.part:
            continue
        r = (spec.tools[s.tool].offset_r if s.tool in spec.tools else 0.0) or 0.0
        kg, inertia = 0.0, 0.0
        for q in carried_parts(spec, s.part):
            m, size = spec.parts[q].mass, part_plan(spec, q, cad)
            if m is None:
                missing.append(f"parts.{q}.mass")
                continue
            if size is None:
                missing.append(f"plan size of {q}")
                continue
            kg += m
            inertia += plate_inertia(m, *size) + m * r * r
        if kg > worst_kg:
            worst_kg, worst_part = kg, s.part
        worst_i = max(worst_i, inertia)
    m = scara_model(spec, sid)
    return {"tools_kg": round(tools_kg, 4), "part": worst_part,
            "part_kg": round(worst_kg, 4), "payload_kg": round(tools_kg + worst_kg, 4),
            "payload_rated": m.payload_rated, "tools_inertia": round(tools_i, 6),
            "part_inertia": round(worst_i, 6), "inertia": round(tools_i + worst_i, 6),
            "inertia_rated": m.inertia_rated, "missing": sorted(set(missing))}


# ---------------------------------------------------------------- clearances (D-014)

def _seg_point(ax, ay, bx, by, px, py) -> float:
    dx, dy = bx - ax, by - ay
    n = dx * dx + dy * dy
    t = 0.0 if n == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / n))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def _segs_cross(a, b, c, d) -> bool:
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    d1, d2, d3, d4 = side(c, d, a), side(c, d, b), side(a, b, c), side(a, b, d)
    return d1 * d2 < 0 and d3 * d4 < 0


def seg_rect_dist(a, b, rect: Rect) -> float:
    """Plan distance from segment a-b to a rectangle (0 when they touch or overlap)."""
    if rect.dist_from(*a) == 0.0 or rect.dist_from(*b) == 0.0:
        return 0.0
    corners = [(rect.x0, rect.y0), (rect.x1, rect.y0), (rect.x1, rect.y1), (rect.x0, rect.y1)]
    edges = list(zip(corners, corners[1:] + corners[:1]))
    if any(_segs_cross(a, b, c, d) for c, d in edges):
        return 0.0
    return min([rect.dist_from(*a), rect.dist_from(*b)]
               + [_seg_point(a[0], a[1], b[0], b[1], *c) for c in corners])


def capsule_box_clearance(a, b, radius: float, z0: float, z1: float, rect: Rect,
                          c0: float, c1: float) -> float:
    """3D clearance between a vertical capsule prism (plan segment a-b swept by `radius`, over
    z0..z1) and an axis-aligned box (rect, c0..c1); negative = overlap (the plan depth, or the
    vertical depth when that is smaller)."""
    plan = seg_rect_dist(a, b, rect) - radius
    vert = max(c0 - z1, z0 - c1)                     # > 0: apart in z
    if vert > 0.0 and plan > 0.0:
        return math.hypot(plan, vert)
    if vert > 0.0:
        return vert
    if plan > 0.0:
        return plan
    return max(plan, vert)                           # both overlapping: the shallower way out


@dataclass
class Capsule:
    name: str
    a: tuple[float, float]
    b: tuple[float, float]
    radius: float
    z0: float
    z1: float


def arm_capsules(spec: LineSpec, sid: str, j1: float, j2: float, cad: CadHeights,
                 tcp: tuple[float, float] | None = None) -> tuple[list[Capsule], list[str]]:
    """The arm at (j1, j2) as capsules in the line frame (D-014): link1 J1 -> J2, link2 J2 -> TCP
    and the shaft, sized from the vendor STEP (reports/vendor_<model>.json, z relative to the TCP
    at the top of its stroke), plus the end-effector: a disc of `end_effector_radius` from the TCP
    down by its longest tool reach (dims[2]).

    `tcp` (D-015) is the TCP's z range (lo, hi) while it is in this plan pose — a vertical move at
    a pick or place sweeps lo..hi; default the top of the stroke. The links do not move in z (a
    SCARA's Z axis is the shaft), so only the shaft and the tools follow the TCP."""
    k = kin(spec, sid)
    model = spec.scaras[sid].model
    if k is None:
        return [], [f"{sid}: base pose is null"]
    if k.links is None:
        return [], [f"robot_models.{model}.link1/link2 null"]
    rep = cad.vendor(model)
    missing = []
    if rep is None:
        missing.append(f"{sid}: no arm capsules for {model} (run tools/cad/vendor.py <STEP> "
                       f"--model {model} -> vendor_{model}.json)")
    ee = end_effector_radius(spec, sid)
    reach = [spec.tools[t].dims[2] for t in spec.scaras[sid].tools]
    if ee is None or None in reach:
        missing.append(f"{sid}: tool dims/offset_r null (the end-effector's envelope)")
    if missing:
        return [], missing
    l1, _ = k.links
    a = k.yaw + j1
    elbow = (k.base[0] + l1 * math.cos(a), k.base[1] + l1 * math.sin(a))
    xy = fk(k, j1, j2)
    j1xy = (k.base[0], k.base[1])
    top = k.z_hi
    lo, hi = tcp if tcp is not None else (top, top)
    caps = rep["capsules"]
    out = [Capsule("link1", j1xy, elbow, caps["link1"]["radius_m"],
                   top + caps["link1"]["z_m"][0], top + caps["link1"]["z_m"][1]),
           Capsule("link2", elbow, xy, caps["link2"]["radius_m"],
                   top + caps["link2"]["z_m"][0], top + caps["link2"]["z_m"][1]),
           Capsule("shaft", xy, xy, caps["shaft"]["radius_m"],
                   lo + caps["shaft"]["z_m"][0], hi + caps["shaft"]["z_m"][1])]
    if spec.scaras[sid].tools:
        out.append(Capsule("tools", xy, xy, ee, lo - max(reach), hi))
    return out, []


def plan_clearance(c: Capsule, rect: Rect) -> float:
    """Plan (XY) clearance of a capsule to a rectangle, height ignored; negative = overlap."""
    return seg_rect_dist(c.a, c.b, rect) - c.radius


def head_envelopes(spec: LineSpec, sid: str, cad: CadHeights, retracted: bool = False
                   ) -> tuple[dict[str, list], list[str]]:
    """{head: envelope [(name, rect, z0, z1)]} for every gang head driven at the SCARA's station,
    and what is missing (a null slide frame is listed, the head box is still used). The envelope
    is the head at its working position plus its slide frame; `retracted` (D-015): the head where
    it waits on its slide while the SCARA moves, plus its slide frame."""
    st = spec.scaras[sid].station
    out, missing = {}, []
    for hid in spec.gang_heads:
        hb = head_box(spec, hid, cad)
        if hb.station != st:
            continue
        if hb.working is None:
            missing += hb.unresolved
            continue
        missing += hb.frame_unresolved
        if retracted:
            out[hid] = [("head (retracted)", hb.rect, hb.z0, hb.z1)] + list(hb.frame)
        else:
            out[hid] = hb.working_envelope()
    return out, missing


def park_clearance(spec: LineSpec, sid: str, joints: tuple[float, float], cad: CadHeights
                   ) -> dict:
    """The parked arm against every working envelope at its station (D-014), in plan (D-015):
    the smallest XY clearance (negative = the arm is over or under the envelope in plan), between
    which arm capsule and which envelope part, and every pair. Height separation is not
    clearance here; the 3D value is reported next to it (`clearance_3d_m`) as extra margin."""
    caps, missing = arm_capsules(spec, sid, *joints, cad)
    envs, miss2 = head_envelopes(spec, sid, cad)
    missing += miss2
    pairs = []
    for hid, env in envs.items():
        for name, rect, c0, c1 in env:
            for c in caps:
                pairs.append({"arm": c.name, "head": hid, "part": name,
                              "clearance_m": round(plan_clearance(c, rect), 4),
                              "clearance_3d_m": round(capsule_box_clearance(
                                  c.a, c.b, c.radius, c.z0, c.z1, rect, c0, c1), 4)})
    best = min(pairs, key=lambda p: p["clearance_m"]) if pairs else None
    best3 = min(pairs, key=lambda p: p["clearance_3d_m"]) if pairs else None
    return {"clearance_m": best and best["clearance_m"], "nearest": best,
            "clearance_3d_m": best3 and best3["clearance_3d_m"], "pairs": pairs,
            "heads": sorted(envs), "unresolved": missing}


PATH_CLEAR_M = 0.02       # every arm path clears a slide frame or a waiting head by this (D-015)


def _leg_tcp_z(label: str, i: int, n: int, travel: float, pick_z: float, place_z: float,
               top: float) -> tuple[float, float]:
    """The TCP's z range at sample i of n along a leg (D-015). Horizontal moves run at the
    step's safe_z (`travel`). At a pick or a place the TCP also goes down to it, so the end
    sample there spans down to that z. A leg to or from park ends at the top of the stroke: Z is
    interpolated with J1/J2 (a point-to-point move), from safe_z at the pick/place end to the top
    at the park end, and each sample spans the z of its neighbours."""
    def z(t: int) -> float:
        t = max(0, min(n, t))
        if label.endswith("_to_park"):
            return travel + (top - travel) * t / n
        if label.startswith("park_to_"):
            return top + (travel - top) * t / n
        return travel
    zs = [z(i - 1), z(i), z(i + 1)]
    lo, hi = min(zs), max(zs)
    start, end = label.split("_to_")
    if i == 0 and start in ("pick", "place"):
        lo = min(lo, pick_z if start == "pick" else place_z)
    if i == n and end in ("pick", "place"):
        lo = min(lo, pick_z if end == "pick" else place_z)
    return lo, hi


def path_clearance(spec: LineSpec, sid: str, cad: CadHeights) -> dict:
    """D-015 (fixes D-014 M): the arm along every leg of every step (start -> pick, pick ->
    place, place -> end, home or park at the ends; and the dual-tool switch at the step's start)
    against every gang head at its station where it waits retracted on its slide while the SCARA
    moves, and that head's slide frame (rail, post). The arm is its capsules (`arm_capsules`) at
    every joint-path sample, the TCP at the step's safe_z (down to the pick/place at the ends, up
    to the top of the stroke on a park leg). 3D clearance. {"steps": {id: {leg: {clearance_m,
    arm, head, part}}}, "clearance_m", "nearest", "unresolved"}."""
    out = {"scara": sid, "steps": {}, "clearance_m": None, "nearest": None, "heads": [],
           "unresolved": []}
    k = kin(spec, sid)
    if k is None or k.links is None:
        out["unresolved"].append(f"{sid}: base pose or links null")
        return out
    envs, missing = head_envelopes(spec, sid, cad, retracted=True)
    out["heads"] = sorted(envs)
    out["unresolved"] += missing
    if not envs:
        return out
    switches = set(tool_switches(spec, sid))
    worst = None
    for step in scara_steps(spec, sid):
        sz = step_safe_z(spec, sid, step, cad, switch=step.id in switches)
        legs = step_legs(spec, sid, step, k)
        pick, place = step_points(spec, step)
        if sz.unresolved or sz.value is None or legs is None:
            out["unresolved"] += sz.unresolved or [f"{step.id}: legs or safe_z unknown"]
            continue
        rows = out["steps"].setdefault(step.id, {})
        samples = []
        for label, a, b, _, ja, jb in legs:
            js = leg_joints(k, a, b, ja, jb)
            if js is None:
                out["unresolved"].append(f"{step.id}: no joint path for {label}")
                continue
            for i, j in enumerate(js):
                samples.append((label, j, _leg_tcp_z(label, i, len(js) - 1, sz.value, pick[2],
                                                     place[2], k.z_hi)))
        if step.id in switches and legs:
            first = legs[0]
            j0 = first[4] if first[4] is not None else (leg_ends(k, first[1], first[2]) or
                                                         [None])[0]
            if j0 is not None:
                lo, hi = (min(sz.value, k.z_hi), max(sz.value, k.z_hi)) \
                    if first[4] is not None else (sz.value, sz.value)
                samples.append(("tool_switch", j0, (lo, hi)))
        for label, j, tz in samples:
            caps, miss = arm_capsules(spec, sid, j[0], j[1], cad, tcp=tz)
            if miss:
                out["unresolved"] += miss
                break
            for hid, env in envs.items():
                for name, rect, c0, c1 in env:
                    for c in caps:
                        cl = capsule_box_clearance(c.a, c.b, c.radius, c.z0, c.z1, rect, c0, c1)
                        row = rows.get(label)
                        if row is None or cl < row["clearance_m"]:
                            rows[label] = {"clearance_m": round(cl, 4), "arm": c.name,
                                           "head": hid, "part": name}
                        if worst is None or cl < worst[0]:
                            worst = (cl, {"step": step.id, "leg": label, "arm": c.name,
                                          "head": hid, "part": name})
    out["unresolved"] = list(dict.fromkeys(out["unresolved"]))
    if worst is not None:
        out["clearance_m"] = round(worst[0], 4)
        out["nearest"] = worst[1]
    return out


OPERATOR_REFILLED = ("magazine", "nest")   # feeder types an operator refills by hand (D-016)


def refill_envelopes(spec: LineSpec, sid: str
                     ) -> tuple[dict[str, tuple[Rect, float, str]], list[str]]:
    """{id: (plan footprint, pick plane z, kind)} of every refill envelope at the SCARA's
    station: a buffer a source refills (D-015, kind "buffer"), and a magazine or nest an operator
    refills by hand (D-016, kind "magazine"). Either is refilled from above while the SCARA is
    parked, so its refill envelope is the open-topped column over its footprint; it is checked
    in plan, i.e. at every height (the source's own body fills the column below its pick
    plane)."""
    st = spec.scaras[sid].station
    out, missing = {}, []
    filled = {src.buffer for src in spec.sources.values()}
    items = [(bid, b, "buffer", "buffers") for bid, b in spec.buffers.items()
             if b.station == st and bid in filled]
    items += [(fid, f, "magazine", "feeders") for fid, f in spec.feeders.items()
              if f.station == st and f.type in OPERATOR_REFILLED]
    for eid, e, kind, group in items:
        rect = source_rect(spec, eid)
        if rect is None or e.pose is None or e.pose[2] is None:
            missing.append(f"{group}.{eid}: pose null (its refill envelope)")
            continue
        fp = spec.layout.feeder_footprints.get(eid) if spec.layout else None
        if fp is None or None in fp[:2]:
            missing.append(f"layout.feeder_footprints.{eid} null (its refill envelope)")
            continue
        out[eid] = (rect, e.pose[2], kind)
    return out, missing


def refill_clearance(spec: LineSpec, sid: str, cad: CadHeights) -> dict:
    """D-015, D-016: while the SCARA is parked its station's sources may be refilled — a buffer
    by its source, a magazine or nest by an operator's hand — so nothing may stand over a refill
    envelope (the open-topped column over its footprint) then: not the parked arm, not its paths
    into and out of park, and not its station's gang head (working, retracted, or anywhere on
    its slide frame). A leg out of park that picks from that same source ends in it by design: it
    is listed as `excluded` (the refill must be done before the SCARA goes in — an interlock, not
    a geometry). Clearance per envelope, in plan (the column has no top and its source's body
    fills it below the pick plane, so height is no separation); negative = it crosses.
    {"envelopes": {id: {"kind", "parked", "path", "head"}}, "clearance_m", "nearest",
    "head_clearance_m", "excluded", "unresolved"}."""
    out = {"scara": sid, "envelopes": {}, "clearance_m": None, "nearest": None,
           "head_clearance_m": None, "excluded": [], "unresolved": []}
    envs, missing = refill_envelopes(spec, sid)
    out["unresolved"] += missing
    if not envs:
        return out
    for eid, (_, _, kind) in envs.items():
        out["envelopes"][eid] = {"kind": kind}
    heads, miss = head_envelopes(spec, sid, cad)
    back, _ = head_envelopes(spec, sid, cad, retracted=True)
    out["unresolved"] += miss
    hworst = None
    for hid, env in heads.items():
        parts = env + [p for p in back.get(hid, []) if p[0] == "head (retracted)"]
        for name, hrect, _, _ in parts:
            for eid, (rect, _, _) in envs.items():
                cl = hrect.gap(rect)
                row = out["envelopes"][eid]
                if "head" not in row or cl < row["head"]["clearance_m"]:
                    row["head"] = {"clearance_m": round(cl, 4), "head": hid, "part": name}
                if hworst is None or cl < hworst:
                    hworst = cl
    if hworst is not None:
        out["head_clearance_m"] = round(hworst, 4)
    k = kin(spec, sid)
    joints = park_joints(spec, sid)
    if k is None or k.links is None:
        out["unresolved"].append(f"{sid}: base pose or links null")
        return out
    ends, starts = park_steps(spec, sid)
    if not ends and not starts:
        return out
    if joints is None:
        out["unresolved"].append(f"scaras.{sid}.park_pose is null")
        return out
    samples = [("parked", joints, (k.z_hi, k.z_hi))]
    for step in scara_steps(spec, sid):
        if step.id not in ends | starts:
            continue
        sz = step_safe_z(spec, sid, step, cad)
        legs = step_legs(spec, sid, step, k)
        pick, place = step_points(spec, step)
        if sz.unresolved or sz.value is None or legs is None:
            out["unresolved"] += sz.unresolved or [f"{step.id}: legs or safe_z unknown"]
            continue
        for label, a, b, _, ja, jb in legs:
            if "park" not in label:
                continue
            skip = step.from_ in envs and label.startswith("park")
            if skip:
                out["excluded"].append(f"{step.id} {label}: picks from {step.from_}")
            js = leg_joints(k, a, b, ja, jb)
            if js is None:
                out["unresolved"].append(f"{step.id}: no joint path for {label}")
                continue
            samples += [(f"{step.id} {label}", j, _leg_tcp_z(label, i, len(js) - 1, sz.value,
                                                             pick[2], place[2], k.z_hi),
                         step.from_ if skip else None)
                        for i, j in enumerate(js)]
    samples[0] = samples[0] + (None,)
    worst = None
    for label, j, tz, skip in samples:
        caps, miss = arm_capsules(spec, sid, j[0], j[1], cad, tcp=tz)
        if miss:
            out["unresolved"] += miss
            break
        for eid, (rect, _, _) in envs.items():
            if eid == skip:
                continue
            for c in caps:
                cl = plan_clearance(c, rect)
                where = "parked" if label == "parked" else "path"
                row = out["envelopes"][eid]
                if where not in row or cl < row[where]["clearance_m"]:
                    row[where] = {"clearance_m": round(cl, 4), "at": label, "arm": c.name}
                if worst is None or cl < worst[0]:
                    worst = (cl, {"envelope": eid, "at": label, "arm": c.name})
    out["unresolved"] = list(dict.fromkeys(out["unresolved"]))
    if worst is not None:
        out["clearance_m"] = round(worst[0], 4)
        out["nearest"] = worst[1]
    return out


def with_park(spec: LineSpec, sid: str, joints: tuple[float, float]) -> LineSpec:
    """A copy of `spec` with `sid` parked at `joints` (what choose_park tries)."""
    sc = spec.scaras[sid].model_copy(update={"park_pose": ParkPose(j1=joints[0], j2=joints[1])})
    return spec.model_copy(update={"scaras": {**spec.scaras, sid: sc}})


PARK_CLEAR_M = 0.03       # an auto-chosen park pose clears the working envelope by this much
PARK_STEP_DEG = 0.5
PARK_FOLD_STEP_DEG = 5.0  # D-015: j2 eased off its limit in these steps when no folded pose clears


def _parked_refill(spec: LineSpec, sid: str, joints, cad: CadHeights) -> dict[str, float]:
    """{kind: plan clearance} of the parked arm alone to the station's refill envelopes of that
    kind (cheap: no paths), for choose_park's first cut."""
    envs, _ = refill_envelopes(spec, sid)
    caps, miss = arm_capsules(spec, sid, *joints, cad)
    if not envs or miss:
        return {}
    out: dict[str, float] = {}
    for rect, _, kind in envs.values():
        cl = min(plan_clearance(c, rect) for c in caps)
        out[kind] = min(out.get(kind, cl), cl)
    return out


def _refill_by_kind(rc: dict) -> dict[str, float]:
    """{kind: smallest arm clearance (parked or path)} from a refill_clearance result."""
    out: dict[str, float] = {}
    for row in rc["envelopes"].values():
        for where in ("parked", "path"):
            if where in row:
                cl = row[where]["clearance_m"]
                out[row["kind"]] = min(out.get(row["kind"], cl), cl)
    return out


def choose_park(spec: LineSpec, sid: str, cad: CadHeights, need: float = PARK_CLEAR_M) -> dict:
    """The dev park pose (D-014..D-016): j2 folded on the elbow side of the place pose, j1
    searched over its limits in PARK_STEP_DEG steps for the value closest to the place pose's j1
    that
      - clears its station's working envelope(s) in plan by >= `need` (park_clearance),
      - keeps the parked arm >= `need` clear in plan of every refill envelope at the station
        (refilled buffers and, D-016, operator-refilled magazines), and its legs into and out of
        park clear of them (refill_clearance >= 0),
      - keeps every path >= PATH_CLEAR_M clear of the waiting heads and their slide frames
        (path_clearance; the park legs change with the pose).
    j2 is at its limit first (D-014). Only when no such pose exists there is j2 eased off the
    limit in PARK_FOLD_STEP_DEG steps (D-015), the most folded level that works winning. D-016:
    the fold is never eased for a magazine: the first level where a pose meets every rule but
    the magazine ones is the last level tried, and if none there clears the magazines too it is
    an error naming them.
    {"j1", "j2", "clearance_m", "place_j", "fold_eased_deg", ...} or {"error"}."""
    k = kin(spec, sid)
    st = spec.stations[spec.scaras[sid].station].pose
    if k is None or k.links is None or None in st[:2]:
        return {"error": f"{sid}: base, links or station pose null"}
    sols, why = ik(k, st[0], st[1])
    if not sols:
        return {"error": f"{sid}: place pose not reachable ({why})"}
    n = int(math.floor(math.degrees(k.j1_lim) / PARK_STEP_DEG))
    levels = int(math.floor(math.degrees(k.j2_lim) / PARK_FOLD_STEP_DEG))
    rejected: dict[str, int] = {}
    tried = 0
    for level in range(levels + 1):
        ease = math.radians(level * PARK_FOLD_STEP_DEG)
        cands = []
        for pj1, pj2 in sols:
            j2 = math.copysign(k.j2_lim - ease, pj2 if pj2 != 0 else 1.0)
            for i in range(-n, n + 1):
                j1 = math.radians(i * PARK_STEP_DEG)
                pc = park_clearance(spec, sid, (j1, j2), cad)
                if pc["unresolved"] and pc["clearance_m"] is None:
                    return {"error": "; ".join(pc["unresolved"])}
                if pc["clearance_m"] is None or pc["clearance_m"] < need - EPS:
                    continue
                cands.append(((abs(j1 - pj1), abs(j2 - pj2)), j1, j2, pc, (pj1, pj2)))
        tried += len(cands)
        but_magazine = None          # the nearest pose meeting every rule but the magazine ones
        for key, j1, j2, pc, place in sorted(cands, key=lambda c: c[0]):
            mag_fail = []
            parked = _parked_refill(spec, sid, (j1, j2), cad)
            if parked.get("buffer", need) < need - EPS:
                rejected["refill (parked)"] = rejected.get("refill (parked)", 0) + 1
                continue
            if parked.get("magazine", need) < need - EPS:
                mag_fail.append("parked")
            trial = with_park(spec, sid, (j1, j2))
            rc = refill_clearance(trial, sid, cad)
            kinds = _refill_by_kind(rc)
            if kinds.get("buffer", 0.0) < -EPS:
                rejected["refill (path)"] = rejected.get("refill (path)", 0) + 1
                continue
            if kinds.get("magazine", 0.0) < -EPS:
                mag_fail.append("path")
            if mag_fail and but_magazine is not None:
                rejected["magazine"] = rejected.get("magazine", 0) + 1
                continue
            path = path_clearance(trial, sid, cad)
            if path["clearance_m"] is None and path["unresolved"]:
                rejected["path unresolved"] = rejected.get("path unresolved", 0) + 1
                continue
            if path["clearance_m"] is not None and path["clearance_m"] < PATH_CLEAR_M - EPS:
                rejected["path"] = rejected.get("path", 0) + 1
                continue
            if mag_fail:
                rejected["magazine"] = rejected.get("magazine", 0) + 1
                but_magazine = (j1, j2, rc, mag_fail)
                continue
            return {"j1": round(j1, 4), "j2": round(j2, 6), "j1_deg": round(math.degrees(j1), 1),
                    "j2_deg": round(math.degrees(j2), 1), "clearance_m": pc["clearance_m"],
                    "clearance_3d_m": pc["clearance_3d_m"], "nearest": pc["nearest"],
                    "refill_clearance_m": rc["clearance_m"],
                    "path_clearance_m": path["clearance_m"],
                    "place_j": [round(place[0], 4), round(place[1], 4)],
                    "fold_eased_deg": level * PARK_FOLD_STEP_DEG,
                    "rejected_nearer": rejected, "unresolved": pc["unresolved"]}
        if but_magazine is not None:
            j1, j2, rc, why = but_magazine
            mags = sorted(e for e, r in rc["envelopes"].items() if r["kind"] == "magazine")
            return {"error": f"{sid}: no park pose at j2 {math.degrees(abs(j2)):.0f}° (fold eased "
                             f"{level * PARK_FOLD_STEP_DEG:g}°, the most folded level that meets "
                             f"every other rule) keeps the parked arm >= {need:g} m and its park "
                             f"legs clear of the refill column of {', '.join(mags)}; easing j2 "
                             f"for a magazine is not allowed (D-016). Nearest: j1 "
                             f"{math.degrees(j1):.1f}°, fails {'/'.join(why)} (refill "
                             f"{rc['clearance_m']} m at {rc['nearest']}); rejected: {rejected}",
                    "magazine_blocked": True}
    return {"error": f"{sid}: no park pose clears its station's gang head(s) by {need:g} m in "
                     f"plan with its refill envelopes and paths clear ({tried} plan-clear "
                     f"pose(s) tried over every fold; rejected: {rejected})"}


def slide_clearance(spec: LineSpec, hid: str, cad: CadHeights) -> dict:
    """The swept path of every part a SCARA carries at the head's station against the head
    waiting retracted on its slide (D-014): per carrying leg, the part's band (its radius +
    the tool's offset_r) at every path sample, over z from its bottom at the step's safe_z (or
    down to the pick/place z at the two ends) to the safe_z, against the head box. Smallest
    clearance per step."""
    hb = head_box(spec, hid, cad)
    out = {"head": hid, "station": hb.station, "steps": {}, "clearance_m": None,
           "unresolved": list(hb.unresolved)}
    if hb.rect is None or hb.station is None:
        return out
    sid = scara_id(hb.station)
    if sid not in spec.scaras:
        return out
    k = kin(spec, sid)
    for step in scara_steps(spec, sid):
        if not step.part:
            continue
        sz = step_safe_z(spec, sid, step, cad)
        ext, pr = part_extent(spec, step.part, cad), part_radius(spec, step.part, cad)
        legs = step_legs(spec, sid, step, k) if k is not None else None
        if sz.unresolved or ext is None or pr is None or legs is None:
            out["unresolved"] += sz.unresolved or [f"{step.id}: part size, legs or safe_z unknown"]
            continue
        t = spec.tools.get(step.tool)
        band = pr + ((t.offset_r or 0.0) if t else 0.0)
        pick, place = step_points(spec, step)
        label, a, b, _, ja, jb = legs[1]
        path = leg_path(k, a, b, ja, jb)
        if path is None:
            out["unresolved"].append(f"{step.id}: no joint path for {label}")
            continue
        travel = sz.value
        worst = None
        for i, (x, y) in enumerate(path):
            lo = travel - ext[1]
            if i == 0:
                lo = min(lo, pick[2])
            elif i == len(path) - 1:
                lo = min(lo, place[2])
            c = capsule_box_clearance((x, y), (x, y), band, lo, travel, hb.rect, hb.z0, hb.z1)
            worst = c if worst is None else min(worst, c)
        out["steps"][step.id] = {"part": step.part, "band_m": round(band, 4),
                                 "safe_z": round(travel, 4), "clearance_m": round(worst, 4)}
    vals = [d["clearance_m"] for d in out["steps"].values()]
    out["clearance_m"] = min(vals) if vals else None
    return out


def step_payload(spec: LineSpec, sid: str, step) -> float | None:
    """What the arm carries on a step: every tool on the end-effector + the part and what rides
    in it (D-014: compared with the robot model's standard_cycle payload)."""
    ms = [spec.tools[t].mass for t in spec.scaras[sid].tools]
    if None in ms:
        return None
    part = part_mass(spec, step.part) if step.part else 0.0
    return None if part is None else sum(ms) + part
