"""Where the robots stand, what they reach, and how high they must travel (D-012, D-013). Plain
floats, no ROS, no CAD.

Shared by check_line.py (`scara_reach`, `scara_clearance`, `safe_z`, `payload`, `wrist_inertia`),
estimate_moves.py and
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

from spec import (REPO, LineSpec, robot_served, scara_base, scara_id, scara_model,  # noqa: F401
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
    return out


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


def joint_path(k: Kin, a: tuple[float, float], b: tuple[float, float]
               ) -> list[tuple[float, float]] | None:
    """The plan path of a point-to-point move a -> b: j1 and j2 interpolated together (how a
    SCARA moves between two poses), on the elbow solution valid at both ends with the least joint
    travel. None when no elbow solution is valid at both ends."""
    sa, _ = ik(k, *a)
    sb, _ = ik(k, *b)
    pairs = [(p, q) for p in sa for q in sb if (p[1] >= 0) == (q[1] >= 0)]
    if not pairs:
        return None
    p, q = min(pairs, key=lambda pq: abs(pq[0][0] - pq[1][0]) + abs(pq[0][1] - pq[1][1]))
    return [fk(k, p[0] + (q[0] - p[0]) * t / PATH_SAMPLES, p[1] + (q[1] - p[1]) * t / PATH_SAMPLES)
            for t in range(PATH_SAMPLES + 1)]


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


def step_legs(spec: LineSpec, sid: str, step, k: Kin):
    """[(label, a, b, carrying)] of one pick-and-place from home, or a `remove` (lift off the
    pallet, put down on the return path): home -> pick, pick -> place carrying, place -> home."""
    pick, place = step_points(spec, step)
    hm = home(k, spec.stations[spec.scaras[sid].station].pose[:2])
    if pick is None or place is None or hm is None:
        return None
    return [("home_to_pick", hm, pick[:2], False), ("pick_to_place", pick[:2], place[:2], True),
            ("place_to_home", place[:2], hm, False)]


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
        out.unresolved.append(f"{step.id}: pick, place or home pose not known")
    items, missing = _plan_items(spec, sid, step, cad)
    out.unresolved += missing
    if out.unresolved:
        return out
    need: list[tuple[float, str, float, tuple[str, float]]] = []   # (z, item, top, hang)
    under: dict[str, float] = {}
    for label, a, b, carrying in legs:
        path = joint_path(k, a, b)
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
