"""Where the gantries stand and how high they must travel (D-009). Plain floats, no ROS, no CAD.

Shared by check_line.py (`gantry_straddle`, `safe_z`) and estimate_moves.py, so the check and the
estimate cannot disagree about a footprint or a height. tools/cad/layout.py draws the same posts
(its own Dim arithmetic, for provenance); tools/tests/test_layout.py pins the two together.

Line frame throughout: origin at the pallet datum of station 1, z = 0 on the datum plane.

Heights. A part's top in the stack is `z_in_stack + height`, height from its `envelope[2]` or
`dims` (box z, cylinder height). A build123d part (pallet, carriers) has no height in the spec:
its generator is the authority on that geometry (D-003), so it is read from the generator's report,
reports/cad_<part>.json (`.dev.json` under --dev), bbox z in the part's own frame, whose z = 0 is
the rest plane. A missing report makes that height UNRESOLVED, never a guess.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from spec import REPO, LineSpec, gantry_served, zone_owner

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

    def doc(self) -> list[float]:
        return [round(v, 6) for v in (self.x0, self.x1, self.y0, self.y1)]


def envelope(spec: LineSpec, gid: str) -> list[tuple[float, float]] | None:
    """[(x0, x1), (y0, y1), (z0, z1)] of the TCP travel in the line frame, or None if a null."""
    g = spec.gantries[gid]
    out = []
    for i, a in enumerate("xyz"):
        ax = getattr(g.axes, a)
        if g.mount[i] is None or ax.min is None or ax.max is None:
            return None
        out.append((g.mount[i] + ax.min, g.mount[i] + ax.max))
    return out


def conveyor_rect(spec: LineSpec) -> Rect | None:
    """The belt in plan: along every station pose, pitch/2 beyond the end stations (layout.py)."""
    if spec.layout is None or spec.layout.conveyor.width is None or spec.conveyor.pitch is None:
        return None
    poses = [s.pose for s in spec.stations.values()]
    if any(p[0] is None or p[1] is None for p in poses):
        return None
    w, half = spec.layout.conveyor.width / 2.0, spec.conveyor.pitch / 2.0
    return Rect(min(p[0] for p in poses) - half, max(p[0] for p in poses) + half,
                min(p[1] for p in poses) - w, max(p[1] for p in poses) + w)


def post_rects(spec: LineSpec, gid: str) -> list[Rect] | None:
    """Four posts, each post_size square, just outside the corners of the travel envelope."""
    env = envelope(spec, gid)
    p = spec.layout.gantry_frame.post_size if spec.layout else None
    if env is None or p is None:
        return None
    (x0, x1), (y0, y1), _ = env
    return [Rect(x0 - p, x0, y0 - p, y0), Rect(x1, x1 + p, y0 - p, y0),
            Rect(x1, x1 + p, y1, y1 + p), Rect(x0 - p, x0, y1, y1 + p)]


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


def stack_parts(spec: LineSpec, station: str) -> list[str]:
    """What stands on the pallet at `station`: the pallet, and every part an active step places
    at this station or an earlier one (by station index). Nothing is ever taken off: conservative."""
    idx = spec.stations[station].index
    out = ["pallet"] if "pallet" in spec.parts else []
    for s in spec.sequence:
        if s.status == "active" and s.part and s.station in spec.stations \
                and spec.stations[s.station].index <= idx and s.part not in out:
            out.append(s.part)
    return out


@dataclass
class SafeZ:
    gantry: str
    value: float | None = None          # line frame z the gantry's XY moves travel at
    tallest: tuple[str, float] | None = None
    hang: tuple[str, float] | None = None
    items: list[tuple[str, float]] = field(default_factory=list)   # (label, top z) under it
    unresolved: list[str] = field(default_factory=list)

    def doc(self) -> dict:
        return {"safe_z": None if self.value is None else round(self.value, 4),
                "tallest_item": self.tallest and [self.tallest[0], round(self.tallest[1], 4)],
                "hang": self.hang and [self.hang[0], round(self.hang[1], 4)],
                "items": [[n, round(z, 4)] for n, z in self.items],
                "unresolved": self.unresolved}


def _inside(env, x, y) -> bool:
    return env[0][0] - EPS <= x <= env[0][1] + EPS and env[1][0] - EPS <= y <= env[1][1] + EPS


def safe_z(spec: LineSpec, gid: str, cad: CadHeights) -> SafeZ:
    """Tallest item under the gantry + tallest part it carries + gantry_defaults.z_margin.

    Items under it: every station, feeder and buffer whose pose lies inside its XY travel —
    a station counts at its pose z plus the tallest part stack standing there; a feeder or buffer
    at its pose z, which is its pick plane (the top of what it presents, D-004).
    """
    out = SafeZ(gid)
    env = envelope(spec, gid)
    if env is None:
        out.unresolved.append(f"{gid}: travel envelope has a null (mount or axis limits)")
        return out
    for sid, st in spec.stations.items():
        x, y, z = st.pose[:3]
        if x is None or y is None or not _inside(env, x, y):
            continue
        if z is None:
            out.unresolved.append(f"stations.{sid}.pose[2] is null")
            continue
        tops = []
        for pid in stack_parts(spec, sid):
            ext = part_extent(spec, pid, cad)
            if ext is None:
                out.unresolved.append(f"{sid}: {unknown_extent(spec, pid, cad)}")
            else:
                tops.append(ext[0])
        out.items.append((f"station {sid}", z + max(tops, default=0.0)))
    for coll, kind in ((spec.feeders, "feeder"), (spec.buffers, "buffer")):
        for fid, f in coll.items():
            if f.pose is None or f.pose[0] is None or f.pose[1] is None \
                    or not _inside(env, f.pose[0], f.pose[1]):
                continue
            if f.pose[2] is None:
                out.unresolved.append(f"{kind} {fid}: pose z is null")
                continue
            out.items.append((f"{kind} {fid}", f.pose[2]))
    owner = zone_owner(spec)
    hangs = []
    for s in spec.sequence:
        if s.status == "active" and gantry_served(s) and owner.get(s.station) == gid and s.part:
            ext = part_extent(spec, s.part, cad)
            if ext is None:
                out.unresolved.append(f"{s.id}: {unknown_extent(spec, s.part, cad)}")
            else:
                hangs.append((s.part, ext[1]))
    margin = spec.gantry_defaults.z_margin
    if margin is None:
        out.unresolved.append("gantry_defaults.z_margin is null")
    if not out.items:
        out.unresolved.append(f"{gid}: no station, feeder or buffer under its travel")
    if out.unresolved:
        return out
    out.tallest = max(out.items, key=lambda it: it[1])
    out.hang = max(hangs, key=lambda h: h[1]) if hangs else ("nothing", 0.0)
    out.value = out.tallest[1] + out.hang[1] + margin
    return out
