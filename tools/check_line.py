"""Static checks on line_spec.yaml (P2). No ROS, no Gazebo.

    tools/.venv/bin/python tools/check_line.py [--spec PATH] [--report PATH] [--dev]
                                               [--milestone NAME | --stations IDS [--name N]]
                                               [-v]

Prints a table, writes reports/line_check.json (`passed: true|false`), and exits non-zero on
any FAIL. WARN never fails the run: a `null` in the spec is WARN "UNRESOLVED" until the phase
that needs the value.

`--milestone NAME` checks only that milestone (D-012): its stations, their SCARAs and gang heads,
and the feeders and buffers they draw from (spec.milestone_subset), and writes
reports/line_check.<name>.json. `--stations a,b,...` (D-015) does the same for any run of
consecutive stations (spec.station_subset; the first keeps its real upstream gap for the index),
writing reports/line_check.<--name, default batch>.json.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import estimate_moves  # noqa: E402
import robot_geometry as rg  # noqa: E402
from spec import (DEFAULT_SPEC, REPO, LineSpec, Merged, SpecError, add_dev_argument,  # noqa: E402
                  load_merged, milestone_subset, parse_spec, parse_station_list, report_path,
                  robot_served, scara_base, scara_id, scara_model, station_subset)

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"

# Thresholds that are not in the spec; override on the command line.
MIN_BUFFER_CAPACITY = 1
MAX_FASTENER_LENGTHS = 3
DATUM_TOL_M = 1e-4
# An arm tooling hole stays this many screw diameters from every arm screw hole (D-012).
PIN_CLEAR_SCREW_D = 2.0
# Plates that sit on the pallet pins and must share datum positions (summary §7.2).
PLATE_STACK = ("bottom_plate", "mid_plate", "top_plate")
# A carried part's swept path clears a head waiting retracted on its slide by this much (D-014).
SLIDE_CLEAR_M = 0.02


@dataclass
class Result:
    name: str
    findings: list[tuple[str, str]] = field(default_factory=list)  # (level, message)
    info: dict = field(default_factory=dict)
    cfg: dict = field(default_factory=dict)  # thresholds not in the spec

    def add(self, level: str, msg: str) -> None:
        self.findings.append((level, msg))

    @property
    def status(self) -> str:
        levels = {lvl for lvl, _ in self.findings}
        return FAIL if FAIL in levels else WARN if WARN in levels else PASS

    def unresolved(self, msg: str) -> None:
        self.add(WARN, f"UNRESOLVED {msg}")


def _active_steps(spec: LineSpec):
    return [s for s in spec.sequence if s.status == "active"]


def _walk_nulls(node, path=""):
    if node is None:
        yield path
    elif isinstance(node, dict):
        for k, v in node.items():
            yield from _walk_nulls(v, f"{path}.{k}" if path else str(k))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from _walk_nulls(v, f"{path}[{i}]")


# --------------------------------------------------------------------------- checks


def check_schema(spec: LineSpec, r: Result) -> None:
    """Parsing already happened (spec.py); here every reference must resolve."""
    parts, feeders, buffers = set(spec.parts), set(spec.feeders), set(spec.buffers)
    heads = set(spec.gang_heads)
    tools = set(spec.tools)
    stations, steps = set(spec.stations), [s.id for s in spec.sequence]

    def ref(where: str, kind: str, value: str | None, valid: set[str]):
        if value is not None and value not in valid:
            r.add(FAIL, f"{where} -> {kind} '{value}' does not exist")

    for name, p in spec.parts.items():
        ref(f"parts.{name}.carried_by", "part", p.carried_by, parts)
    for name, f in spec.fasteners.items():
        ref(f"fasteners.{name}.feeder", "feeder", f.feeder, feeders)
    for name, f in spec.feeders.items():
        ref(f"feeders.{name}.part", "part/fastener", f.part, parts | set(spec.fasteners))
        ref(f"feeders.{name}.target", "head", f.target, heads)
    for name, s in spec.sources.items():
        ref(f"sources.{name}.output", "part", s.output, parts)
        ref(f"sources.{name}.buffer", "buffer", s.buffer, buffers)
    for coll, items in (("feeders", spec.feeders), ("buffers", spec.buffers),
                        ("return_paths", spec.return_paths)):
        for name, f in items.items():
            ref(f"{coll}.{name}.station", "station", f.station, stations)
    for name, rp in spec.return_paths.items():
        ref(f"return_paths.{name}.carrier", "part", rp.carrier, parts)
        # A milestone subset (pallet_in set) may return to a sub-line it does not include.
        if spec.pallet_in is None or rp.sink in spec.sources or rp.sink in buffers:
            ref(f"return_paths.{name}.sink", "source/buffer", rp.sink,
                set(spec.sources) | buffers)
    for name, h in spec.gang_heads.items():
        # `bottom_plate.arm_holes`: the part must exist; sub-features are not modelled yet.
        if h.pattern_from:
            ref(f"gang_heads.{name}.pattern_from", "part", h.pattern_from.split(".")[0], parts)
    for i, dup in enumerate(steps):
        if dup in steps[:i]:
            r.add(FAIL, f"sequence: duplicate step id '{dup}'")
    for s in spec.sequence:
        w = f"sequence.{s.id}"
        ref(f"{w}.station", "station", s.station, stations)
        ref(f"{w}.part", "part", s.part, parts)
        ref(f"{w}.from", "feeder/buffer", s.from_, feeders | buffers)
        ref(f"{w}.tool", "tool", s.tool, tools)
        ref(f"{w}.to", "return path", s.to, set(spec.return_paths))
        if (s.capability == "remove") != (s.to is not None):
            r.add(FAIL, f"{w}: a `remove` step names the return path it puts the part on (`to`), "
                        "and only it does")
        sid = scara_id(s.station)
        if s.tool in tools and sid in spec.scaras and s.tool not in spec.scaras[sid].tools:
            r.add(FAIL, f"{w}.tool -> '{s.tool}' is not on {sid}, the SCARA at {s.station}")
        ref(f"{w}.head", "head", s.head, heads)
        ref(f"{w}.fastener", "fastener", s.fastener, set(spec.fasteners))
        if (s.capability == "actuate") != (s.device is not None):
            r.add(FAIL, f"{w}: an `actuate` step names the `device` it moves, and only it does")
        for req in s.requires:
            ref(f"{w}.requires", "step", req, set(steps))
    pins = spec.drone.arm_pins if spec.drone is not None else None
    if pins is not None:
        step = next((s for s in spec.sequence if s.id == pins.extend_step), None)
        upstream = step is None and spec.pallet_in is not None       # extended before this cut
        if not upstream and (step is None or step.device != "arm_pins"):
            r.add(FAIL, f"drone.arm_pins.extend_step -> '{pins.extend_step}' is not an actuate "
                        "step of device arm_pins")
    if spec.layout is not None:
        for key in spec.layout.feeder_footprints:
            ref("layout.feeder_footprints", "feeder/buffer", key, feeders | buffers)
    idx = [st.index for st in spec.stations.values()]
    if len(idx) != len(set(idx)):
        r.add(FAIL, "stations: duplicate index")


def _step_actor(spec: LineSpec, s) -> str | None:
    """The one actor type that performs a step, or None when nothing in the spec says."""
    st = spec.stations.get(s.station)
    if st is not None and st.actor == "sub_line":
        return "sub_line"
    if robot_served(s):
        return "scara"
    if s.capability == "drive":
        return "driver" if s.head is not None else None
    if s.capability == "test":
        return "test"
    return None


def _buffer_holds(spec: LineSpec, buf: str, part: str) -> bool:
    """A buffer holds what the sources filling it make, or the fixture that carries it. A buffer
    no source fills is unconstrained (fixtures returning on their own loop)."""
    outs = [src.output for src in spec.sources.values() if src.buffer == buf]
    return not outs or any(o == part or (o in spec.parts and spec.parts[o].carried_by == part)
                           for o in outs)


def _head_mounts(spec: LineSpec) -> dict[str, set[str]]:
    """{gang head: stations of every step that names it}. A gang head is fixed hardware: it is
    mounted at a station only when that is the one station it is ever named at."""
    out: dict[str, set[str]] = {}
    for s in spec.sequence:
        if s.head in spec.gang_heads:
            out.setdefault(s.head, set()).add(s.station)
    return out


def check_actors(spec: LineSpec, r: Result) -> None:
    """Who does each station's work, and where each part and fastener comes from (D-010..D-012).

    Actors: a station has **at most one SCARA plus at most one gang head** (D-012). Every station
    with an active step has exactly one **primary** actor — scara (a step with a tool, or a part
    drawn `from` a feeder/buffer; the station's own SCARA, spec.py makes that one per station),
    driver (a drive step's gang `head`), test (a test step), or the station's stated `actor`
    (operator, sub_line) — plus, at a SCARA station, at most one **gang head that fastens what the
    SCARA placed**: mounted at this station (every step naming it stands here) and driving a step
    that requires a SCARA step at this station. Any other drive step at a SCARA station makes
    "driver" a second primary actor. A driver station has one head. A station with only parked
    steps is skipped. `actor: null` is UNRESOLVED (not decided yet).

    Material: a step's part comes `from` a feeder that holds it, a buffer a source fills with it
    (or with what it carries), or — at a sub_line station — the station's `source`. A step's
    fastener comes from that fastener's feeder. A drive step that names no fastener is UNRESOLVED
    (the screw is not chosen). Every feeder and buffer is drawn from by at least one active step.
    """
    active = _active_steps(spec)
    mounts = _head_mounts(spec)
    for head, where in sorted(mounts.items()):
        if len(where) > 1:
            r.add(FAIL, f"gang head {head} is named at {len(where)} stations "
                        f"({', '.join(sorted(where))}); it is mounted at one")
    info: dict[str, str] = {}
    for sid, st in sorted(spec.stations.items(), key=lambda kv: kv[1].index):
        steps = [s for s in active if s.station == sid]
        if not steps:
            if any(s.station == sid for s in spec.sequence):
                info[sid] = "parked"
            else:
                r.add(FAIL, f"{sid}: no step stands at this station, so nothing works there")
            continue
        scara_here = {s.id for s in steps if _step_actor(spec, s) == "scara"}
        types = {st.actor} if st.actor else set()
        primary_heads: set[str] = set()
        fastening_heads: set[str] = set()
        for s in steps:
            a = _step_actor(spec, s)
            if a == "driver":
                if s.head in spec.gang_heads and mounts.get(s.head) == {sid} and scara_here \
                        and set(s.requires) & scara_here:
                    fastening_heads.add(s.head)
                    continue
                primary_heads.add(s.head)
            if a is not None:
                types.add(a)
            elif s.capability == "drive":
                r.add(FAIL, f"{s.id}: drive step at {sid} has no driver (head: a gang head)")
            elif s.capability in ("load", "place", "pick"):
                r.add(FAIL, f"{s.id}: moves part '{s.part}' but no SCARA (tool or `from`) or "
                            "sub_line station does")
        info[sid] = " + ".join(sorted(types) + sorted(fastening_heads)) or "none"
        if len(types) > 1:
            why = (" (a drive step here that does not fasten what the SCARA placed, with a gang "
                   "head mounted here, is a second actor)" if {"scara", "driver"} <= types
                   else "")
            r.add(FAIL, f"{sid}: {len(types)} primary actors ({', '.join(sorted(types))}); a "
                        f"station has exactly one{why}")
        elif not types:
            if "actor" in st.model_fields_set:
                r.unresolved(f"stations.{sid}.actor: who does "
                             f"{', '.join(s.id for s in steps)} is not decided")
            else:
                r.add(FAIL, f"{sid}: no actor for {', '.join(s.id for s in steps)} (no SCARA, "
                            "driver or test step, and no stated `actor`)")
        if len(fastening_heads) > 1:
            r.add(FAIL, f"{sid}: {len(fastening_heads)} gang heads fasten what the SCARA "
                        f"placed ({', '.join(sorted(fastening_heads))}); a station has at most "
                        "one SCARA plus one gang head")
        if types == {"driver"} and len(primary_heads) > 1:
            r.add(FAIL, f"{sid}: {len(primary_heads)} drivers ({', '.join(sorted(primary_heads))})"
                        "; a station has exactly one primary actor")
        if st.actor == "sub_line" and st.source is None:
            r.add(FAIL, f"{sid}: a sub_line station needs `source` (the buffer or source it "
                        "loads from)")
        if st.source is not None and st.actor != "sub_line":
            r.add(FAIL, f"{sid}: `source` is only for a sub_line station (actor is "
                        f"{st.actor})")
        if st.source is not None and st.source not in spec.buffers \
                and st.source not in spec.sources:
            r.add(FAIL, f"stations.{sid}.source -> buffer/source '{st.source}' does not exist")
    r.info["station_actors"] = info

    drawn: set[str] = set()
    for s in active:
        st = spec.stations.get(s.station)
        if s.from_ is not None:
            drawn.add(s.from_)
        if st is not None and st.actor == "sub_line" and st.source is not None:
            drawn.add(st.source)
            if st.source in spec.sources:
                drawn.add(spec.sources[st.source].buffer)
        if s.fastener in spec.fasteners:
            fd = spec.fasteners[s.fastener].feeder
            drawn.add(fd)
            if fd in spec.feeders and spec.feeders[fd].part != s.fastener:
                r.add(FAIL, f"{s.id}: fastener {s.fastener}'s feeder {fd} feeds "
                            f"'{spec.feeders[fd].part}'")
        elif s.capability == "drive" and s.fastener is None:
            r.unresolved(f"{s.id}: drives {s.qty if s.qty else 'an unknown number of'} "
                         "fastener(s) but names no `fastener` (no feeder supplies them)")
        if s.part is None:
            continue
        if s.capability == "remove":
            if s.part not in rg.stack_parts(spec, s.station, own=True, before_step=s.id):
                r.add(FAIL, f"{s.id}: removes '{s.part}', which is not on the pallet at "
                            f"{s.station}")
            rp = spec.return_paths.get(s.to)
            if rp is not None and rp.carrier != s.part:
                r.add(FAIL, f"{s.id}: puts '{s.part}' on {s.to}, which returns '{rp.carrier}'")
            if rp is not None and rp.station != s.station:
                r.add(FAIL, f"{s.id}: {s.to} belongs to {rp.station}, not {s.station}")
            continue
        origin = s.from_
        if origin is None and st is not None and st.actor == "sub_line":
            origin = st.source
        if origin is None:
            r.add(FAIL, f"{s.id}: part '{s.part}' comes from nowhere (no `from`, and "
                        f"{s.station} is not a sub_line station with a `source`)")
        elif origin in spec.feeders:
            if spec.feeders[origin].part != s.part:
                r.add(FAIL, f"{s.id}: takes '{s.part}' from {origin}, which holds "
                            f"'{spec.feeders[origin].part}'")
        elif origin in spec.buffers:
            if not _buffer_holds(spec, origin, s.part):
                r.add(FAIL, f"{s.id}: takes '{s.part}' from {origin}, which no source fills "
                            "with it")
        elif origin in spec.sources:
            if spec.sources[origin].output != s.part:
                r.add(FAIL, f"{s.id}: takes '{s.part}' from {origin}, which makes "
                            f"'{spec.sources[origin].output}'")
        # A dangling `from` is reported by schema; a dangling station source just above.
    for coll, kind in ((spec.feeders, "feeder"), (spec.buffers, "buffer")):
        for fid in coll:
            if fid not in drawn:
                r.add(FAIL, f"{kind} {fid}: no active step draws from it")
    used = {s.to for s in active if s.capability == "remove"}
    for rid in spec.return_paths:
        if rid not in used:
            r.add(FAIL, f"return path {rid}: no active `remove` step puts a carrier on it")


def check_unresolved(spec: LineSpec, r: Result, raw: dict) -> None:
    paths = list(_walk_nulls(raw))
    for p in paths:
        r.unresolved(p)
    r.info["count"] = len(paths)
    r.info["paths"] = paths


def check_precedence(spec: LineSpec, r: Result) -> None:
    order = {s.id: i for i, s in enumerate(spec.sequence)}
    for s in spec.sequence:
        for req in s.requires:
            if req not in order:
                continue  # reported by schema
            if req == s.id:
                r.add(FAIL, f"{s.id} requires itself")
            elif order[req] > order[s.id]:
                r.add(FAIL, f"{s.id} requires {req}, which appears later in the sequence")
    # Cycle detection (DFS, three colours).
    graph = {s.id: [q for q in s.requires if q in order] for s in spec.sequence}
    colour: dict[str, int] = {}

    def visit(n: str, stack: list[str]) -> None:
        colour[n] = 1
        for m in graph[n]:
            if colour.get(m) == 1:
                cycle = stack[stack.index(m):] + [m] if m in stack else [n, m]
                r.add(FAIL, "precedence cycle: " + " -> ".join(cycle))
            elif m not in colour:
                visit(m, stack + [m])
        colour[n] = 2

    for n in graph:
        if n not in colour:
            visit(n, [n])


def _station_loads(spec: LineSpec):
    """{station: (known_sum_s, n_unknown, n_steps)} over active steps."""
    out = {name: [0.0, 0, 0] for name in spec.stations}
    for s in _active_steps(spec):
        if s.station not in out:
            continue
        out[s.station][2] += 1
        if s.duration_s is None:
            out[s.station][1] += 1
        else:
            out[s.station][0] += s.duration_s
    return out


def check_takt(spec: LineSpec, r: Result) -> None:
    takt = spec.line.takt_s
    for name, (total, unknown, n) in _station_loads(spec).items():
        if n == 0:
            continue
        per_cell = total / spec.stations[name].count
        if per_cell > takt:
            r.add(FAIL, f"{name}: {total:g} s / {spec.stations[name].count} cell(s) = "
                        f"{per_cell:.2f} s > takt {takt:g} s")
        elif unknown:
            r.unresolved(f"{name}: {unknown}/{n} step duration(s); known part "
                         f"{per_cell:.2f} s <= takt {takt:g} s")


def check_line_balance(spec: LineSpec, r: Result) -> None:
    takt = spec.line.takt_s
    util: dict[str, float] = {}
    for name, (total, unknown, n) in _station_loads(spec).items():
        if n == 0:
            continue
        util[name] = 100.0 * total / spec.stations[name].count / takt
        if unknown:
            r.unresolved(f"{name}: utilization {util[name]:.1f}% is partial "
                         f"({unknown}/{n} durations missing)")
    r.info["utilization_pct"] = {k: round(v, 1) for k, v in util.items()}
    if util:
        bottleneck = max(util, key=util.get)
        r.info["bottleneck"] = bottleneck
        r.info["bottleneck_pct"] = round(util[bottleneck], 1)


def check_anchors(spec: LineSpec, r: Result) -> None:
    """D-013: stations stand on an anchor chain and everything a station owns is placed by an
    offset in its frame. FAIL a station without `anchor`, a station anchored to anything but the
    station before it in line order (the first one: `origin`), and any SCARA, gang head, feeder,
    buffer or return path with an absolute pose (no `offset`). A blow feeder rides its gang head
    and has no pose at all. In a milestone subset the first station's `after` ran upstream."""
    order = sorted(spec.stations, key=lambda s: spec.stations[s].index)
    for i, sid in enumerate(order):
        a = spec.stations[sid].anchor
        if a is None:
            r.add(FAIL, f"stations.{sid}: absolute pose; place it by `anchor: {{after, gap}}`")
            continue
        want = order[i - 1] if i else None
        if i == 0 and a.after != "origin" and a.after in spec.stations:
            r.add(FAIL, f"stations.{sid}: the first station is anchored to the line origin "
                        f"(`after: origin`), not '{a.after}'")
        elif i and a.after not in spec.stations and a.after != "origin" \
                and spec.stations[sid].pose[0] is None:
            r.add(FAIL, f"stations.{sid}.anchor.after: '{a.after}' is not a station")
        elif i and a.after != want:
            r.add(FAIL, f"stations.{sid}.anchor.after is '{a.after}', but the station before it "
                        f"in line order is '{want}'")
    owned = [("feeders", spec.feeders), ("buffers", spec.buffers), ("scaras", spec.scaras),
             ("gang_heads", spec.gang_heads),
             ("return_paths", spec.return_paths)]
    n = 0
    for coll, items in owned:
        for eid, ent in items.items():
            if coll == "feeders" and ent.type == "blow" and ent.pose is None \
                    and ent.offset is None:
                continue
            n += 1
            if ent.offset is None:
                r.add(FAIL, f"{coll}.{eid}: absolute pose; place it by `offset` in its "
                            "station's anchor frame")
            elif coll in ("feeders", "buffers", "return_paths") and ent.station is None:
                r.add(FAIL, f"{coll}.{eid}: an offset needs `station` (whose frame it is in)")
    r.info["stations"] = len(order)
    r.info["owned_entities"] = n


def check_scara_reach(spec: LineSpec, r: Result) -> None:
    """Every pick and place a SCARA works at lies inside its robot model (D-013): an inverse
    kinematics solution with j1 and j2 inside their limits (not a full circle — j1 is measured
    from the direction the base `yaw` points), and z inside base z .. base z + z_stroke. With the
    link split null only the radius is checked (<= reach) and the joint check is UNRESOLVED."""
    seen: set[tuple[str, str]] = set()
    for s in _active_steps(spec):
        if not robot_served(s) or s.station not in spec.stations:
            continue
        sid = scara_id(s.station)
        if sid not in spec.scaras:
            continue                            # reported by schema
        k = rg.kin(spec, sid)
        pick, place = rg.step_points(spec, s)
        if s.capability == "remove":
            targets = [(f"lift {s.station}", spec.stations[s.station].pose),
                       (f"put down {s.to}", place)]
        else:
            targets = [(f"pick {s.from_}", rg.source_pose(spec, s.from_)),
                       (f"place {s.station}", spec.stations[s.station].pose)]
        for label, pose in targets:
            if (sid, label) in seen:
                continue
            seen.add((sid, label))
            if k is None or pose is None or None in pose[:3]:
                r.unresolved(f"{sid}: {label} not evaluable (base or pose null)")
                continue
            d = math.hypot(pose[0] - k.base[0], pose[1] - k.base[1])
            row = r.info.setdefault("targets", {}).setdefault(sid, {})
            row[label] = {"radius_m": round(d, 4), "z": pose[2]}
            if not k.z_lo - rg.EPS <= pose[2] <= k.z_hi + rg.EPS:
                r.add(FAIL, f"{sid}: {label} at z {pose[2]:g} is outside its z range "
                            f"{k.z_lo:g}..{k.z_hi:g} (base z + z_stroke)")
            if k.links is None:
                if d > k.reach + rg.EPS:
                    r.add(FAIL, f"{sid}: {label} is {d:.3f} m from the base, beyond reach "
                                f"{k.reach:g}")
                else:
                    r.unresolved(f"{sid}: {label} joint limits not evaluable (robot model "
                                 "link1/link2 null)")
                continue
            sols, why = rg.ik(k, pose[0], pose[1])
            if not sols:
                r.add(FAIL, f"{sid}: {label} at ({pose[0]:g}, {pose[1]:g}) is outside its "
                            f"reach: {why}")
            else:
                row[label]["j1_j2_deg"] = [[round(math.degrees(a), 1) for a in sol]
                                           for sol in sols]


def _circle_rect_gap(cx: float, cy: float, radius: float, rect: rg.Rect) -> float:
    """Clear distance from a circle to a rectangle; negative when they overlap."""
    return rect.dist_from(cx, cy) - radius if rect.dist_from(cx, cy) > 0 else \
        -(radius + min(cx - rect.x0, rect.x1 - cx, cy - rect.y0, rect.y1 - cy))


def _lens(a, ra, b, rb) -> tuple[float, float, float, float]:
    """Bounding box of the intersection of two discs (they do overlap)."""
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    x = (d * d + ra * ra - rb * rb) / (2.0 * d)
    h = math.sqrt(max(ra * ra - x * x, 0.0))
    ux, uy = (b[0] - a[0]) / d, (b[1] - a[1]) / d
    mx, my = a[0] + ux * x, a[1] + uy * x
    pts = [(mx - uy * h, my + ux * h), (mx + uy * h, my - ux * h),
           (a[0] + ux * ra, a[1] + uy * ra), (b[0] - ux * rb, b[1] - uy * rb)]
    return (min(p[0] for p in pts), max(p[0] for p in pts),
            min(p[1] for p in pts), max(p[1] for p in pts))


def check_scara_clearance(spec: LineSpec, r: Result) -> None:
    """FAIL a SCARA base footprint (a `layout.scara_base.diameter` circle) that overlaps the
    conveyor, another base, a feeder or a buffer. WARN where two SCARAs' reach envelopes (r_max
    discs) overlap, naming the shared zone and what stands in it: a real line runs an interlock
    there (D-012)."""
    radius = rg.base_radius(spec)
    belt = rg.conveyor_rect(spec)
    bases = {sid: scara_base(spec, sid) for sid in spec.scaras}
    if radius is None:
        r.unresolved("scara_clearance: layout.scara_base.diameter is null")
    for sid, b in bases.items():
        if b is None:
            r.unresolved(f"{sid}: base not evaluable (its offset or station pose is null)")
    known = {sid: b for sid, b in bases.items() if b is not None}
    if radius is not None:
        others = [("conveyor", belt)] if belt is not None else []
        if belt is None:
            r.unresolved("scara_clearance: conveyor footprint (layout.conveyor.width, "
                         "conveyor.pitch, station poses)")
        for coll, kind in ((spec.feeders, "feeder"), (spec.buffers, "buffer")):
            for fid in coll:
                rect = rg.source_rect(spec, fid)
                if rect is not None:
                    others.append((f"{kind} {fid}", rect))
        for rid in spec.return_paths:
            rect = rg.return_rect(spec, rid)
            if rect is not None:
                others.append((f"return path {rid}", rect))
        for sid, b in known.items():
            for label, rect in others:
                gap = _circle_rect_gap(b[0], b[1], radius, rect)
                if gap < -rg.EPS:
                    r.add(FAIL, f"{sid}: base at ({b[0]:g}, {b[1]:g}) overlaps the {label} "
                                f"by {-gap * 1000:.1f} mm")
        ids = list(known)
        for i, a in enumerate(ids):
            for c in ids[i + 1:]:
                gap = math.hypot(known[a][0] - known[c][0], known[a][1] - known[c][1]) \
                    - 2.0 * radius
                if gap < -rg.EPS:
                    r.add(FAIL, f"{a} and {c}: bases overlap by {-gap * 1000:.1f} mm")
    ids = list(known)
    zones = []
    for i, a in enumerate(ids):
        for c in ids[i + 1:]:
            ra, rc = scara_model(spec, a).reach, scara_model(spec, c).reach
            d = math.hypot(known[a][0] - known[c][0], known[a][1] - known[c][1])
            if d >= ra + rc - rg.EPS:
                continue
            box = _lens(known[a], ra, known[c], rc)
            inside = []
            for st in spec.stations:
                rect = rg.station_rect(spec, st)
                if rect is not None and rg.in_reach(spec, a, rect) and rg.in_reach(spec, c, rect):
                    inside.append(st)
            for coll in (spec.feeders, spec.buffers):
                for fid in coll:
                    rect = rg.source_rect(spec, fid)
                    if rect is not None and rg.in_reach(spec, a, rect) \
                            and rg.in_reach(spec, c, rect):
                        inside.append(fid)
            zones.append({"scaras": [a, c], "overlap_m": round(ra + rc - d, 4),
                          "zone_xy_m": [round(v, 4) for v in box], "inside_both": inside})
            r.add(WARN, f"{a} and {c}: reach envelopes overlap by {(ra + rc - d) * 1000:.0f} mm; "
                        f"shared zone x {box[0]:.3f}..{box[1]:.3f}, y {box[2]:.3f}..{box[3]:.3f}"
                        + (f" (both reach {', '.join(inside)})" if inside else "")
                        + " — needs an interlock")
    r.info["shared_zones"] = zones


def check_robot_load(spec: LineSpec, r: Result) -> None:
    """Each SCARA's pick-and-place time per drone (tools/estimate_moves.py), at the calibrated
    motion x cycle_derate (D-014), must fit its budget: takt_s - its station's index. FAIL when
    the stored v_max/a_max no longer reproduce the model's standard_cycle; WARN at a step whose
    payload exceeds the standard cycle's (its times are optimistic)."""
    doc = estimate_moves.estimate(spec, r.cfg["cad"])
    for e in doc["errors"]:
        if e.startswith("robot_models."):
            r.add(FAIL, e)
    for sid, row in doc["scaras"].items():
        for u in row["unresolved"]:
            r.unresolved(f"{sid}: {u}")
        for w in row["warnings"]:
            r.add(WARN, f"{sid}: {w}")
        if row["total_s"] is not None and row["total_s"] > row["budget_s"]:
            r.add(FAIL, f"{sid}: {row['total_s']:.2f} s of moves per drone at derate "
                        f"{doc['cycle_derate']:g} > budget {row['budget_s']:.2f} s "
                        f"(takt {doc['takt_s']:g} - index {row['index_s']:.2f})")
    r.info["load_pct"] = {sid: g["load_pct"] for sid, g in doc["scaras"].items()}
    r.info["total_s"] = {sid: g["total_s"] for sid, g in doc["scaras"].items()}
    r.info["budget_s"] = {sid: g["budget_s"] for sid, g in doc["scaras"].items()}
    r.info["calibration"] = doc["calibration"]
    r.info["payload_warnings"] = doc["warnings"]


def check_station_time(spec: LineSpec, r: Result) -> None:
    """Per station (D-011..D-014): its SCARA step times (park moves in) + gang head drive_s x
    passes + slide t_in + t_out + actuate + its index (the distance the pallet travels to enter
    it / belt_speed + stop unit) must fit in takt_s (tools/estimate_moves.py station_times)."""
    doc = estimate_moves.estimate(spec, r.cfg["cad"])
    rows = estimate_moves.station_times(spec, doc)
    r.info["stations"] = rows
    r.info["queue_gaps"] = doc["queue_gaps"]
    for sid, row in rows.items():
        for u in row["unresolved"]:
            r.unresolved(f"{sid}: {u}")
        if row["status"] == "FAIL":
            parts = ", ".join(f"{s['id']} {s['time_s']:.2f}" for s in row["steps"])
            slide = f", slide {row['slide_s']:.2f}" if row["slide_s"] else ""
            r.add(FAIL, f"{sid}: {row['total_s']:.2f} s ({parts}{slide}, index "
                        f"{row['index_s']:.2f}) > takt {row['takt_s']:g} s")


def check_park_clearance(spec: LineSpec, r: Result) -> None:
    """D-014: while a station's gang head is in its working position its SCARA is at
    `park_pose`. The arm there — link capsules sized from the vendor STEP (vendor_<model>.json),
    the shaft and the end-effector's tools, TCP at the top of its stroke — against every head's
    working envelope at that station (the head at its working position plus its slide frame).
    D-015: the clearance is in plan (XY): no part of the parked arm may be over or under the
    working envelope; height separation is only extra margin (reported as clearance_3d_m).
    FAIL on any plan overlap or a park pose outside the joint limits. A null park pose, a missing
    capsule report or a null frame is UNRESOLVED."""
    r.info["stations"] = {}
    for sid, sc in spec.scaras.items():
        ends, starts = rg.park_steps(spec, sid)
        if not ends and not starts:
            continue
        joints = rg.park_joints(spec, sid)
        if joints is None:
            r.unresolved(f"scaras.{sid}.park_pose is null ({sc.station} drives a gang head)")
            continue
        k = rg.kin(spec, sid)
        if k is not None and (abs(joints[0]) > k.j1_lim + rg.EPS
                              or abs(joints[1]) > k.j2_lim + rg.EPS):
            r.add(FAIL, f"{sid}: park_pose j1 {math.degrees(joints[0]):.1f}°, j2 "
                        f"{math.degrees(joints[1]):.1f}° is outside ±"
                        f"{math.degrees(k.j1_lim):g}° / ±{math.degrees(k.j2_lim):g}°")
        pc = rg.park_clearance(spec, sid, joints, r.cfg["cad"])
        for u in pc["unresolved"]:
            r.unresolved(u)
        r.info["stations"][sc.station] = {
            "scara": sid, "park_pose_deg": [round(math.degrees(j), 2) for j in joints],
            "heads": pc["heads"], "clearance_m": pc["clearance_m"],
            "clearance_3d_m": pc["clearance_3d_m"], "nearest": pc["nearest"]}
        for pair in pc["pairs"]:
            if pair["clearance_m"] < -rg.EPS:
                r.add(FAIL, f"{sid} parked at j1 {math.degrees(joints[0]):.1f}°, j2 "
                            f"{math.degrees(joints[1]):.1f}°: its {pair['arm']} overlaps "
                            f"{pair['head']} {pair['part']} in plan by "
                            f"{-pair['clearance_m'] * 1000:.1f} mm (3D "
                            f"{pair['clearance_3d_m'] * 1000:.1f} mm)")


def check_path_clearance(spec: LineSpec, r: Result) -> None:
    """D-015 (D-014 M): every SCARA path — start -> pick, pick -> place, place -> end (home or
    park), and the dual-tool switch — sampled along its joint-space path, the arm's capsules
    (links, shaft, tools; TCP at the step's safe_z, down to the pick/place, up to the top of the
    stroke on a park leg) against every gang head at its station waiting retracted on its slide
    and that head's slide frame. FAIL on contact or a 3D clearance under PATH_CLEAR_M."""
    need = r.cfg.get("path_clear", rg.PATH_CLEAR_M)
    r.info["need_m"] = need
    r.info["stations"] = {}
    for sid, sc in spec.scaras.items():
        pc = rg.path_clearance(spec, sid, r.cfg["cad"])
        for u in pc["unresolved"]:
            r.unresolved(u)
        if not pc["heads"]:
            continue
        r.info["stations"][sc.station] = {"scara": sid, "heads": pc["heads"],
                                          "clearance_m": pc["clearance_m"],
                                          "nearest": pc["nearest"], "steps": pc["steps"]}
        for step, legs in pc["steps"].items():
            for leg, d in legs.items():
                if d["clearance_m"] < need - rg.EPS:
                    r.add(FAIL, f"{sid}: {step} {leg}: its {d['arm']} comes "
                                f"{d['clearance_m'] * 1000:.1f} mm from {d['head']} {d['part']};"
                                f" needs {need * 1000:g} mm")


def check_refill_clearance(spec: LineSpec, r: Result) -> None:
    """D-015: a buffer a source refills while the SCARA is parked keeps its refill envelope (the
    column over its footprint from its pick plane up) free: the parked arm and its legs into and
    out of park must not cross it. A leg out of park that picks from that buffer is excluded (the
    refill finishes first). FAIL on any crossing."""
    r.info["stations"] = {}
    for sid, sc in spec.scaras.items():
        rc = rg.refill_clearance(spec, sid, r.cfg["cad"])
        for u in rc["unresolved"]:
            r.unresolved(u)
        if not rc["buffers"]:
            continue
        r.info["stations"][sc.station] = {"scara": sid, "clearance_m": rc["clearance_m"],
                                          "nearest": rc["nearest"], "buffers": rc["buffers"],
                                          "excluded": rc["excluded"]}
        for bid, row in rc["buffers"].items():
            for where, d in row.items():
                if d["clearance_m"] < -rg.EPS:
                    r.add(FAIL, f"{sid}: {d['at']}: its {d['arm']} crosses {bid}'s refill "
                                f"envelope by {-d['clearance_m'] * 1000:.1f} mm")


def check_slide_clearance(spec: LineSpec, r: Result) -> None:
    """D-014: at every station with a head on a slide, the swept path of every part its SCARA
    carries (the part's radius + its tool's offset_r, from its bottom at the step's safe_z down
    to the pick/place at the ends) must clear the head waiting retracted on the slide by
    SLIDE_CLEAR_M."""
    need = r.cfg.get("slide_clear", SLIDE_CLEAR_M)
    r.info["need_m"] = need
    r.info["stations"] = {}
    for hid, h in spec.gang_heads.items():
        if h.slide is None:
            continue
        sc = rg.slide_clearance(spec, hid, r.cfg["cad"])
        for u in sc["unresolved"]:
            r.unresolved(f"{hid}: {u}")
        if sc["station"] is None:
            continue
        r.info["stations"][sc["station"]] = {"head": hid, "clearance_m": sc["clearance_m"],
                                             "steps": sc["steps"]}
        for step, d in sc["steps"].items():
            if d["clearance_m"] < need - rg.EPS:
                r.add(FAIL, f"{sc['station']}: {step} carries {d['part']} (band "
                            f"{d['band_m']:.3f} m, at safe_z {d['safe_z']:.3f}) "
                            f"{d['clearance_m'] * 1000:.1f} mm from {hid} retracted on its "
                            f"slide; needs {need * 1000:g} mm")


def check_safe_z(spec: LineSpec, r: Result) -> None:
    """Per SCARA step (D-013), safe_z = the tallest item under the band its tool (and the part it
    carries) sweeps along the joint-space path home -> pick -> place -> home, plus the dual-tool
    switch's disc at home, + the carried part's height + z_margin (tools/robot_geometry.py) — not
    everything in its reach circle. FAIL when it is above the model's z_stroke (base z +
    z_stroke): the TCP could not travel over what is under its path."""
    r.info["safe_z"] = {}
    for sid in spec.scaras:
        sz = rg.safe_z(spec, sid, r.cfg["cad"])
        r.info["safe_z"][sid] = sz.doc()
        for u in sz.unresolved:
            r.unresolved(u)
        k = rg.kin(spec, sid)
        if k is None:
            continue
        for step, d in sz.steps.items():
            if d["safe_z"] is not None and d["safe_z"] > k.z_hi + rg.EPS:
                r.add(FAIL, f"{sid}: {step} needs safe_z={d['safe_z']:.3f} ({d['tallest_item'][0]}"
                            f" {d['tallest_item'][1]:.3f} + {d['hang'][0]} {d['hang'][1]:.3f} + "
                            f"margin {spec.scara_defaults.z_margin:g}), above its z range top "
                            f"{k.z_hi:g} (z_stroke {scara_model(spec, sid).z_stroke:g})")


def check_payload(spec: LineSpec, r: Result) -> None:
    """Every tool on the end-effector + the heaviest part a step lifts (with what rides in it)
    <= the robot model's payload_rated (D-013). A null mass is UNRESOLVED."""
    for sid in spec.scaras:
        w = rg.wrist_load(spec, sid, r.cfg["cad"])
        r.info.setdefault("scaras", {})[sid] = {k: w[k] for k in (
            "tools_kg", "part", "part_kg", "payload_kg", "payload_rated")}
        if w["missing"]:
            r.unresolved(f"{sid}: payload not evaluable ({', '.join(w['missing'])})")
        elif w["payload_kg"] > w["payload_rated"] + rg.EPS:
            r.add(FAIL, f"{sid}: {w['payload_kg']:.3f} kg (tools {w['tools_kg']:.3f} + "
                        f"{w['part']} {w['part_kg']:.3f}) > payload_rated "
                        f"{w['payload_rated']:g} kg")


def check_wrist_inertia(spec: LineSpec, r: Result) -> None:
    """Sum of (I_cm + m r^2) about J4 for the tools and the carried part (D-013, docs/12 §4).
    Above the model's inertia_rated the robot still works with reduced J4 acceleration: WARN.
    Above inertia_max it cannot carry it: FAIL (D-015). UNRESOLVED if any mass is null, or
    inertia_max is not stated."""
    for sid in spec.scaras:
        w = rg.wrist_load(spec, sid, r.cfg["cad"])
        imax = scara_model(spec, sid).inertia_max
        r.info.setdefault("scaras", {})[sid] = {k: w[k] for k in (
            "tools_inertia", "part", "part_inertia", "inertia", "inertia_rated")} | {
            "inertia_max": imax}
        if w["missing"]:
            r.unresolved(f"{sid}: wrist inertia not evaluable ({', '.join(w['missing'])})")
            continue
        detail = (f"{w['inertia']:.4f} kg m^2 about J4 (tools {w['tools_inertia']:.4f} + "
                  f"{w['part']} {w['part_inertia']:.4f})")
        if imax is None:
            r.unresolved(f"robot_models.{spec.scaras[sid].model}.inertia_max is not stated")
        elif w["inertia"] > imax + rg.EPS:
            r.add(FAIL, f"{sid}: {detail} > inertia_max {imax:g}")
            continue
        if w["inertia"] > w["inertia_rated"] + rg.EPS:
            r.add(WARN, f"{sid}: {detail} > inertia_rated {w['inertia_rated']:g}: J4 "
                        "acceleration must be reduced")


def check_feeder_rate(spec: LineSpec, r: Result) -> None:
    """Per takt one pallet passes, so demand per takt = quantity used per pallet.
    magazine/step: demand <= capacity / refill_s * takt_s (refill rate, parts per takt).
    blow: demand <= takt_s / feed_s (one fastener per feed_s).
    Demand: fasteners = step.qty; parts drawn via `from` = qty on `place` steps, else 1, times
    the feeder's `per_pick` (a nest hands over a whole set per pick)."""
    takt = spec.line.takt_s
    demand: dict[str, float] = {}
    unknown_demand: set[str] = set()
    for s in _active_steps(spec):
        if s.fastener and s.fastener in spec.fasteners:
            fd = spec.fasteners[s.fastener].feeder
            if s.qty is None:
                unknown_demand.add(fd)
            else:
                demand[fd] = demand.get(fd, 0) + s.qty
        if s.from_ in spec.feeders:
            n = s.qty if s.capability == "place" and s.qty else 1
            demand[s.from_] = demand.get(s.from_, 0) + n * (spec.feeders[s.from_].per_pick or 1)
    for name in sorted(set(demand) | unknown_demand):
        f = spec.feeders.get(name)
        if f is None:
            continue  # dangling feeder reference: reported by schema
        need = demand.get(name, 0)
        if name in unknown_demand:
            r.unresolved(f"{name}: demand per takt (step qty null)")
            continue
        if f.type == "blow":
            if f.feed_s is None:
                r.unresolved(f"{name}: feed_s")
                continue
            supply = takt / f.feed_s
        else:
            if f.capacity is None or f.refill_s is None:
                r.unresolved(f"{name}: capacity/refill_s")
                continue
            supply = f.capacity / f.refill_s * takt
        if need > supply:
            r.add(FAIL, f"{name}: demand {need:g}/takt > supply {supply:.2f}/takt")


def check_buffer(spec: LineSpec, r: Result) -> None:
    takt = spec.line.takt_s
    for name, s in spec.sources.items():
        if s.interval_s > takt:
            r.add(FAIL, f"{name}: interval {s.interval_s:g} s > takt {takt:g} s")
        if s.buffer_capacity is None:
            r.unresolved(f"{name}: buffer_capacity for {s.buffer}")
        elif s.buffer_capacity < r.cfg["min_buffer"]:
            r.add(FAIL, f"{name}: buffer {s.buffer} capacity {s.buffer_capacity} < "
                        f"minimum {r.cfg['min_buffer']}")


def check_fastener_lengths(spec: LineSpec, r: Result) -> None:
    lengths = sorted({f.length for f in spec.fasteners.values()})
    r.info["distinct_lengths_m"] = lengths
    if len(lengths) > r.cfg["max_lengths"]:
        r.add(WARN, f"{len(lengths)} distinct fastener lengths (each is a feeder) > "
                    f"threshold {r.cfg['max_lengths']}")


def check_no_flip(spec: LineSpec, r: Result) -> None:
    for s in _active_steps(spec):
        if s.head in spec.gang_heads and spec.gang_heads[s.head].direction == "up":
            r.add(FAIL, f"{s.id}: head {s.head} fastens from below (needs a flip)")


def check_datums(spec: LineSpec, r: Result) -> None:
    """Plates in the stack need `round` and `diamond`, at the same XY on every plate."""
    ref: dict[str, tuple[str, float]] = {}
    for name in PLATE_STACK:
        part = spec.parts.get(name)
        if part is None:
            continue
        for d in ("round", "diamond"):
            xy = getattr(part.datums, d) if part.datums else None
            if xy is None:
                r.unresolved(f"{name}.datums.{d} not defined")
                continue
            for axis, v in zip("xy", xy):
                if v is None:
                    r.unresolved(f"{name}.datums.{d}.{axis}")
                    continue
                key = f"{d}.{axis}"
                if key not in ref:
                    ref[key] = (name, v)
                elif abs(v - ref[key][1]) > DATUM_TOL_M:
                    r.add(FAIL, f"{name}.datums.{d}.{axis}={v:g} differs from "
                                f"{ref[key][0]} ({ref[key][1]:g})")


def check_arm_pin_clearance(spec: LineSpec, r: Result) -> None:
    """Option B (D-012): the pallet's arm pins go through dedicated arm tooling holes, because the
    screw holes must stay free for the gang head. FAIL a tooling hole within PIN_CLEAR_SCREW_D
    screw diameters of any arm screw hole, or a count that is not one per arm. The screw is the
    fastener of the step whose gang head drives the arm-hole pattern."""
    plate = spec.parts.get("bottom_plate")
    holes = plate.arm_holes if plate is not None else None
    tooling = spec.drone.arm_tooling_holes if spec.drone is not None else None
    if holes is None:
        r.unresolved("parts.bottom_plate.arm_holes is not defined")
        return
    if tooling is None:
        r.unresolved("drone.arm_tooling_holes is not defined")
        return
    arms = len(holes.positions) // holes.per_arm
    if len(tooling) != arms:
        r.add(FAIL, f"drone.arm_tooling_holes has {len(tooling)} hole(s) for {arms} arm(s); "
                    "one pin per arm")
    pattern_heads = {h for h, gh in spec.gang_heads.items()
                     if gh.pattern_from == "bottom_plate.arm_holes"}
    fasteners = {s.fastener for s in spec.sequence if s.head in pattern_heads and s.fastener}
    ds = [spec.fasteners[f].d for f in fasteners if f in spec.fasteners]
    if not ds or None in ds:
        r.unresolved("arm screw diameter: no fastener with `d` is driven into "
                     "bottom_plate.arm_holes")
        return
    limit = PIN_CLEAR_SCREW_D * max(ds)
    r.info["min_clearance_m"] = limit
    nearest = None
    for i, th in enumerate(tooling):
        if None in th:
            r.unresolved(f"drone.arm_tooling_holes[{i}]")
            continue
        for j, h in enumerate(holes.positions):
            if None in h:
                r.unresolved(f"parts.bottom_plate.arm_holes.positions[{j}]")
                continue
            d = math.hypot(th[0] - h[0], th[1] - h[1])
            nearest = d if nearest is None else min(nearest, d)
            if d < limit - rg.EPS:
                r.add(FAIL, f"drone.arm_tooling_holes[{i}] {list(th)} is {d * 1000:.2f} mm from "
                            f"arm screw hole {j} {list(h)}, less than {PIN_CLEAR_SCREW_D:g} x "
                            f"Ø{max(ds) * 1000:g} = {limit * 1000:g} mm")
    r.info["nearest_m"] = nearest and round(nearest, 6)


CHECKS = [
    ("schema", check_schema),
    ("actors", check_actors),
    ("unresolved", check_unresolved),
    ("precedence", check_precedence),
    ("takt", check_takt),
    ("line_balance", check_line_balance),
    ("anchors", check_anchors),
    ("scara_reach", check_scara_reach),
    ("scara_clearance", check_scara_clearance),
    ("safe_z", check_safe_z),
    ("park_clearance", check_park_clearance),
    ("slide_clearance", check_slide_clearance),
    ("path_clearance", check_path_clearance),
    ("refill_clearance", check_refill_clearance),
    ("payload", check_payload),
    ("wrist_inertia", check_wrist_inertia),
    ("robot_load", check_robot_load),
    ("station_time", check_station_time),
    ("feeder_rate", check_feeder_rate),
    ("buffer", check_buffer),
    ("fastener_lengths", check_fastener_lengths),
    ("no_flip", check_no_flip),
    ("datums", check_datums),
    ("arm_pin_clearance", check_arm_pin_clearance),
]


def run_checks(raw: dict, min_buffer: int = MIN_BUFFER_CAPACITY,
               max_lengths: int = MAX_FASTENER_LENGTHS, cad_dir: Path | None = None,
               dev: bool = False) -> tuple[list[Result], bool]:
    """Run every check on the raw spec data. Returns (results, passed).

    `cad_dir`/`dev` say where the build123d parts' reports are (robot_geometry.CadHeights):
    safe_z, robot_load and station_time read those parts' heights from them."""
    cad = rg.CadHeights(cad_dir, dev)
    results: list[Result] = []
    try:
        spec = parse_spec(raw)
    except SpecError as exc:
        r = Result("schema")
        for e in exc.errors:
            r.add(FAIL, e)
        results.append(r)
        results += [Result(n, [(SKIP, "spec did not parse")]) for n, _ in CHECKS[1:]]
        return results, False
    for name, fn in CHECKS:
        r = Result(name, cfg={"min_buffer": min_buffer, "max_lengths": max_lengths, "cad": cad})
        fn(spec, r, raw) if fn is check_unresolved else fn(spec, r)
        results.append(r)
    return results, all(r.status != FAIL for r in results)


# --------------------------------------------------------------------------- output


def _count(r: Result) -> str:
    return f"{sum(1 for l, _ in r.findings if l == FAIL)} failure(s)"


def _summary(r: Result) -> str:
    if r.name == "unresolved":
        return f"{r.info.get('count', 0)} null value(s)"
    if r.name == "line_balance" and "bottleneck" in r.info:
        return f"bottleneck {r.info['bottleneck']} at {r.info['bottleneck_pct']}%"
    if r.name == "robot_load" and r.info.get("load_pct") \
            and any(p is not None for p in r.info["load_pct"].values()):
        return ", ".join(f"{g} {p}%" for g, p in r.info["load_pct"].items() if p is not None) \
            + " of takt - index" + (" — " + _count(r) if r.status == FAIL else "")
    if r.name == "station_time" and r.status != WARN and r.info.get("stations"):
        done = {s: row for s, row in r.info["stations"].items() if row["total_s"] is not None}
        if done:
            top = max(done, key=lambda s: done[s]["total_s"])
            return (f"max {top} {done[top]['total_s']:.2f} s of {done[top]['takt_s']:g} s"
                    + (" — " + _count(r) if r.status == FAIL else ""))
    if r.name in ("park_clearance", "slide_clearance", "path_clearance", "refill_clearance") \
            and r.info.get("stations") \
            and any(d["clearance_m"] is not None for d in r.info["stations"].values()):
        return ", ".join(f"{st} {d['clearance_m'] * 1000:.0f} mm"
                         for st, d in r.info["stations"].items()
                         if d["clearance_m"] is not None) \
            + (" — " + _count(r) if r.status == FAIL else "")
    if r.name == "safe_z" and r.status != WARN and r.info.get("safe_z"):
        return f"max {max(d['safe_z'] for d in r.info['safe_z'].values() if d['safe_z'] is not None):.3f} m" \
            + (" — " + _count(r) if r.status == FAIL else "") \
            if any(d["safe_z"] is not None for d in r.info["safe_z"].values()) else "ok"
    if r.name in ("payload", "wrist_inertia") and r.info.get("scaras") and r.status != WARN \
            or r.name == "wrist_inertia" and r.info.get("scaras") and not any(
                m.startswith("UNRESOLVED") for _, m in r.findings):
        key, rated = (("payload_kg", "payload_rated") if r.name == "payload"
                      else ("inertia", "inertia_rated"))
        return ", ".join(f"{s} {d[key]:g}/{d[rated]:g}" for s, d in r.info["scaras"].items()) \
            + (" — " + _count(r) if r.status == FAIL else "")
    n = {lvl: sum(1 for l, _ in r.findings if l == lvl) for lvl in (FAIL, WARN)}
    if r.status == FAIL:
        return f"{n[FAIL]} failure(s)"
    if r.status == WARN:
        return f"{n[WARN]} warning(s)"
    return "ok"


def print_table(results: list[Result], passed: bool, verbose: bool) -> None:
    print(f"{'check':<18}{'status':<7}detail")
    print("-" * 60)
    for r in results:
        print(f"{r.name:<18}{r.status:<7}{_summary(r)}")
    print("-" * 60)
    for r in results:
        for lvl, msg in r.findings:
            # The shared reach zones are always shown: each one needs an interlock (D-012); so is
            # a payload above the standard cycle's (D-014).
            if lvl == FAIL or (verbose and lvl in (WARN, SKIP)) \
                    or (r.name in ("scara_clearance", "robot_load") and lvl == WARN
                        and not msg.startswith("UNRESOLVED")):
                print(f"  [{lvl}] {r.name}: {msg}")
    st = next((r for r in results if r.name == "station_time"), None)
    if st is not None and st.info.get("stations"):
        print("station_time = SCARA steps (+ tool switches, park moves) + drive_s x passes + "
              "slide t_in/t_out + actuate + index (enter distance / belt_speed + stop unit):")
        estimate_moves.print_station_table(st.info["stations"])
        estimate_moves.print_gaps({"queue_gaps": st.info.get("queue_gaps", [])})
        print("-" * 60)
    warns = sum(1 for r in results for lvl, m in r.findings
                if lvl == WARN and (r.name not in ("scara_clearance", "robot_load")
                                    or m.startswith("UNRESOLVED")))
    if warns and not verbose:
        print(f"  ({warns} warning(s) not shown; -v or see the report)")
    print("line_check:", "PASS" if passed else "FAIL")


def write_report(results: list[Result], passed: bool, spec_path: Path, report: Path,
                 elapsed_s: float, merged: Merged | None = None, dev: bool = False) -> None:
    def rel(p: Path) -> str:
        try:
            return str(p.resolve().relative_to(REPO))
        except ValueError:
            return str(p)

    unresolved = next((r.info.get("paths", []) for r in results if r.name == "unresolved"), [])
    doc = {
        "passed": passed,
        "dev": dev,  # true: numbers include overlay placeholders; not a deliverable
        "spec": rel(spec_path),
        "placeholders": merged.placeholders if merged else [],
        "shadowed_overlay": merged.shadowed if merged else [],
        "elapsed_s": round(elapsed_s, 3),
        "unresolved": unresolved,
        "checks": {
            r.name: {
                "status": r.status,
                "info": {k: v for k, v in r.info.items() if k != "paths"},
                "findings": [{"level": lvl, "message": msg} for lvl, msg in r.findings],
            } for r in results
        },
    }
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(doc, indent=2) + "\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--report", type=Path, default=None,
                    help="default: reports/line_check.json (line_check.dev.json with --dev)")
    add_dev_argument(ap)
    ap.add_argument("--min-buffer", type=int, default=MIN_BUFFER_CAPACITY,
                    help="minimum buffer capacity (default %(default)s)")
    ap.add_argument("--max-fastener-lengths", type=int, default=MAX_FASTENER_LENGTHS,
                    help="WARN above this many distinct lengths (default %(default)s)")
    ap.add_argument("--cad-reports", type=Path, default=None,
                    help=f"where cad_<part>.json live (default {rg.CAD_REPORTS})")
    ap.add_argument("--milestone", default=None,
                    help="check only this milestone's stations and their robots, feeders and "
                         "buffers (D-012); report reports/line_check.<name>[.dev].json")
    ap.add_argument("--stations", nargs="+", default=None,
                    help="check only these consecutive stations (comma or space separated) and "
                         "their robots, heads, feeders, buffers and return paths (D-015)")
    ap.add_argument("--name", default="batch",
                    help="report name for --stations: reports/line_check.<name>[.dev].json")
    ap.add_argument("-v", "--verbose", action="store_true", help="also print warnings")
    args = ap.parse_args(argv)
    stations = parse_station_list(args.stations)
    if stations and args.milestone:
        ap.error("--milestone and --stations are exclusive")

    name = (f"line_check.{args.milestone}" if args.milestone
            else f"line_check.{args.name}" if stations else "line_check")
    report = args.report or report_path(name, args.dev)
    t0 = time.monotonic()
    try:
        merged = load_merged(args.spec, args.dev)
        if args.milestone:
            merged = Merged(milestone_subset(merged.raw, args.milestone), merged.placeholders,
                            merged.shadowed)
        elif stations:
            merged = Merged(station_subset(merged.raw, stations), merged.placeholders,
                            merged.shadowed)
    except SpecError as exc:
        write_report([Result("schema", [(FAIL, e) for e in exc.errors])], False, args.spec,
                     report, time.monotonic() - t0, dev=args.dev)
        print(f"line_check: FAIL — {exc}", file=sys.stderr)
        return 1
    results, passed = run_checks(merged.raw, args.min_buffer, args.max_fastener_lengths,
                                 args.cad_reports, args.dev)
    write_report(results, passed, args.spec, report, time.monotonic() - t0, merged, args.dev)
    if args.dev:
        print(f"DEV MODE: {len(merged.placeholders)} placeholder value(s) from the overlay; "
              "results are not deliverable")
        for sh in merged.shadowed:
            print(f"  [WARN] overlay {sh['path']}={sh['overlay']!r} ignored: base has {sh['base']!r}")
    if args.milestone:
        print(f"MILESTONE {args.milestone}: {', '.join(merged.raw['stations'])}")
    elif stations:
        print(f"STATIONS {args.name}: {', '.join(merged.raw['stations'])}")
    print_table(results, passed, args.verbose)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
