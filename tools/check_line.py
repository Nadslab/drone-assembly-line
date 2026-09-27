"""Static checks on line_spec.yaml (P2). No ROS, no Gazebo.

    tools/.venv/bin/python tools/check_line.py [--spec PATH] [--report PATH] [-v]

Prints a table, writes reports/line_check.json (`passed: true|false`), and exits non-zero on
any FAIL. WARN never fails the run: a `null` in the spec is WARN "UNRESOLVED" until the phase
that needs the value.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import estimate_moves  # noqa: E402
import gantry_geometry as gg  # noqa: E402
from spec import (DEFAULT_SPEC, REPO, LineSpec, Merged, SpecError, add_dev_argument,  # noqa: E402
                  gantry_served, load_merged, parse_spec, report_path, zone_owner)

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"

# Thresholds that are not in the spec; override on the command line.
MIN_BUFFER_CAPACITY = 1
MAX_FASTENER_LENGTHS = 3
DATUM_TOL_M = 1e-4
# Plates that sit on the pallet pins and must share datum positions (summary §7.2).
PLATE_STACK = ("bottom_plate", "mid_plate", "top_plate")


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
    heads = set(spec.gang_heads) | set(spec.scaras)
    tools = {t for g in spec.gantries.values() for t in g.tools}
    stations, steps = set(spec.stations), [s.id for s in spec.sequence]
    owner = zone_owner(spec)

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
        gid = owner.get(s.station)
        if s.tool in tools and gid and s.tool not in spec.gantries[gid].tools:
            r.add(FAIL, f"{w}.tool -> '{s.tool}' is not on {gid}, which serves {s.station}")
        ref(f"{w}.head", "head", s.head, heads)
        ref(f"{w}.fastener", "fastener", s.fastener, set(spec.fasteners))
        for req in s.requires:
            ref(f"{w}.requires", "step", req, set(steps))
    if spec.layout is not None:
        for key in spec.layout.feeder_footprints:
            ref("layout.feeder_footprints", "feeder/buffer", key, feeders | buffers)
    idx = [st.index for st in spec.stations.values()]
    if len(idx) != len(set(idx)):
        r.add(FAIL, "stations: duplicate index")


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


def _pose_in_gantry(spec: LineSpec, gid: str, label: str, pose, r: Result) -> None:
    """Pose in line frame -> that gantry's frame (pose - mount) must be inside its axis limits."""
    g = spec.gantries[gid]
    unknown = False
    for i, ax in enumerate("xyz"):
        lim, p, m = getattr(g.axes, ax), pose[i], g.mount[i]
        if p is None or m is None or lim.min is None or lim.max is None:
            unknown = True
            continue
        rel = p - m
        if not lim.min <= rel <= lim.max:
            r.add(FAIL, f"{label}: {ax}={rel:.3f} m ({gid} frame) outside "
                        f"[{lim.min:g}, {lim.max:g}]")
    yaw = pose[3]
    if yaw is not None and yaw != 0:
        c = g.axes.c
        if c is None:
            r.add(FAIL, f"{label}: needs yaw {yaw:g} rad but {gid} has no C axis")
        elif c.min is not None and c.max is not None and not c.min <= yaw <= c.max:
            r.add(FAIL, f"{label}: yaw {yaw:g} outside {gid} C [{c.min:g}, {c.max:g}]")
    if unknown:
        r.unresolved(f"{label}: reach of {gid} not fully evaluable (pose, mount or axis "
                     "limits null)")


def check_reach(spec: LineSpec, r: Result) -> None:
    """Gantry poses, per zone: every step a gantry performs has its station and its `from`
    (feeder or buffer) inside the axis limits of the gantry whose zone holds that station. Drive
    poses: SCARA stations must be within `reach` of the SCARA base; gang heads are fixed-pattern,
    so only their station pose matters (checked for existence by the schema)."""
    owner = zone_owner(spec)
    seen: set[tuple[str, str]] = set()
    for s in _active_steps(spec):
        if gantry_served(s) and s.station in spec.stations:
            gid = owner.get(s.station)
            if gid is None:
                r.unresolved(f"{s.id}: station {s.station} is in no known zone")
            else:
                targets = [(f"station {s.station}", spec.stations[s.station].pose)]
                if s.from_ in spec.feeders and spec.feeders[s.from_].pose is not None:
                    targets.append((f"feeder {s.from_}", spec.feeders[s.from_].pose))
                elif s.from_ in spec.buffers:
                    targets.append((f"buffer {s.from_}", spec.buffers[s.from_].pose))
                for label, pose in targets:
                    if (gid, label) not in seen:
                        seen.add((gid, label))
                        _pose_in_gantry(spec, gid, label, pose, r)
        if s.capability == "drive" and s.head in spec.scaras and s.station in spec.stations:
            sc, pose = spec.scaras[s.head], spec.stations[s.station].pose
            if sc.base is None or sc.reach is None or None in pose[:2] or None in sc.base[:2]:
                r.unresolved(f"{s.id}: SCARA {s.head} reach not evaluable")
            else:
                d = ((pose[0] - sc.base[0]) ** 2 + (pose[1] - sc.base[1]) ** 2) ** 0.5
                if d > sc.reach:
                    r.add(FAIL, f"{s.id}: {s.station} is {d:.3f} m from {s.head}, "
                                f"reach {sc.reach:g} m")


def check_gantry_load(spec: LineSpec, r: Result) -> None:
    """Each gantry's pick-and-place time per drone (tools/estimate_moves.py), at
    plan_speed_scale, must fit the budget: takt_s - transfer_s."""
    doc = estimate_moves.estimate(spec, r.cfg["cad"])
    for msg in doc["errors"]:
        r.add(FAIL, msg)
    for msg in doc["unresolved"]:
        r.unresolved(msg)
    r.info["load_pct"] = {gid: g["load_pct"] for gid, g in doc["gantries"].items()}
    r.info["total_s"] = {gid: g["total_s"] for gid, g in doc["gantries"].items()}
    r.info["budget_s"] = doc["budget_s"]
    r.info["transfer_s"] = doc["transfer_s"]


def check_gantry_straddle(spec: LineSpec, r: Result) -> None:
    """Every gantry post stands clear of the conveyor by gantry_frame.conveyor_clearance
    (D-009): a post in the belt is a machine that cannot be built."""
    belt = gg.conveyor_rect(spec)
    clear = spec.layout.gantry_frame.conveyor_clearance if spec.layout else None
    if belt is None or clear is None:
        r.unresolved("gantry_straddle: conveyor footprint or gantry_frame.conveyor_clearance "
                     "(layout.conveyor.width, conveyor.pitch, station poses)")
        return
    r.info["conveyor_mm"] = [round(v * 1000.0, 1) for v in belt.doc()]
    for gid in spec.gantries:
        posts = gg.post_rects(spec, gid)
        if posts is None:
            r.unresolved(f"{gid}: post footprints (travel limits, mount or post_size null)")
            continue
        for k, post in enumerate(posts, start=1):
            gap = post.gap(belt)
            if gap < 0:
                r.add(FAIL, f"{gid} post {k} at x,y {post.doc()} stands in the conveyor "
                            f"({-gap * 1000:.1f} mm deep)")
            elif gap < clear - gg.EPS:
                r.add(FAIL, f"{gid} post {k} is {gap * 1000:.1f} mm from the conveyor edge, "
                            f"less than gantry_frame.conveyor_clearance {clear * 1000:g} mm")


def check_safe_z(spec: LineSpec, r: Result) -> None:
    """Per gantry, safe_z = tallest item under it + tallest part it carries + z_margin
    (tools/gantry_geometry.py). No item under it may reach above safe_z - z_margin, and safe_z
    has to be inside the gantry's own z travel or it cannot move at that height."""
    margin = spec.gantry_defaults.z_margin
    r.info["safe_z"] = {}
    for gid, g in spec.gantries.items():
        sz = gg.safe_z(spec, gid, r.cfg["cad"])
        r.info["safe_z"][gid] = sz.doc()
        for u in sz.unresolved:
            r.unresolved(u)
        if sz.value is None:
            continue
        for label, top in sz.items:
            if top > sz.value - margin + gg.EPS:
                r.add(FAIL, f"{gid}: {label} reaches z={top:.3f}, above safe_z - z_margin "
                            f"= {sz.value - margin:.3f}")
        if g.mount[2] is None or g.axes.z.min is None or g.axes.z.max is None:
            r.unresolved(f"{gid}: z travel null; cannot tell whether safe_z is reachable")
        elif not g.axes.z.min - gg.EPS <= sz.value - g.mount[2] <= g.axes.z.max + gg.EPS:
            r.add(FAIL, f"{gid}: safe_z={sz.value:.3f} ({sz.tallest[0]} {sz.tallest[1]:.3f} + "
                        f"{sz.hang[0]} {sz.hang[1]:.3f} + margin {margin:g}) is outside its z "
                        f"travel [{g.mount[2] + g.axes.z.min:g}, {g.mount[2] + g.axes.z.max:g}]")


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


CHECKS = [
    ("schema", check_schema),
    ("unresolved", check_unresolved),
    ("precedence", check_precedence),
    ("takt", check_takt),
    ("line_balance", check_line_balance),
    ("reach", check_reach),
    ("gantry_load", check_gantry_load),
    ("gantry_straddle", check_gantry_straddle),
    ("safe_z", check_safe_z),
    ("feeder_rate", check_feeder_rate),
    ("buffer", check_buffer),
    ("fastener_lengths", check_fastener_lengths),
    ("no_flip", check_no_flip),
    ("datums", check_datums),
]


def run_checks(raw: dict, min_buffer: int = MIN_BUFFER_CAPACITY,
               max_lengths: int = MAX_FASTENER_LENGTHS, cad_dir: Path | None = None,
               dev: bool = False) -> tuple[list[Result], bool]:
    """Run every check on the raw spec data. Returns (results, passed).

    `cad_dir`/`dev` say where the build123d parts' reports are (gantry_geometry.CadHeights):
    safe_z and gantry_load read those parts' heights from them."""
    cad = gg.CadHeights(cad_dir, dev)
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
        return f"{r.info['count']} null value(s)"
    if r.name == "line_balance" and "bottleneck" in r.info:
        return f"bottleneck {r.info['bottleneck']} at {r.info['bottleneck_pct']}%"
    if r.name == "gantry_load" and r.status != WARN and r.info.get("load_pct"):
        return (", ".join(f"{g} {p}%" for g, p in r.info["load_pct"].items() if p is not None)
                + f" of {r.info['budget_s']:g} s")
    if r.name == "safe_z" and r.status != WARN and r.info.get("safe_z"):
        return ", ".join(f"{g} {d['safe_z']:.3f} m" for g, d in r.info["safe_z"].items()
                         if d["safe_z"] is not None) + (" — " + _count(r) if r.status == FAIL
                                                        else "")
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
            if lvl == FAIL or (verbose and lvl in (WARN, SKIP)):
                print(f"  [{lvl}] {r.name}: {msg}")
    warns = sum(1 for r in results for lvl, _ in r.findings if lvl == WARN)
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
                    help=f"where cad_<part>.json live (default {gg.CAD_REPORTS})")
    ap.add_argument("-v", "--verbose", action="store_true", help="also print warnings")
    args = ap.parse_args(argv)

    report = args.report or report_path("line_check", args.dev)
    t0 = time.monotonic()
    try:
        merged = load_merged(args.spec, args.dev)
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
    print_table(results, passed, args.verbose)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
