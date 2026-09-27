"""Static move-time estimate: how long each gantry's pick-and-places take per drone, against takt.

    tools/.venv/bin/python tools/estimate_moves.py [--spec PATH] [--dev]

No ROS, no Gazebo: arithmetic on the spec. One pick-and-place is

    XY to the pick at safe_z, Z down, grip, Z up, XY to the place, Z down, release, Z up

with `gantry_defaults.settle_s` after each of the four Z moves. Each axis follows a trapezoidal
profile from its own `v_max`/`a_max`:

    t = d/v + v/a     if d >= v^2/a      (reaches v_max)
    t = 2 sqrt(d/a)   otherwise          (triangular)

and an XY move takes the slower of X and Y. Distances come from the step's `from` pose (feeder or
buffer) and its station pose; Z runs between `safe_z` and the pose's z.

A gantry's steps are the active sequence steps it serves (a `tool`, or a `part` drawn `from`
somewhere) at a station in its zone, in sequence order. The gantry works them as a tour, once per
drone: each cycle starts where the previous one placed, and the first cycle of the first step
starts at the last step's station (the previous drone). A step with `qty: n` is n cycles, so
cycles 2..n start at the step's own station. The sum per gantry is compared with `line.takt_s`.

Writes reports/move_estimate.json (`--dev`: move_estimate.dev.json) with per-step times,
per-gantry load %, and `passed`. A null the estimate needs is UNRESOLVED, not a failure — the
check_line.py convention — and leaves that gantry's total out. Exits non-zero on a failure.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from spec import (DEFAULT_SPEC, LineSpec, SpecError, add_dev_argument, gantry_ids,  # noqa: E402
                  gantry_served, load_merged, parse_spec, report_path, zone_owner)

Z_MOVES = ("z_down_pick", "z_up_pick", "z_down_place", "z_up_place")


def axis_time(d: float, v: float, a: float) -> float:
    """Rest-to-rest time for distance `d` on one axis with a trapezoidal velocity profile."""
    d = abs(d)
    if d == 0.0:
        return 0.0
    return d / v + v / a if d >= v * v / a else 2.0 * math.sqrt(d / a)


def xy_time(frm, to, axes) -> float:
    return max(axis_time(to[0] - frm[0], axes.x.v_max, axes.x.a_max),
               axis_time(to[1] - frm[1], axes.y.v_max, axes.y.a_max))


def cycle(frm, pick, place, axes, d) -> dict:
    """One pick-and-place, split into its phases (seconds). `frm` is where the TCP starts."""
    z = axes.z
    down_pick = axis_time(d.safe_z - pick[2], z.v_max, z.a_max)
    down_place = axis_time(d.safe_z - place[2], z.v_max, z.a_max)
    out = {
        "xy_to_pick": xy_time(frm, pick, axes),
        "z_down_pick": down_pick + d.settle_s,
        "grip": d.grip_s,
        "z_up_pick": down_pick + d.settle_s,
        "xy_to_place": xy_time(pick, place, axes),
        "z_down_place": down_place + d.settle_s,
        "release": d.release_s,
        "z_up_place": down_place + d.settle_s,
    }
    out = {k: round(v, 4) for k, v in out.items()}
    out["total"] = round(sum(out.values()), 4)
    return out


def _source_pose(spec: LineSpec, source: str | None):
    if source in spec.feeders:
        return spec.feeders[source].pose
    if source in spec.buffers:
        return spec.buffers[source].pose
    return None


def _nulls(label: str, values: dict) -> list[str]:
    return [f"{label}: {k} is null" for k, v in values.items() if v is None]


def estimate(spec: LineSpec) -> dict:
    """Per-gantry move time per drone. Never raises on a null: it lands in `unresolved`."""
    takt = spec.line.takt_s
    d = spec.gantry_defaults
    owner = zone_owner(spec)
    doc: dict = {"takt_s": takt, "passed": True, "complete": True, "gantries": {},
                 "unresolved": [], "errors": [],
                 "model": {"grip_s": d.grip_s, "release_s": d.release_s,
                           "settle_s": d.settle_s, "safe_z": d.safe_z,
                           "profile": "trapezoidal per axis; XY = max(X, Y)"}}
    missing_defaults = _nulls("gantry_defaults", {"grip_s": d.grip_s, "release_s": d.release_s,
                                                  "settle_s": d.settle_s, "safe_z": d.safe_z})
    raw_ids = gantry_ids({"gantries": spec.gantries})

    for gid in raw_ids:
        g = spec.gantries[gid]
        row: dict = {"zone": g.zone, "steps": [], "total_s": None, "load_pct": None,
                     "status": "UNRESOLVED", "unresolved": list(missing_defaults), "errors": []}
        doc["gantries"][gid] = row
        if g.zone is None:
            row["unresolved"].append(f"gantries.{gid}.zone is null")
        for ax in "xyz":
            a = getattr(g.axes, ax)
            row["unresolved"] += _nulls(f"gantries.{gid}.axes.{ax}",
                                        {"v_max": a.v_max, "a_max": a.a_max})
        if d.safe_z is not None and g.mount[2] is not None \
                and None not in (g.axes.z.min, g.axes.z.max):
            rel = d.safe_z - g.mount[2]
            if not g.axes.z.min <= rel <= g.axes.z.max:
                row["errors"].append(f"gantry_defaults.safe_z={d.safe_z:g} is z={rel:.3f} in "
                                     f"{gid}'s frame, outside [{g.axes.z.min:g}, "
                                     f"{g.axes.z.max:g}]")

        steps = [s for s in spec.sequence
                 if s.status == "active" and gantry_served(s) and owner.get(s.station) == gid]
        legs = []
        for s in steps:
            pick, place = _source_pose(spec, s.from_), spec.stations[s.station].pose
            if pick is None:
                row["unresolved"].append(f"sequence.{s.id}: no pick pose (`from` names no "
                                         "feeder or buffer with a pose)")
                continue
            if None in pick[:3] or None in place[:3]:
                row["unresolved"].append(f"sequence.{s.id}: pick or place pose has a null")
                continue
            for what, pose in (("pick", pick), ("place", place)):
                if d.safe_z is not None and pose[2] > d.safe_z:
                    row["errors"].append(f"sequence.{s.id}: {what} z={pose[2]:g} is above "
                                         f"safe_z={d.safe_z:g}")
            legs.append((s, pick[:3], place[:3]))

        if row["unresolved"] or row["errors"]:
            doc["unresolved"] += [f"{gid}: {u}" for u in row["unresolved"]]
            doc["errors"] += [f"{gid}: {e}" for e in row["errors"]]
            row["status"] = "FAIL" if row["errors"] else "UNRESOLVED"
            continue

        pos = legs[-1][2] if legs else None       # the tour closes: the previous drone's last place
        total = 0.0
        for s, pick, place in legs:
            qty = s.qty or 1
            first = cycle(pos, pick, place, g.axes, d)
            repeat = cycle(place, pick, place, g.axes, d) if qty > 1 else None
            step_s = first["total"] + (qty - 1) * (repeat["total"] if repeat else 0.0)
            row["steps"].append({"id": s.id, "station": s.station, "from": s.from_,
                                 "tool": s.tool, "qty": qty, "pick": list(pick),
                                 "place": list(place), "first_cycle": first,
                                 "repeat_cycle": repeat, "time_s": round(step_s, 3)})
            total += step_s
            pos = place
        row["total_s"] = round(total, 3)
        row["load_pct"] = round(100.0 * total / takt, 1)
        row["status"] = "PASS" if total <= takt else "FAIL"
        if total > takt:
            row["errors"].append(f"{total:.2f} s of moves per drone > takt {takt:g} s "
                                 f"({row['load_pct']}%)")
            doc["errors"] += [f"{gid}: {row['errors'][-1]}"]

    doc["complete"] = not doc["unresolved"]
    doc["passed"] = not doc["errors"]
    return doc


def print_table(doc: dict) -> None:
    print(f"{'gantry':<10}{'step':<20}{'from':<14}{'qty':>4}{'first_s':>9}{'cycle_s':>9}"
          f"{'step_s':>9}")
    print("-" * 75)
    for gid, g in doc["gantries"].items():
        for s in g["steps"]:
            rep = f"{s['repeat_cycle']['total']:.3f}" if s["repeat_cycle"] else "-"
            print(f"{gid:<10}{s['id']:<20}{s['from'] or '-':<14}{s['qty']:>4}"
                  f"{s['first_cycle']['total']:>9.3f}{rep:>9}{s['time_s']:>9.3f}")
        if g["total_s"] is not None:
            print(f"{gid:<10}{'total':<20}{'':<14}{'':>4}{'':>9}{'':>9}{g['total_s']:>9.3f}"
                  f"   {g['load_pct']:5.1f}% of takt {doc['takt_s']:g} s  {g['status']}")
        else:
            print(f"{gid:<10}{'total':<20}{'':<14}{'':>4}{'':>9}{'':>9}{'-':>9}   {g['status']}")
        print("-" * 75)
    for e in doc["errors"]:
        print(f"  [FAIL] {e}")
    if doc["unresolved"]:
        print(f"  ({len(doc['unresolved'])} UNRESOLVED input(s); see the report)")
    print("move_estimate:", "PASS" if doc["passed"] else "FAIL",
          "" if doc["complete"] else "(incomplete: unresolved inputs)")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--report", type=Path, default=None,
                    help="default: reports/move_estimate.json (move_estimate.dev.json with --dev)")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    report = args.report or report_path("move_estimate", args.dev)
    try:
        merged = load_merged(args.spec, args.dev)
        spec = parse_spec(merged.raw)
    except SpecError as exc:
        doc = {"passed": False, "dev": args.dev, "errors": exc.errors}
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(doc, indent=2) + "\n")
        print(f"move_estimate: FAIL — invalid spec: {exc}", file=sys.stderr)
        return 1
    doc = {"dev": args.dev, **estimate(spec),
           "placeholders": [p["path"] for p in merged.placeholders]}
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(doc, indent=2) + "\n")
    if args.dev:
        print(f"DEV MODE: {len(merged.placeholders)} placeholder value(s) from the overlay; "
              "results are not deliverable")
    print_table(doc)
    print(f"-> {report}")
    return 0 if doc["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
