"""Static move-time estimate: each gantry's pick-and-places per drone, against the takt budget.

    tools/.venv/bin/python tools/estimate_moves.py [--spec PATH] [--dev] [--cad-reports DIR]

No ROS, no Gazebo: arithmetic on the spec. One pick-and-place is

    XY to the pick at safe_z, Z down, grip, Z up, XY to the place, Z down, release, Z up

with `gantry_defaults.settle_s` after each of the four Z moves. Each axis follows a trapezoidal
profile from its own limits, scaled by `gantry_defaults.plan_speed_scale` (D-007: moves are
planned below v_max/a_max, which are ceilings):

    t = d/v + v/a     if d >= v^2/a      (reaches v)
    t = 2 sqrt(d/a)   otherwise          (triangular)

and an XY move takes the slower of X and Y. Every time is also given at scale 1.0 for comparison.

Which steps: the active steps with capability place, load or pick at a station in the gantry's
zone. A drive step is the driver's, never the gantry's (D-009). Distances come from the step's
`from` pose and its station pose; Z runs between the gantry's own safe_z
(tools/gantry_geometry.py: tallest item under it + tallest part it carries + z_margin) and the
pose's z. The gantry works its steps as a tour, once per drone: each cycle starts where the
previous one placed, the first at the last step's station (the previous drone); a `qty: n` step
is n cycles, cycles 2..n starting at its own station.

Budget: the pallet is only at a station between transfers, so each gantry's load is compared
with takt_s - transfer_s, transfer_s = index_s + layout.stop_unit_cycle_s, where index_s =
conveyor.pitch / layout.conveyor.belt_speed (spec.index_s; the spec states no index time, D-010).

Writes reports/move_estimate.json (`--dev`: move_estimate.dev.json) with per-step phase times,
per-gantry load at both scales, the budget, and `passed`. A null the estimate needs is
UNRESOLVED, not a failure (the check_line.py convention). Exits non-zero on a failure.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gantry_geometry as gg  # noqa: E402
from spec import (DEFAULT_SPEC, LineSpec, SpecError, add_dev_argument, gantry_ids,  # noqa: E402
                  index_s, load_merged, parse_spec, report_path, zone_owner)

GANTRY_CAPABILITIES = ("place", "load", "pick")


def axis_time(d: float, v: float, a: float) -> float:
    """Rest-to-rest time for distance `d` on one axis with a trapezoidal velocity profile."""
    d = abs(d)
    if d == 0.0:
        return 0.0
    return d / v + v / a if d >= v * v / a else 2.0 * math.sqrt(d / a)


def _t(axis, d: float, scale: float) -> float:
    return axis_time(d, axis.v_max * scale, axis.a_max * scale)


def xy_time(frm, to, axes, scale: float = 1.0) -> float:
    return max(_t(axes.x, to[0] - frm[0], scale), _t(axes.y, to[1] - frm[1], scale))


def cycle(frm, pick, place, axes, d, safe_z: float, scale: float = 1.0) -> dict:
    """One pick-and-place, split into its phases (seconds). `frm` is where the TCP starts."""
    down_pick = _t(axes.z, safe_z - pick[2], scale)
    down_place = _t(axes.z, safe_z - place[2], scale)
    out = {
        "xy_to_pick": xy_time(frm, pick, axes, scale),
        "z_down_pick": down_pick + d.settle_s,
        "grip": d.grip_s,
        "z_up_pick": down_pick + d.settle_s,
        "xy_to_place": xy_time(pick, place, axes, scale),
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


def transfer(spec: LineSpec) -> tuple[float | None, list[str]]:
    """Seconds per drone the pallet spends moving between stations: index_s + stop unit."""
    lay = spec.layout
    missing = _nulls("conveyor", {"pitch": spec.conveyor.pitch})
    if lay is None:
        return None, missing + ["layout section is absent (belt_speed, stop_unit_cycle_s)"]
    missing += _nulls("layout", {"conveyor.belt_speed": lay.conveyor.belt_speed,
                                 "stop_unit_cycle_s": lay.stop_unit_cycle_s})
    if missing:
        return None, missing
    return index_s(spec) + lay.stop_unit_cycle_s, []


def estimate(spec: LineSpec, cad: gg.CadHeights | None = None) -> dict:
    """Per-gantry move time per drone. Never raises on a null: it lands in `unresolved`."""
    cad = cad or gg.CadHeights()
    takt = spec.line.takt_s
    d = spec.gantry_defaults
    scale = d.plan_speed_scale
    owner = zone_owner(spec)
    transfer_s, transfer_missing = transfer(spec)
    budget = None if transfer_s is None else takt - transfer_s
    idx = index_s(spec)
    doc: dict = {"takt_s": takt, "index_s": idx and round(idx, 3),
                 "transfer_s": transfer_s and round(transfer_s, 3),
                 "budget_s": budget and round(budget, 3), "plan_speed_scale": scale,
                 "passed": True, "complete": True, "gantries": {}, "unresolved": [], "errors": [],
                 "model": {"grip_s": d.grip_s, "release_s": d.release_s,
                           "settle_s": d.settle_s, "z_margin": d.z_margin,
                           "capabilities": list(GANTRY_CAPABILITIES),
                           "profile": "trapezoidal per axis at plan_speed_scale x v_max/a_max; "
                                      "XY = max(X, Y)"}}
    common_missing = _nulls("gantry_defaults", {
        "grip_s": d.grip_s, "release_s": d.release_s, "settle_s": d.settle_s,
        "plan_speed_scale": scale}) + transfer_missing

    for gid in gantry_ids({"gantries": spec.gantries}):
        g = spec.gantries[gid]
        sz = gg.safe_z(spec, gid, cad)
        row: dict = {"zone": g.zone, "safe_z": sz.doc(), "budget_s": doc["budget_s"],
                     "steps": [], "total_s": None, "total_s_at_1": None, "load_pct": None,
                     "status": "UNRESOLVED",
                     "unresolved": list(common_missing) + sz.unresolved, "errors": []}
        doc["gantries"][gid] = row
        if g.zone is None:
            row["unresolved"].append(f"gantries.{gid}.zone is null")
        for ax in "xyz":
            a = getattr(g.axes, ax)
            row["unresolved"] += _nulls(f"gantries.{gid}.axes.{ax}",
                                        {"v_max": a.v_max, "a_max": a.a_max})
        if sz.value is not None and g.mount[2] is not None \
                and None not in (g.axes.z.min, g.axes.z.max):
            rel = sz.value - g.mount[2]
            if not g.axes.z.min - gg.EPS <= rel <= g.axes.z.max + gg.EPS:
                row["errors"].append(f"safe_z={sz.value:.3f} is z={rel:.3f} in {gid}'s frame, "
                                     f"outside its travel [{g.axes.z.min:g}, {g.axes.z.max:g}]")

        steps = [s for s in spec.sequence if s.status == "active"
                 and s.capability in GANTRY_CAPABILITIES and owner.get(s.station) == gid]
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
            legs.append((s, pick[:3], place[:3]))

        if row["unresolved"] or row["errors"]:
            doc["unresolved"] += [f"{gid}: {u}" for u in row["unresolved"]]
            doc["errors"] += [f"{gid}: {e}" for e in row["errors"]]
            row["status"] = "FAIL" if row["errors"] else "UNRESOLVED"
            continue

        totals = {}
        for key, sc in (("plan", scale), ("at_1", 1.0)):
            pos = legs[-1][2] if legs else None      # the tour closes: previous drone's last place
            total = 0.0
            for i, (s, pick, place) in enumerate(legs):
                qty = s.qty or 1
                first = cycle(pos, pick, place, g.axes, d, sz.value, sc)
                repeat = cycle(place, pick, place, g.axes, d, sz.value, sc) if qty > 1 else None
                step_s = first["total"] + (qty - 1) * (repeat["total"] if repeat else 0.0)
                if key == "plan":
                    row["steps"].append({"id": s.id, "station": s.station, "from": s.from_,
                                         "tool": s.tool, "qty": qty, "pick": list(pick),
                                         "place": list(place), "first_cycle": first,
                                         "repeat_cycle": repeat, "time_s": round(step_s, 3)})
                else:
                    row["steps"][i]["time_s_at_1"] = round(step_s, 3)
                total += step_s
                pos = place
            totals[key] = total
        row["total_s"] = round(totals["plan"], 3)
        row["total_s_at_1"] = round(totals["at_1"], 3)
        row["load_pct"] = round(100.0 * totals["plan"] / budget, 1)
        row["load_pct_at_1"] = round(100.0 * totals["at_1"] / budget, 1)
        row["status"] = "PASS" if totals["plan"] <= budget else "FAIL"
        if row["status"] == "FAIL":
            row["errors"].append(f"{totals['plan']:.2f} s of moves per drone at scale {scale:g} "
                                 f"> budget {budget:.2f} s (takt {takt:g} - transfer "
                                 f"{transfer_s:.2f})")
            doc["errors"].append(f"{gid}: {row['errors'][-1]}")

    doc["complete"] = not doc["unresolved"]
    doc["passed"] = not doc["errors"]
    return doc


def print_table(doc: dict) -> None:
    scale = doc["plan_speed_scale"]
    head = f"@{scale:g}" if scale is not None else "@plan"
    print(f"{'gantry':<10}{'step':<19}{'from':<14}{'qty':>4}{'safe_z':>8}"
          f"{'step_s ' + head:>14}{'step_s @1':>11}")
    print("-" * 80)
    for gid, g in doc["gantries"].items():
        sz = g["safe_z"]["safe_z"]
        for s in g["steps"]:
            print(f"{gid:<10}{s['id']:<19}{s['from'] or '-':<14}{s['qty']:>4}"
                  f"{sz:>8.3f}{s['time_s']:>14.3f}{s['time_s_at_1']:>11.3f}")
        if g["total_s"] is not None:
            print(f"{gid:<10}{'total':<19}{'':<14}{'':>4}{'':>8}{g['total_s']:>14.3f}"
                  f"{g['total_s_at_1']:>11.3f}")
            print(f"{'':<10}{'budget':<19}{'':<14}{'':>4}{'':>8}{g['budget_s']:>14.3f}"
                  f"{g['budget_s']:>11.3f}   {g['load_pct']:.1f}% / {g['load_pct_at_1']:.1f}%"
                  f"  {g['status']}")
        else:
            print(f"{gid:<10}{'total':<19}{'-':>54}   {g['status']}")
        print("-" * 80)
    if doc["transfer_s"] is not None:
        print(f"budget = takt {doc['takt_s']:g} s - transfer {doc['transfer_s']:g} s "
              f"(index {doc['index_s']:g} s = pitch / belt_speed, + stop unit) "
              f"= {doc['budget_s']:g} s per gantry")
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
    ap.add_argument("--cad-reports", type=Path, default=None,
                    help=f"where cad_<part>.json live (default {gg.CAD_REPORTS})")
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
    doc = {"dev": args.dev, **estimate(spec, gg.CadHeights(args.cad_reports, args.dev)),
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
