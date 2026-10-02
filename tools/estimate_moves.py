"""Static move-time estimate: each SCARA's pick-and-places per drone, against the takt budget.

    tools/.venv/bin/python tools/estimate_moves.py [--spec PATH] [--dev] [--cad-reports DIR]

No ROS, no Gazebo: arithmetic on the spec. One SCARA pick-and-place (D-012) is

    home -> pick (horizontal, straight line), Z down, grip, Z up,
    pick -> place (horizontal), Z down, release, Z up, place -> home

with `scara_defaults.settle_s` after each of the four Z moves. Every move is a trapezoid on the
TCP distance (straight line in plan for a horizontal move, the drop for a Z move) at the SCARA's
`motion.v_max`/`a_max` scaled by `scara_defaults.plan_speed_scale` (D-007: plan below the
ceilings; its 0.5 comes from the gantry spike and must be re-derived for a SCARA):

    t = d/v + v/a     if d >= v^2/a      (reaches v)
    t = 2 sqrt(d/a)   otherwise          (triangular)

Every time is also given at scale 1.0 for comparison.

Home is the TCP folded in to `reach.r_min`, straight out from the base towards its station, at
safe_z: the arm parked as tight as its reach allows, clear of the pallet. Z runs between the
SCARA's own safe_z (tools/robot_geometry.py: tallest item in its reach envelope + the part it
carries + z_margin) and the pose's z.

Which steps: the active steps with capability place, load or pick at the SCARA's station. A drive
step is the gang head's, never the SCARA's (D-009). A `qty: n` step is n whole cycles.

Budget: the pallet is only at a station between transfers, so each SCARA's load is compared with
takt_s - transfer_s, transfer_s = index_s + layout.stop_unit_cycle_s, where index_s =
conveyor.pitch / layout.conveyor.belt_speed (spec.index_s; the spec states no index time, D-010).

Station time (D-011, D-012): SCARA steps + gang head drive_s x passes + slide t_in + t_out
(a head on a slide: SCARA places, clears, slide in, drive, slide out) + index_s, against takt.

Writes reports/move_estimate.json (`--dev`: move_estimate.dev.json) with per-step phase times,
per-SCARA load at both scales, the budget, and `passed`. A null the estimate needs is
UNRESOLVED, not a failure (the check_line.py convention). Exits non-zero on a failure.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import robot_geometry as rg  # noqa: E402
from spec import (DEFAULT_SPEC, LineSpec, SpecError, add_dev_argument, index_s,  # noqa: E402
                  load_merged, parse_spec, report_path, robot_served, scara_base)

ROBOT_CAPABILITIES = ("place", "load", "pick")


def axis_time(d: float, v: float, a: float) -> float:
    """Rest-to-rest time for distance `d` with a trapezoidal velocity profile."""
    d = abs(d)
    if d == 0.0:
        return 0.0
    return d / v + v / a if d >= v * v / a else 2.0 * math.sqrt(d / a)


def plan_dist(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def cycle(home, pick, place, motion, d, safe_z: float, scale: float = 1.0) -> dict:
    """One pick-and-place from home and back, split into its phases (seconds)."""
    v, a = motion.v_max * scale, motion.a_max * scale

    def t(dist):
        return axis_time(dist, v, a)

    down_pick = t(safe_z - pick[2])
    down_place = t(safe_z - place[2])
    out = {
        "home_to_pick": t(plan_dist(home, pick)),
        "z_down_pick": down_pick + d.settle_s,
        "grip": d.grip_s,
        "z_up_pick": down_pick + d.settle_s,
        "pick_to_place": t(plan_dist(pick, place)),
        "z_down_place": down_place + d.settle_s,
        "release": d.release_s,
        "z_up_place": down_place + d.settle_s,
        "place_to_home": t(plan_dist(place, home)),
    }
    out = {k: round(val, 4) for k, val in out.items()}
    out["total"] = round(sum(out.values()), 4)
    return out


def home_xy(base, station_pose, r_min: float) -> tuple[float, float]:
    """The arm folded in to r_min, pointing from the base at its station."""
    dx, dy = station_pose[0] - base[0], station_pose[1] - base[1]
    n = math.hypot(dx, dy) or 1.0
    return base[0] + dx / n * r_min, base[1] + dy / n * r_min


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


def scara_steps(spec: LineSpec, sid: str) -> list:
    st = spec.scaras[sid].station
    return [s for s in spec.sequence if s.status == "active" and s.station == st
            and s.capability in ROBOT_CAPABILITIES and robot_served(s)]


def estimate(spec: LineSpec, cad: rg.CadHeights | None = None) -> dict:
    """Per-SCARA move time per drone. Never raises on a null: it lands in `unresolved`."""
    cad = cad or rg.CadHeights()
    takt = spec.line.takt_s
    d = spec.scara_defaults
    scale = d.plan_speed_scale
    transfer_s, transfer_missing = transfer(spec)
    budget = None if transfer_s is None else takt - transfer_s
    idx = index_s(spec)
    doc: dict = {"takt_s": takt, "index_s": idx and round(idx, 3),
                 "transfer_s": transfer_s and round(transfer_s, 3),
                 "budget_s": budget and round(budget, 3), "plan_speed_scale": scale,
                 "passed": True, "complete": True, "scaras": {}, "unresolved": [], "errors": [],
                 "model": {"grip_s": d.grip_s, "release_s": d.release_s,
                           "settle_s": d.settle_s, "z_margin": d.z_margin,
                           "capabilities": list(ROBOT_CAPABILITIES),
                           "home": "TCP at reach.r_min from the base, towards its station",
                           "profile": "trapezoid on the TCP distance (plan straight line, or "
                                      "the Z drop) at plan_speed_scale x motion.v_max/a_max"}}
    common_missing = _nulls("scara_defaults", {
        "grip_s": d.grip_s, "release_s": d.release_s, "settle_s": d.settle_s,
        "plan_speed_scale": scale}) + transfer_missing

    for sid in sorted(spec.scaras, key=lambda k: (spec.stations[spec.scaras[k].station].index
                                                   if spec.scaras[k].station in spec.stations
                                                   else 0, k)):
        sc = spec.scaras[sid]
        sz = rg.safe_z(spec, sid, cad)
        base = scara_base(spec, sid)
        row: dict = {"station": sc.station, "base": base and [round(c, 4) for c in base],
                     "safe_z": sz.doc(), "budget_s": doc["budget_s"], "steps": [],
                     "total_s": None, "total_s_at_1": None, "load_pct": None,
                     "status": "UNRESOLVED",
                     "unresolved": list(common_missing) + sz.unresolved, "errors": []}
        doc["scaras"][sid] = row
        row["unresolved"] += _nulls(f"scaras.{sid}", {
            "motion.v_max": sc.motion.v_max, "motion.a_max": sc.motion.a_max,
            "reach.r_min": sc.reach.r_min, "z_stroke": sc.z_stroke})
        if base is None:
            row["unresolved"].append(f"scaras.{sid}: base_side/base_offset_y or the station "
                                     "pose is null")
        if sz.value is not None and sc.z_stroke is not None \
                and sz.value > sc.z_stroke + rg.EPS:
            row["errors"].append(f"safe_z={sz.value:.3f} is above its z_stroke "
                                 f"{sc.z_stroke:g}")

        legs = []
        place = spec.stations[sc.station].pose
        for s in scara_steps(spec, sid):
            pick = rg.source_pose(spec, s.from_)
            if pick is None:
                row["unresolved"].append(f"sequence.{s.id}: no pick pose (`from` names no "
                                         "feeder or buffer with a pose)")
                continue
            if None in pick[:3] or None in place[:3]:
                row["unresolved"].append(f"sequence.{s.id}: pick or place pose has a null")
                continue
            legs.append((s, pick[:3], place[:3]))

        doc["errors"] += [f"{sid}: {e}" for e in row["errors"]]
        if row["unresolved"]:
            doc["unresolved"] += [f"{sid}: {u}" for u in row["unresolved"]]
            row["status"] = "FAIL" if row["errors"] else "UNRESOLVED"
            continue
        # A safe_z above z_stroke is already a failure; the times are still worked out at that
        # safe_z, so the breakdown is there to read.

        home = home_xy(base, place, sc.reach.r_min)
        row["home"] = [round(c, 4) for c in home]
        totals = {}
        for key, k_scale in (("plan", scale), ("at_1", 1.0)):
            total = 0.0
            for i, (s, pick, plc) in enumerate(legs):
                qty = s.qty or 1
                one = cycle(home, pick, plc, sc.motion, d, sz.value, k_scale)
                step_s = qty * one["total"]
                if key == "plan":
                    row["steps"].append({
                        "id": s.id, "station": s.station, "from": s.from_, "tool": s.tool,
                        "qty": qty, "pick": list(pick), "place": list(plc),
                        "dist_m": {"home_to_pick": round(plan_dist(home, pick), 4),
                                   "pick_to_place": round(plan_dist(pick, plc), 4),
                                   "place_to_home": round(plan_dist(plc, home), 4)},
                        "cycle": one, "time_s": round(step_s, 3)})
                else:
                    row["steps"][i]["time_s_at_1"] = round(step_s, 3)
                total += step_s
            totals[key] = total
        row["total_s"] = round(totals["plan"], 3)
        row["total_s_at_1"] = round(totals["at_1"], 3)
        row["load_pct"] = round(100.0 * totals["plan"] / budget, 1)
        row["load_pct_at_1"] = round(100.0 * totals["at_1"] / budget, 1)
        over = totals["plan"] > budget
        row["status"] = "FAIL" if over or row["errors"] else "PASS"
        if over:
            row["errors"].append(f"{totals['plan']:.2f} s of moves per drone at scale {scale:g} "
                                 f"> budget {budget:.2f} s (takt {takt:g} - transfer "
                                 f"{transfer_s:.2f})")
            doc["errors"].append(f"{sid}: {row['errors'][-1]}")

    doc["complete"] = not doc["unresolved"]
    doc["passed"] = not doc["errors"]
    return doc


def station_times(spec: LineSpec, doc: dict) -> dict:
    """Per station with a SCARA or drive step (D-011, D-012):

        station_time = SCARA step times + drive time + slide time + index_s

    SCARA step times are the estimate's per-step times at plan_speed_scale (home and back); drive
    time = gang head `drive_s` x passes, passes = ceil(qty / spindles); slide time = `slide.t_in +
    t_out` of a head on a slide, once per drive step (place, SCARA clears, slide in, drive, slide
    out). A null input is UNRESOLVED; status FAIL when station_time > takt_s."""
    takt = spec.line.takt_s
    idx = index_s(spec)
    step_s = {s["id"]: s["time_s"] for g in doc["scaras"].values() for s in g["steps"]}
    rows: dict = {}
    for sid, st in sorted(spec.stations.items(), key=lambda kv: kv[1].index):
        steps = [s for s in spec.sequence if s.status == "active" and s.station == sid
                 and (s.capability in ROBOT_CAPABILITIES and robot_served(s)
                      or s.capability == "drive")]
        if not steps:
            continue
        row = {"scara_s": 0.0, "drive_s": 0.0, "slide_s": 0.0, "index_s": idx,
               "total_s": None, "takt_s": takt, "status": "UNRESOLVED", "steps": [],
               "unresolved": []}
        if idx is None:
            row["unresolved"].append("index time: conveyor.pitch or layout.conveyor.belt_speed "
                                     "is null")
        for s in steps:
            if s.capability != "drive":
                t = step_s.get(s.id)
                if t is None:
                    row["unresolved"].append(f"{s.id}: no move estimate")
                else:
                    row["scara_s"] += t
                    row["steps"].append({"id": s.id, "kind": "scara", "time_s": t})
                continue
            head = spec.gang_heads.get(s.head)
            if head is None:
                row["unresolved"].append(f"{s.id}: no gang head, so no drive_s")
                continue
            missing = [k for k, v in (("drive_s", head.drive_s), ("spindles", head.spindles))
                       if v is None] + (["qty"] if s.qty is None else [])
            if head.slide is not None:
                missing += [f"slide.{k}" for k in ("t_in", "t_out")
                            if getattr(head.slide, k) is None]
            if missing:
                row["unresolved"].append(f"{s.id}: {', '.join(missing)} null "
                                         f"(gang_heads.{s.head} / step)")
                continue
            passes = math.ceil(s.qty / head.spindles)
            t = head.drive_s * passes
            row["drive_s"] += t
            entry = {"id": s.id, "kind": "drive", "head": s.head, "qty": s.qty,
                     "spindles": head.spindles, "passes": passes,
                     "drive_s_per_pass": head.drive_s, "time_s": round(t, 3)}
            if head.slide is not None:
                slide = head.slide.t_in + head.slide.t_out
                row["slide_s"] += slide
                entry["slide_s"] = round(slide, 3)
            row["steps"].append(entry)
        for k in ("scara_s", "drive_s", "slide_s"):
            row[k] = round(row[k], 3)
        if not row["unresolved"]:
            total = row["scara_s"] + row["drive_s"] + row["slide_s"] + idx
            row["total_s"] = round(total, 3)
            row["status"] = "PASS" if total <= takt else "FAIL"
        rows[sid] = row
    return rows


def print_station_table(rows: dict) -> None:
    print(f"{'station':<24}{'scara_s':>9}{'drive_s':>9}{'slide_s':>9}{'index_s':>9}"
          f"{'total_s':>9}{'takt_s':>8}  status")
    for sid, r in rows.items():
        def f(v):
            return f"{v:>9.3f}" if v is not None else f"{'-':>9}"
        print(f"{sid:<24}{f(r['scara_s'])}{f(r['drive_s'])}{f(r['slide_s'])}{f(r['index_s'])}"
              f"{f(r['total_s'])}{r['takt_s']:>8g}  {r['status']}")


def print_table(doc: dict) -> None:
    scale = doc["plan_speed_scale"]
    head = f"@{scale:g}" if scale is not None else "@plan"
    print(f"{'scara':<25}{'step':<19}{'from':<14}{'qty':>4}{'safe_z':>8}"
          f"{'step_s ' + head:>14}{'step_s @1':>11}")
    print("-" * 95)
    for sid, g in doc["scaras"].items():
        sz = g["safe_z"]["safe_z"]
        for s in g["steps"]:
            print(f"{sid:<25}{s['id']:<19}{s['from'] or '-':<14}{s['qty']:>4}"
                  f"{sz:>8.3f}{s['time_s']:>14.3f}{s['time_s_at_1']:>11.3f}")
        if g["total_s"] is not None:
            print(f"{'':<25}{'budget':<19}{'':<14}{'':>4}{'':>8}{g['budget_s']:>14.3f}"
                  f"{g['budget_s']:>11.3f}   {g['load_pct']:.1f}% / {g['load_pct_at_1']:.1f}%"
                  f"  {g['status']}" + (f" ({'; '.join(g['errors'])})" if g["errors"] else ""))
        else:
            print(f"{sid:<25}{'total':<19}{'-':>51}   {g['status']}")
        print("-" * 95)
    if doc["transfer_s"] is not None:
        print(f"budget = takt {doc['takt_s']:g} s - transfer {doc['transfer_s']:g} s "
              f"(index {doc['index_s']:g} s = pitch / belt_speed, + stop unit) "
              f"= {doc['budget_s']:g} s per SCARA")
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
                    help=f"where cad_<part>.json live (default {rg.CAD_REPORTS})")
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
    est = estimate(spec, rg.CadHeights(args.cad_reports, args.dev))
    rows = station_times(spec, est)
    doc = {"dev": args.dev, **est, "station_times": rows,
           "placeholders": [p["path"] for p in merged.placeholders]}
    doc["passed"] = est["passed"] and all(r["status"] != "FAIL" for r in rows.values())
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(doc, indent=2) + "\n")
    if args.dev:
        print(f"DEV MODE: {len(merged.placeholders)} placeholder value(s) from the overlay; "
              "results are not deliverable")
    print_table(doc)
    print("station_time = SCARA steps + drive_s x passes + slide t_in/t_out + index "
          "(pitch / belt_speed):")
    print_station_table(rows)
    print(f"-> {report}")
    return 0 if doc["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
