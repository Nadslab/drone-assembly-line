"""Static move-time estimate: each SCARA's pick-and-places per drone, against the takt budget.

    tools/.venv/bin/python tools/estimate_moves.py [--spec PATH] [--dev] [--cad-reports DIR]
    tools/.venv/bin/python tools/estimate_moves.py --dev --calibrate   # v_max/a_max from the
                                                                      # standard cycle

No ROS, no Gazebo: arithmetic on the spec. One SCARA pick-and-place (D-012) is

    start -> pick (horizontal, straight line), Z down, grip, Z up,
    pick -> place (horizontal), Z down, release, Z up, place -> end

with `scara_defaults.settle_s` after each of the four Z moves. Every move is a trapezoid on the
TCP distance (straight line in plan for a horizontal move, the drop for a Z move) at the robot
model's `motion.v_max`/`a_max` x `scara_defaults.cycle_derate`:

    t = d/v + v/a     if d >= v^2/a      (reaches v)
    t = 2 sqrt(d/a)   otherwise          (triangular)

Every time is also given at derate 1.0 (the vendor-optimised speed) for comparison.

Calibration (D-014): `motion.v_max`/`a_max` are derived from the model's `standard_cycle` — the
same trapezoids over its round trip (up vertical_m, across horizontal_m, down; and back), grip
and settle excluded, must take its time_s. `--calibrate` keeps the current a_max / v_max ratio
and solves the one scale factor; every run reports the standard-cycle time at the stored values
and fails when they no longer reproduce it. A step whose payload (every tool on the
end-effector + the part and what rides in it) exceeds standard_cycle.payload_kg is a WARN: its
times are optimistic there.

Start and end are home — the TCP folded in to the model's r_min, straight out from the base
towards its station — except around a gang head's drive (D-014): the SCARA step before it ends at
`park_pose` and the next one starts from there (robot_geometry.park_steps). Z runs between the
step's own safe_z (tools/robot_geometry.py, path-based, D-013) and the pose's z.

Dual-tool switch (D-013): a step whose tool differs from the SCARA's previous step's (read
cyclically) starts with a 180 deg J4 turn, timed pi / (motion.j4_speed x cycle_derate), in series.

Which steps: the active steps a SCARA performs at its station (a tool, or a part drawn `from` a
source). A drive step is the gang head's, never the SCARA's (D-009). A `qty: n` step is n whole
cycles.

Index and budget (D-014): a station's index is the distance the pallet travels to enter it (from
the last queue slot, or the previous station) / belt_speed + layout.stop_unit_cycle_s
(spec.station_index). Each SCARA's load is compared with takt_s - its station's index.

Station time (D-011..D-014): SCARA steps (tool switches and the park moves included) + gang head
drive_s x passes + slide t_in + t_out + `actuate` step durations + the station's index, against
takt. The old fixed index (pitch / belt_speed) is reported next to it.

Writes reports/move_estimate.json (`--dev`: move_estimate.dev.json). A null the estimate needs is
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
                  load_merged, parse_spec, queue_gaps, report_path, robot_served, scara_base,
                  scara_model, station_index)

STANDARD_CYCLE_TOL_S = 0.001    # stored v_max/a_max must reproduce standard_cycle.time_s this close


def axis_time(d: float, v: float, a: float) -> float:
    """Rest-to-rest time for distance `d` with a trapezoidal velocity profile."""
    d = abs(d)
    if d == 0.0:
        return 0.0
    return d / v + v / a if d >= v * v / a else 2.0 * math.sqrt(d / a)


def plan_dist(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


# ---------------------------------------------------------------- calibration (D-014)

def standard_cycle_time(sc, v: float, a: float) -> float:
    """The move model over a vendor standard cycle (round_trip_arch): up vertical_m, across
    horizontal_m, down vertical_m, and back the same way. No grip, no settle."""
    one_way = 2.0 * axis_time(sc.vertical_m, v, a) + axis_time(sc.horizontal_m, v, a)
    return 2.0 * one_way


def calibrate(sc, v0: float, a0: float) -> dict:
    """The one factor k with standard_cycle_time(k v0, k a0) == sc.time_s (keeps a0 / v0), by
    bisection: the time falls monotonically as k grows."""
    lo, hi = 1e-6, 1e6
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if standard_cycle_time(sc, mid * v0, mid * a0) > sc.time_s:
            lo = mid
        else:
            hi = mid
    k = math.sqrt(lo * hi)
    v, a = k * v0, k * a0
    return {"scale": round(k, 6), "v_max": round(v, 4), "a_max": round(a, 4),
            "ratio_a_over_v": round(a0 / v0, 6),
            "check_s": round(standard_cycle_time(sc, round(v, 4), round(a, 4)), 5),
            "reaches_v_max": {"vertical": sc.vertical_m >= v * v / a,
                              "horizontal": sc.horizontal_m >= v * v / a}}


def calibration(spec: LineSpec) -> dict:
    """Per robot model with a standard_cycle: the standard-cycle time at the stored motion, and
    whether it reproduces time_s."""
    out = {}
    for mid, m in spec.robot_models.items():
        sc = m.standard_cycle
        if sc is None:
            continue
        row = {"standard_cycle": sc.model_dump(), "v_max": m.motion.v_max,
               "a_max": m.motion.a_max, "time_s": None, "ok": None}
        if m.motion.v_max is not None and m.motion.a_max is not None:
            t = standard_cycle_time(sc, m.motion.v_max, m.motion.a_max)
            row["time_s"] = round(t, 5)
            row["ok"] = abs(t - sc.time_s) <= STANDARD_CYCLE_TOL_S
        out[mid] = row
    return out


# ---------------------------------------------------------------- one cycle

def cycle(home, pick, place, motion, d, safe_z: float, derate: float = 1.0, end=None,
          start_label: str = "home", end_label: str = "home") -> dict:
    """One pick-and-place from `home` (or the start pose) and on to `end` (default: back to
    home), split into its phases (seconds)."""
    v, a = motion.v_max * derate, motion.a_max * derate
    end = home if end is None else end

    def t(dist):
        return axis_time(dist, v, a)

    down_pick = t(safe_z - pick[2])
    down_place = t(safe_z - place[2])
    out = {
        f"{start_label}_to_pick": t(plan_dist(home, pick)),
        "z_down_pick": down_pick + d.settle_s,
        "grip": d.grip_s,
        "z_up_pick": down_pick + d.settle_s,
        "pick_to_place": t(plan_dist(pick, place)),
        "z_down_place": down_place + d.settle_s,
        "release": d.release_s,
        "z_up_place": down_place + d.settle_s,
        f"place_to_{end_label}": t(plan_dist(place, end)),
    }
    out = {k: round(val, 4) for k, val in out.items()}
    out["total"] = round(sum(out.values()), 4)
    return out


def _nulls(label: str, values: dict) -> list[str]:
    return [f"{label}: {k} is null" for k, v in values.items() if v is None]


def scara_steps(spec: LineSpec, sid: str) -> list:
    return rg.scara_steps(spec, sid)


def switch_time(spec: LineSpec, sid: str, derate: float) -> float | None:
    """One dual-tool switch: 180 deg about J4 at j4_speed x derate."""
    w = scara_model(spec, sid).motion.j4_speed
    return None if w is None or derate is None else math.pi / (w * derate)


def estimate(spec: LineSpec, cad: rg.CadHeights | None = None) -> dict:
    """Per-SCARA move time per drone. Never raises on a null: it lands in `unresolved`."""
    cad = cad or rg.CadHeights()
    takt = spec.line.takt_s
    d = spec.scara_defaults
    derate = d.cycle_derate
    stations = station_index(spec)
    cal = calibration(spec)
    doc: dict = {"takt_s": takt, "cycle_derate": derate, "calibration": cal,
                 "index_s_old": (lambda i: i and round(i, 3))(index_s(spec)),
                 "station_index": stations, "queue_gaps": queue_gaps(spec),
                 "passed": True, "complete": True, "scaras": {}, "unresolved": [], "errors": [],
                 "warnings": [],
                 "model": {"grip_s": d.grip_s, "release_s": d.release_s,
                           "settle_s": d.settle_s, "z_margin": d.z_margin,
                           "steps": "every active step a SCARA performs (tool or `from`)",
                           "home": "TCP at the model's r_min from the base, towards its station",
                           "park": "the step before a drive ends at park_pose, the next starts "
                                   "there (D-014)",
                           "tool_switch": "pi / (j4_speed x cycle_derate), in series",
                           "profile": "trapezoid on the TCP distance (plan straight line, or "
                                      "the Z drop) at cycle_derate x motion.v_max/a_max, "
                                      "calibrated to standard_cycle (D-014)"}}
    for mid, row in cal.items():
        if row["ok"] is False:
            doc["errors"].append(f"robot_models.{mid}.motion: v_max {row['v_max']:g} / a_max "
                                 f"{row['a_max']:g} give the standard cycle in "
                                 f"{row['time_s']:.4f} s, not {row['standard_cycle']['time_s']:g}"
                                 " s (re-run estimate_moves.py --calibrate)")
    common_missing = _nulls("scara_defaults", {
        "grip_s": d.grip_s, "release_s": d.release_s, "settle_s": d.settle_s,
        "cycle_derate": derate})

    for sid in sorted(spec.scaras, key=lambda k: (spec.stations[spec.scaras[k].station].index
                                                   if spec.scaras[k].station in spec.stations
                                                   else 0, k)):
        sc = spec.scaras[sid]
        model = scara_model(spec, sid)
        k = rg.kin(spec, sid)
        sz = rg.safe_z(spec, sid, cad)
        base = scara_base(spec, sid)
        switches = rg.tool_switches(spec, sid)
        ends, starts = rg.park_steps(spec, sid)
        st_idx = stations.get(sc.station, {})
        idx = st_idx.get("index_s")
        budget = None if idx is None else takt - idx
        row: dict = {"station": sc.station, "model": sc.model,
                     "base": base and [round(c, 4) for c in base],
                     "safe_z": sz.doc(), "index_s": idx, "budget_s": budget and round(budget, 3),
                     "steps": [], "tool_switches": switches, "switch_s": None,
                     "park_after": sorted(ends), "park_before": sorted(starts),
                     "park_pose": rg.park_joints(spec, sid),
                     "total_s": None, "total_s_at_1": None, "load_pct": None,
                     "status": "UNRESOLVED", "warnings": [],
                     "unresolved": list(common_missing) + sz.unresolved
                     + list(st_idx.get("unresolved", [])), "errors": []}
        doc["scaras"][sid] = row
        row["unresolved"] += _nulls(f"robot_models.{sc.model}", {
            "motion.v_max": model.motion.v_max, "motion.a_max": model.motion.a_max,
            "link1": model.link1, "link2": model.link2})
        if switches and model.motion.j4_speed is None:
            row["unresolved"].append(f"robot_models.{sc.model}: motion.j4_speed is null (times "
                                     f"the tool switch before {', '.join(switches)})")
        if (ends or starts) and row["park_pose"] is None:
            row["unresolved"].append(f"scaras.{sid}.park_pose is null (it parks while "
                                     f"{sc.station}'s gang head works)")
        if base is None:
            row["unresolved"].append(f"scaras.{sid}: offset or the station pose is null")
        if k is not None:
            for step, one in sz.steps.items():
                if one["safe_z"] is not None and one["safe_z"] > k.z_hi + rg.EPS:
                    row["errors"].append(f"{step}: safe_z={one['safe_z']:.3f} is above its z "
                                         f"range top {k.z_hi:g}")
        limit = model.standard_cycle.payload_kg if model.standard_cycle else None

        legs = []
        for s in scara_steps(spec, sid):
            pick, place = rg.step_points(spec, s)
            if pick is None or place is None:
                row["unresolved"].append(f"sequence.{s.id}: no pick or place pose (`from` names "
                                         "no feeder or buffer with a pose, or a pose is null)")
                continue
            legs.append((s, pick[:3], place[:3]))
            kg = rg.step_payload(spec, sid, s)
            if limit is not None and kg is not None and kg > limit + rg.EPS:
                row["warnings"].append(f"{s.id}: payload {kg:.3f} kg (tools + {s.part}) > the "
                                       f"standard cycle's {limit:g} kg: its times are optimistic")

        doc["errors"] += [f"{sid}: {e}" for e in row["errors"]]
        doc["warnings"] += [f"{sid}: {w}" for w in row["warnings"]]
        if row["unresolved"]:
            doc["unresolved"] += [f"{sid}: {u}" for u in row["unresolved"]]
            row["status"] = "FAIL" if row["errors"] else "UNRESOLVED"
            continue
        # A safe_z above the stroke is already a failure; the times are still worked out at that
        # safe_z, so the breakdown is there to read.

        home = rg.home(k, spec.stations[sc.station].pose[:2])
        park = rg.fk(k, *row["park_pose"]) if row["park_pose"] is not None else None
        row["home"] = [round(c, 4) for c in home]
        row["park_tcp"] = park and [round(c, 4) for c in park]
        totals = {}
        for key, k_derate in (("plan", derate), ("at_1", 1.0)):
            total = 0.0
            sw = switch_time(spec, sid, k_derate) if switches else 0.0
            for i, (s, pick, plc) in enumerate(legs):
                qty = s.qty or 1
                z = sz.steps[s.id]["safe_z"]
                first = park if s.id in starts else home
                last = park if s.id in ends else home
                one = cycle(first, pick, plc, model.motion, d, z, k_derate, end=last,
                            start_label="park" if s.id in starts else "home",
                            end_label="park" if s.id in ends else "home")
                if qty > 1:         # the repeats run home to home; only the ends touch park
                    rep = cycle(home, pick, plc, model.motion, d, z, k_derate)
                    step_t = one["total"] + (qty - 1) * rep["total"]
                else:
                    step_t = one["total"]
                switch = sw if s.id in switches else 0.0
                step_s = step_t + switch
                if key == "plan":
                    row["steps"].append({
                        "id": s.id, "station": s.station, "from": s.from_, "tool": s.tool,
                        "qty": qty, "pick": list(pick), "place": list(plc), "safe_z": z,
                        "start": "park" if s.id in starts else "home",
                        "end": "park" if s.id in ends else "home",
                        "payload_kg": rg.step_payload(spec, sid, s),
                        "dist_m": {"start_to_pick": round(plan_dist(first, pick), 4),
                                   "pick_to_place": round(plan_dist(pick, plc), 4),
                                   "place_to_end": round(plan_dist(plc, last), 4)},
                        "cycle": one, "tool_switch_s": round(switch, 4),
                        "time_s": round(step_s, 3)})
                else:
                    row["steps"][i]["time_s_at_1"] = round(step_s, 3)
                total += step_s
            totals[key] = total
            if key == "plan":
                row["switch_s"] = round(sw, 4)
        row["total_s"] = round(totals["plan"], 3)
        row["total_s_at_1"] = round(totals["at_1"], 3)
        row["load_pct"] = round(100.0 * totals["plan"] / budget, 1)
        row["load_pct_at_1"] = round(100.0 * totals["at_1"] / budget, 1)
        over = totals["plan"] > budget
        row["status"] = "FAIL" if over or row["errors"] else "PASS"
        if over:
            row["errors"].append(f"{totals['plan']:.2f} s of moves per drone at derate "
                                 f"{derate:g} > budget {budget:.2f} s (takt {takt:g} - index "
                                 f"{idx:.2f})")
            doc["errors"].append(f"{sid}: {row['errors'][-1]}")

    doc["complete"] = not doc["unresolved"]
    doc["passed"] = not doc["errors"]
    return doc


def station_times(spec: LineSpec, doc: dict) -> dict:
    """Per station with a SCARA, drive or actuate step (D-011..D-014):

        station_time = SCARA step times (tool switches and park moves in) + drive time
                       + slide time + actuate time + the station's index

    SCARA step times are the estimate's per-step times at cycle_derate; drive time = gang head
    `drive_s` x passes, passes = ceil(qty / spindles); slide time = `slide.t_in + t_out` of a head
    on a slide, once per drive step (place, SCARA parks, slide in, drive, slide out); the index is
    spec.station_index (the old pitch / belt_speed is `index_s_old`, for comparison). A null
    input is UNRESOLVED; status FAIL when station_time > takt_s."""
    takt = spec.line.takt_s
    old = index_s(spec)
    stations = station_index(spec)
    step_s = {s["id"]: s["time_s"] for g in doc["scaras"].values() for s in g["steps"]}
    rows: dict = {}
    for sid, st in sorted(spec.stations.items(), key=lambda kv: kv[1].index):
        steps = [s for s in spec.sequence if s.status == "active" and s.station == sid
                 and (robot_served(s) or s.capability in ("drive", "actuate"))]
        if not steps:
            continue
        si = stations.get(sid, {})
        idx = si.get("index_s")
        row = {"scara_s": 0.0, "drive_s": 0.0, "slide_s": 0.0, "actuate_s": 0.0, "index_s": idx,
               "index_s_old": old and round(old, 3), "enter_m": si.get("enter_m"),
               "enter_from": si.get("from"), "total_s": None, "total_s_old_index": None,
               "takt_s": takt, "status": "UNRESOLVED", "steps": [],
               "unresolved": list(si.get("unresolved", []))}
        for s in steps:
            if s.capability == "actuate":
                if s.duration_s is None:
                    row["unresolved"].append(f"{s.id}: duration_s null (actuate {s.device})")
                else:
                    row["actuate_s"] += s.duration_s
                    row["steps"].append({"id": s.id, "kind": "actuate", "device": s.device,
                                         "time_s": s.duration_s})
                continue
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
        for k in ("scara_s", "drive_s", "slide_s", "actuate_s"):
            row[k] = round(row[k], 3)
        if not row["unresolved"]:
            work = row["scara_s"] + row["drive_s"] + row["slide_s"] + row["actuate_s"]
            row["total_s"] = round(work + idx, 3)
            if old is not None:
                row["total_s_old_index"] = round(work + old, 3)
            row["status"] = "PASS" if work + idx <= takt else "FAIL"
        rows[sid] = row
    return rows


def print_station_table(rows: dict) -> None:
    print(f"{'station':<24}{'scara_s':>9}{'drive_s':>9}{'slide_s':>9}{'actuate':>9}"
          f"{'idx_old':>9}{'index_s':>9}{'total_s':>9}{'takt_s':>8}  status")
    for sid, r in rows.items():
        def f(v):
            return f"{v:>9.3f}" if v is not None else f"{'-':>9}"
        print(f"{sid:<24}{f(r['scara_s'])}{f(r['drive_s'])}{f(r['slide_s'])}"
              f"{f(r.get('actuate_s', 0.0))}{f(r.get('index_s_old'))}{f(r['index_s'])}"
              f"{f(r['total_s'])}{r['takt_s']:>8g}  {r['status']}")


def print_gaps(doc: dict) -> None:
    print("queue slots (pitch = pallet length + slot_gap) and the distance into each station:")
    for g in doc["queue_gaps"]:
        def f(v, fmt):
            return format(v, fmt) if v is not None else "-"
        print(f"  {g['after']} -> {g['before']}: gap {f(g['gap_m'], '.3f')} m, "
              f"{f(g['slots'], 'd')} slot(s) at {f(g['slot_pitch_m'], '.3f')} m, enters over "
              f"{f(g['enter_m'], '.3f')} m")


def print_table(doc: dict) -> None:
    derate = doc["cycle_derate"]
    head = f"@{derate:g}" if derate is not None else "@derate"
    for mid, c in doc["calibration"].items():
        if c["time_s"] is not None:
            print(f"calibration {mid}: v_max {c['v_max']:g} m/s, a_max {c['a_max']:g} m/s^2 -> "
                  f"standard cycle {c['time_s']:.4f} s (spec {c['standard_cycle']['time_s']:g} s)"
                  f" {'OK' if c['ok'] else 'MISMATCH'}; moves at x {derate}")
    print(f"{'scara':<25}{'step':<23}{'from':<14}{'qty':>4}{'safe_z':>8}{'switch':>8}"
          f"{'step_s ' + head:>14}{'step_s @1':>11}")
    print("-" * 107)
    for sid, g in doc["scaras"].items():
        for s in g["steps"]:
            ends = ("P" if s["start"] == "park" else "") + ("p" if s["end"] == "park" else "")
            print(f"{sid:<25}{(s['id'] + (' [' + ends + ']' if ends else '')):<23}"
                  f"{s['from'] or '-':<14}{s['qty']:>4}"
                  f"{s['safe_z']:>8.3f}{s['tool_switch_s']:>8.3f}{s['time_s']:>14.3f}"
                  f"{s['time_s_at_1']:>11.3f}")
        if g["total_s"] is not None:
            print(f"{'':<25}{'budget':<23}{'':<14}{'':>4}{'':>8}{'':>8}{g['budget_s']:>14.3f}"
                  f"{g['budget_s']:>11.3f}   {g['load_pct']:.1f}% / {g['load_pct_at_1']:.1f}%"
                  f"  {g['status']}" + (f" ({'; '.join(g['errors'])})" if g["errors"] else ""))
        else:
            print(f"{sid:<25}{'total':<23}{'-':>59}   {g['status']}")
        print("-" * 107)
    print("[P] starts from park_pose, [p] ends at it; budget = takt - the station's index")
    for w in doc["warnings"]:
        print(f"  [WARN] {w}")
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
    ap.add_argument("--calibrate", action="store_true",
                    help="print v_max/a_max that reproduce each model's standard_cycle (the "
                         "current a_max / v_max ratio kept) and exit")
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
    if args.calibrate:
        code = 0
        for mid, m in spec.robot_models.items():
            if m.standard_cycle is None or m.motion.v_max is None or m.motion.a_max is None:
                print(f"calibrate {mid}: no standard_cycle or no v_max/a_max ratio to keep")
                code = 1
                continue
            c = calibrate(m.standard_cycle, m.motion.v_max, m.motion.a_max)
            print(f"calibrate {mid}: x{c['scale']:g} -> v_max {c['v_max']:g} m/s, a_max "
                  f"{c['a_max']:g} m/s^2 (a/v {c['ratio_a_over_v']:g}); standard cycle "
                  f"{c['check_s']:.4f} s; reaches v_max: {c['reaches_v_max']}")
        return code
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
    print_gaps(doc)
    print("station_time = SCARA steps (+ tool switches, park moves) + drive_s x passes + slide "
          "t_in/t_out + actuate + index (enter distance / belt_speed + stop unit):")
    print_station_table(rows)
    print(f"-> {report}")
    return 0 if doc["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
