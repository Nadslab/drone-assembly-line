"""tools/estimate_moves.py: the trapezoidal move model, the per-gantry tour, the plan speed scale,
the takt budget, and the per-gantry safe_z it moves at (D-009)."""

import json
import math
import subprocess
import sys

import pytest
import yaml

import estimate_moves as em
import gantry_geometry as gg
import spec as spec_mod
from conftest import FIXTURES, TOOLS


def est(raw):
    return em.estimate(spec_mod.parse_spec(raw), gg.CadHeights(FIXTURES / "cad"))


# ---------------------------------------------------------------- the profile

def test_a_long_move_reaches_v_max():
    # d >= v^2/a: cruise at v, plus one v/a for the two ramps together.
    assert em.axis_time(2.0, 1.0, 2.0) == pytest.approx(2.0 / 1.0 + 1.0 / 2.0)


def test_a_short_move_is_triangular():
    # d < v^2/a = 0.5: never reaches v.
    assert em.axis_time(0.3, 1.0, 2.0) == pytest.approx(2.0 * math.sqrt(0.3 / 2.0))


def test_the_two_profiles_meet_at_the_switch_distance():
    v, a = 1.0, 3.0
    d = v * v / a
    assert em.axis_time(d, v, a) == pytest.approx(2.0 * math.sqrt(d / a))


def test_no_move_takes_no_time_and_direction_does_not_matter():
    assert em.axis_time(0.0, 1.0, 2.0) == 0.0
    assert em.axis_time(-0.7, 1.0, 2.0) == em.axis_time(0.7, 1.0, 2.0)


@pytest.mark.parametrize("scale", [1.0, 0.5])
def test_one_cycle_is_the_eight_phases_with_four_settles(pass_raw, scale):
    spec = spec_mod.parse_spec(pass_raw)
    g, d = spec.gantries["gantry_1"], spec.gantry_defaults
    pick, place = (0.5, 0.3, 0.1), (0.5, 0.0, 0.1)
    c = em.cycle(place, pick, place, g.axes, d, 0.3, scale)
    xy = em.axis_time(0.3, 1.0 * scale, 2.0 * scale)          # only y moves
    z = em.axis_time(0.2, 0.5 * scale, 2.0 * scale)           # safe_z 0.3 down to 0.1
    assert c["xy_to_pick"] == pytest.approx(xy, abs=1e-4)
    assert c["total"] == pytest.approx(2 * xy + 4 * (z + d.settle_s) + d.grip_s + d.release_s,
                                       abs=1e-3)


# ---------------------------------------------------------------- which steps, which scale

def test_qty_repeats_the_cycle_from_the_station(pass_raw):
    pass_raw["sequence"][1]["qty"] = 3             # place: 3 mid plates
    doc = est(pass_raw)
    step = next(s for s in doc["gantries"]["gantry_1"]["steps"] if s["id"] == "place")
    assert step["qty"] == 3
    assert step["time_s"] == pytest.approx(step["first_cycle"]["total"]
                                           + 2 * step["repeat_cycle"]["total"], abs=1e-3)


def test_each_gantry_only_counts_the_steps_in_its_zone(pass_raw):
    doc = est(pass_raw)
    assert [s["id"] for s in doc["gantries"]["gantry_1"]["steps"]] == ["load", "place"]
    assert [s["id"] for s in doc["gantries"]["gantry_2"]["steps"]] == ["place_spider",
                                                                       "place_pod"]
    assert doc["passed"] and doc["complete"]


def test_a_drive_step_is_the_drivers_and_a_pick_step_is_the_gantrys(pass_raw):
    pass_raw["sequence"] += [
        {"id": "screw", "station": "st_place", "capability": "drive", "qty": 4,
         "duration_s": 1.0, "requires": ["place"]},
        {"id": "pick_again", "station": "st_place", "capability": "pick", "part": "mid_plate",
         "from": "fd_mid_plate", "tool": "tool_plate", "duration_s": 1.0, "requires": ["place"]}]
    ids = [s["id"] for s in est(pass_raw)["gantries"]["gantry_1"]["steps"]]
    assert "screw" not in ids and "pick_again" in ids


def test_the_plan_scale_is_slower_than_full_speed_and_1_0_is_full_speed(pass_raw):
    g = est(pass_raw)["gantries"]["gantry_1"]
    assert g["total_s"] > g["total_s_at_1"]
    pass_raw["gantry_defaults"]["plan_speed_scale"] = 1.0
    g1 = est(pass_raw)["gantries"]["gantry_1"]
    assert g1["total_s"] == pytest.approx(g1["total_s_at_1"]) == pytest.approx(g["total_s_at_1"])


# ---------------------------------------------------------------- budget and safe_z

def test_the_budget_is_takt_minus_the_transfer(pass_raw):
    doc = est(pass_raw)
    lay = pass_raw["layout"]
    transfer = pass_raw["conveyor"]["pitch"] / lay["conveyor"]["belt_speed"] \
        + lay["stop_unit_cycle_s"]
    assert doc["transfer_s"] == pytest.approx(transfer, abs=1e-3)
    assert doc["budget_s"] == pytest.approx(pass_raw["line"]["takt_s"] - transfer, abs=1e-3)
    g = doc["gantries"]["gantry_2"]
    assert g["load_pct"] == pytest.approx(100.0 * g["total_s"] / doc["budget_s"], abs=0.1)


def test_a_load_under_takt_but_over_the_budget_fails(pass_raw):
    base = est(pass_raw)["gantries"]["gantry_2"]["total_s"]
    takt = pass_raw["line"]["takt_s"]
    # Leave a budget 1 s short of gantry_2's load, while the load stays under takt.
    pass_raw["layout"]["stop_unit_cycle_s"] = takt - base + 1.0 \
        - pass_raw["conveyor"]["pitch"] / pass_raw["layout"]["conveyor"]["belt_speed"]
    doc = est(pass_raw)
    assert doc["gantries"]["gantry_2"]["total_s"] < takt
    assert doc["gantries"]["gantry_2"]["status"] == "FAIL"


def test_safe_z_is_the_tallest_item_plus_the_tallest_hang_plus_the_margin(pass_raw):
    g = est(pass_raw)["gantries"]
    # gantry_1: st_load and st_place stand at 0.1 with the pallet pins (68 mm) the tallest,
    # and it carries plates 3 mm thick.
    assert g["gantry_1"]["safe_z"]["safe_z"] == pytest.approx(0.1 + 0.068 + 0.003 + 0.03)
    # gantry_2: the pod tray at st_pod tops out at 136.5 mm; the spider carrier hangs 105.4 mm.
    assert g["gantry_2"]["safe_z"]["tallest_item"] == ["station st_pod", pytest.approx(0.2365)]
    assert g["gantry_2"]["safe_z"]["hang"] == ["carrier_spider", pytest.approx(0.1054)]
    assert g["gantry_2"]["safe_z"]["safe_z"] == pytest.approx(0.2365 + 0.1054 + 0.03)


def test_a_missing_cad_report_is_unresolved_not_a_guess(pass_raw, tmp_path):
    doc = em.estimate(spec_mod.parse_spec(pass_raw), gg.CadHeights(tmp_path))
    assert doc["gantries"]["gantry_2"]["status"] == "UNRESOLVED"
    assert any("cad_carrier_pod.json" in u for u in doc["unresolved"])
    assert doc["passed"] is True and doc["complete"] is False


def test_a_safe_z_the_gantry_cannot_reach_is_a_failure(pass_raw):
    pass_raw["gantries"]["gantry_2"]["axes"]["z"]["max"] = 0.3
    doc = est(pass_raw)
    assert not doc["passed"]
    assert any("outside its travel" in e for e in doc["errors"])


# ---------------------------------------------------------------- station time (D-011)

def test_station_time_is_gantry_steps_plus_drive_passes_plus_index(pass_raw):
    spec = spec_mod.parse_spec(pass_raw)
    doc = est(pass_raw)
    rows = em.station_times(spec, doc)
    idx = 0.5 / 0.3
    place = next(s for s in doc["gantries"]["gantry_1"]["steps"] if s["id"] == "place")
    assert rows["st_place"]["total_s"] == pytest.approx(place["time_s"] + idx, abs=1e-3)
    assert rows["st_drive"]["drive_s"] == pytest.approx(3.0)          # 8 screws / 8 spindles
    assert rows["st_drive"]["total_s"] == pytest.approx(3.0 + idx, abs=1e-3)
    assert "st_test" not in rows                                      # no gantry or drive step


def test_more_screws_than_spindles_take_more_passes(pass_raw):
    pass_raw["gang_heads"]["gh_top"].update(spindles=3, pattern_from=None)
    rows = em.station_times(spec_mod.parse_spec(pass_raw), est(pass_raw))
    step = rows["st_drive"]["steps"][0]
    assert step["passes"] == 3                                        # ceil(8 / 3)
    assert rows["st_drive"]["drive_s"] == pytest.approx(9.0)


def test_station_time_over_takt_names_its_parts(pass_raw):
    import check_line
    pass_raw["gang_heads"]["gh_top"]["drive_s"] = 19.0
    r = next(r for r in check_line.run_checks(pass_raw)[0] if r.name == "station_time")
    assert r.findings == [("FAIL", "st_drive: 20.67 s (drive 19.00, index 1.67) > takt 20 s")]


# ---------------------------------------------------------------- CLI

def test_cli_writes_the_report_and_exits_on_overload(tmp_path, pass_raw):
    def run(raw):
        spec = tmp_path / "spec.yaml"
        spec.write_text(yaml.safe_dump(raw))
        report = tmp_path / "move_estimate.json"
        proc = subprocess.run([sys.executable, str(TOOLS / "estimate_moves.py"), "--spec",
                               str(spec), "--report", str(report), "--cad-reports",
                               str(FIXTURES / "cad")], capture_output=True, text=True)
        return proc, json.loads(report.read_text())

    proc, doc = run(pass_raw)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert doc["passed"] is True
    assert set(doc["gantries"]["gantry_1"]) >= {"steps", "total_s", "total_s_at_1", "load_pct",
                                                "budget_s", "safe_z", "status"}
    assert "step_s @0.5" in proc.stdout and "step_s @1" in proc.stdout
    assert "budget = takt" in proc.stdout

    pass_raw["gantries"]["gantry_2"]["axes"]["x"]["v_max"] = 0.01
    proc, doc = run(pass_raw)
    assert proc.returncode == 1
    assert doc["passed"] is False and doc["gantries"]["gantry_2"]["status"] == "FAIL"
