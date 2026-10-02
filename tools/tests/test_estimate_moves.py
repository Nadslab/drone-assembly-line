"""tools/estimate_moves.py: the SCARA pick-and-place model (D-012, D-013) — home, pick, place, home on a
trapezoid over the TCP distance — the plan speed scale, the takt budget, the per-SCARA safe_z it
moves at, and the station time with a gang head's slide."""

import json
import math
import subprocess
import sys

import pytest
import yaml

import estimate_moves as em
import robot_geometry as rg
import spec as spec_mod
from conftest import FIXTURES, TOOLS


def est(raw):
    return em.estimate(spec_mod.parse_spec(raw), rg.CadHeights(FIXTURES / "cad"))


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
def test_one_cycle_is_home_pick_place_home_with_four_settles(pass_raw, scale):
    spec = spec_mod.parse_spec(pass_raw)
    motion, d = spec_mod.scara_model(spec, "scara_place").motion, spec.scara_defaults
    home, pick, place = (0.5, 0.14), (0.3, 0.3, 0.15), (0.5, 0.0, 0.1)
    c = em.cycle(home, pick, place, motion, d, 0.3, scale)

    def t(dist):
        return em.axis_time(dist, 1.0 * scale, 2.0 * scale)

    assert c["home_to_pick"] == pytest.approx(t(math.hypot(0.2, 0.16)), abs=1e-4)
    assert c["pick_to_place"] == pytest.approx(t(math.hypot(0.2, 0.3)), abs=1e-4)
    assert c["place_to_home"] == pytest.approx(t(0.14), abs=1e-4)
    assert c["z_down_pick"] == pytest.approx(t(0.15) + d.settle_s, abs=1e-4)
    assert c["z_up_place"] == pytest.approx(t(0.2) + d.settle_s, abs=1e-4)
    assert c["total"] == pytest.approx(
        t(math.hypot(0.2, 0.16)) + t(math.hypot(0.2, 0.3)) + t(0.14)
        + 2 * t(0.15) + 2 * t(0.2) + 4 * d.settle_s + d.grip_s + d.release_s, abs=1e-3)


def test_home_is_the_arm_folded_to_r_min_towards_its_station(pass_raw):
    g = est(pass_raw)["scaras"]["scara_place"]
    assert g["base"] == [0.5, 0.24, 0.0]
    # r_min = the elbow folded to its j2 limit: 0.1 m from the base, towards y 0.
    assert g["home"] == pytest.approx([0.5, 0.14], abs=1e-4)


# ---------------------------------------------------------------- which steps, which scale

def test_qty_repeats_the_whole_cycle(pass_raw):
    pass_raw["sequence"][1]["qty"] = 3             # place: 3 mid plates
    doc = est(pass_raw)
    step = doc["scaras"]["scara_place"]["steps"][0]
    assert step["qty"] == 3
    assert step["time_s"] == pytest.approx(3 * step["cycle"]["total"], abs=1e-3)


def test_each_scara_only_counts_the_steps_at_its_station(pass_raw):
    doc = est(pass_raw)
    assert {sid: [s["id"] for s in g["steps"]] for sid, g in doc["scaras"].items()} == {
        "scara_load": ["load"], "scara_place": ["place"], "scara_spider": ["place_spider"],
        "scara_pod": ["remove_spider", "place_pod"]}
    assert doc["passed"] and doc["complete"]


def test_a_drive_step_is_the_drivers_and_a_pick_step_is_the_scaras(pass_raw):
    pass_raw["sequence"] += [
        {"id": "screw", "station": "st_place", "capability": "drive", "qty": 4,
         "duration_s": 1.0, "requires": ["place"]},
        {"id": "pick_again", "station": "st_place", "capability": "pick", "part": "mid_plate",
         "from": "fd_mid_plate", "tool": "tool_plate", "duration_s": 1.0, "requires": ["place"]}]
    ids = [s["id"] for s in est(pass_raw)["scaras"]["scara_place"]["steps"]]
    assert "screw" not in ids and "pick_again" in ids


def test_the_plan_scale_is_slower_than_full_speed_and_1_0_is_full_speed(pass_raw):
    g = est(pass_raw)["scaras"]["scara_place"]
    assert g["total_s"] > g["total_s_at_1"]
    pass_raw["scara_defaults"]["plan_speed_scale"] = 1.0
    g1 = est(pass_raw)["scaras"]["scara_place"]
    assert g1["total_s"] == pytest.approx(g1["total_s_at_1"]) == pytest.approx(g["total_s_at_1"])


# ---------------------------------------------------------------- budget and safe_z

def test_the_budget_is_takt_minus_the_transfer(pass_raw):
    doc = est(pass_raw)
    lay = pass_raw["layout"]
    transfer = pass_raw["conveyor"]["pitch"] / lay["conveyor"]["belt_speed"] \
        + lay["stop_unit_cycle_s"]
    assert doc["transfer_s"] == pytest.approx(transfer, abs=1e-3)
    assert doc["budget_s"] == pytest.approx(pass_raw["line"]["takt_s"] - transfer, abs=1e-3)
    g = doc["scaras"]["scara_pod"]
    assert g["load_pct"] == pytest.approx(100.0 * g["total_s"] / doc["budget_s"], abs=0.1)


def test_a_load_under_takt_but_over_the_budget_fails(pass_raw):
    base = est(pass_raw)["scaras"]["scara_pod"]["total_s"]
    takt = pass_raw["line"]["takt_s"]
    # Leave a budget 1 s short of scara_pod's load, while the load stays under takt.
    pass_raw["layout"]["stop_unit_cycle_s"] = takt - base + 1.0 \
        - pass_raw["conveyor"]["pitch"] / pass_raw["layout"]["conveyor"]["belt_speed"]
    doc = est(pass_raw)
    assert doc["scaras"]["scara_pod"]["total_s"] < takt
    assert doc["scaras"]["scara_pod"]["status"] == "FAIL"


def test_each_step_moves_at_its_own_path_based_safe_z(pass_raw):
    g = est(pass_raw)["scaras"]["scara_pod"]
    sz = g["safe_z"]
    # remove_spider (D-013): lifting the spider carrier (105.4 mm) off its own stack (104.4 mm)
    # and over to rt_fixture is the highest move, + 0.03.
    assert sz["tallest_item"] == ["station st_pod (before remove_spider)", pytest.approx(0.1044)]
    assert sz["hang"] == ["carrier_spider", pytest.approx(0.1054)]
    assert sz["safe_z"] == pytest.approx(0.1044 + 0.1054 + 0.03)
    # place_pod moves at its own, lower height: the carrier is gone, buf_pod (0.1) is the top.
    steps = {s["id"]: s for s in g["steps"]}
    assert steps["remove_spider"]["safe_z"] == pytest.approx(sz["safe_z"])
    assert steps["place_pod"]["safe_z"] == pytest.approx(0.1 + 0.0795 + 0.03)
    # Nothing off its paths counts: st_spider's stack is 0.5 m away.
    names = [n for n, _ in sz["items"]]
    assert not any("st_spider" in n for n in names) and "return path rt_fixture" in names


def test_a_missing_cad_report_is_unresolved_not_a_guess(pass_raw, tmp_path):
    doc = em.estimate(spec_mod.parse_spec(pass_raw), rg.CadHeights(tmp_path))
    assert doc["scaras"]["scara_pod"]["status"] == "UNRESOLVED"
    assert any("cad_carrier_pod.json" in u for u in doc["unresolved"])
    assert doc["passed"] is True and doc["complete"] is False


def test_a_safe_z_above_the_stroke_fails_but_keeps_its_breakdown(pass_raw):
    pass_raw["robot_models"]["test_scara"]["z_stroke"] = 0.2
    doc = est(pass_raw)
    assert not doc["passed"]
    assert any("scara_pod: remove_spider: safe_z=0.240 is above its z range top 0.2" in e
               for e in doc["errors"])
    g = doc["scaras"]["scara_pod"]
    assert g["status"] == "FAIL" and g["total_s"] is not None and g["steps"]


# ---------------------------------------------------------------- station time (D-011, D-012)

def test_station_time_is_scara_steps_plus_drive_passes_plus_index(pass_raw):
    spec = spec_mod.parse_spec(pass_raw)
    doc = est(pass_raw)
    rows = em.station_times(spec, doc)
    idx = 0.5 / 0.3
    place = doc["scaras"]["scara_place"]["steps"][0]
    assert rows["st_place"]["total_s"] == pytest.approx(place["time_s"] + idx, abs=1e-3)
    assert rows["st_drive"]["drive_s"] == pytest.approx(3.0)          # 8 screws / 8 spindles
    assert rows["st_drive"]["slide_s"] == 0.0                          # no SCARA there, no slide
    assert rows["st_drive"]["total_s"] == pytest.approx(3.0 + idx, abs=1e-3)
    assert "st_test" not in rows                                      # no SCARA or drive step


def test_a_slide_adds_its_in_and_out_once_per_drive_step(pass_raw):
    """D-012: SCARA places, clears, slide in, drive, slide out."""
    from conftest import _slide_at_place
    _slide_at_place(0.5, 0.75)(pass_raw)
    spec = spec_mod.parse_spec(pass_raw)
    doc = est(pass_raw)
    row = em.station_times(spec, doc)["st_place"]
    assert row["slide_s"] == pytest.approx(1.25)
    assert row["total_s"] == pytest.approx(row["scara_s"] + 3.0 + 1.25 + 0.5 / 0.3, abs=1e-3)


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
    assert set(doc["scaras"]["scara_place"]) >= {"steps", "total_s", "total_s_at_1", "load_pct",
                                                 "budget_s", "safe_z", "status", "home"}
    assert set(doc["station_times"]) == {"st_load", "st_place", "st_drive", "st_spider", "st_pod"}
    assert "step_s @0.5" in proc.stdout and "step_s @1" in proc.stdout
    assert "budget = takt" in proc.stdout and "slide_s" in proc.stdout

    pass_raw["robot_models"]["test_scara"]["motion"]["v_max"] = 0.01
    proc, doc = run(pass_raw)
    assert proc.returncode == 1
    assert doc["passed"] is False and doc["scaras"]["scara_pod"]["status"] == "FAIL"
