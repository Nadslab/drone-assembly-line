"""tools/estimate_moves.py: the SCARA pick-and-place model (D-012..D-014) — home (or park), pick,
place, home (or park) on a trapezoid over the TCP distance — the cycle derate, the calibration to a
vendor standard cycle, the takt budget per station, the per-SCARA safe_z it moves at, and the
station time with a gang head's slide and the per-station index."""

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
    """`scale` is the cycle derate (D-014)."""
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


def test_the_derate_is_slower_than_full_speed_and_1_0_is_full_speed(pass_raw):
    g = est(pass_raw)["scaras"]["scara_place"]
    assert g["total_s"] > g["total_s_at_1"]
    pass_raw["scara_defaults"]["cycle_derate"] = 1.0
    g1 = est(pass_raw)["scaras"]["scara_place"]
    assert g1["total_s"] == pytest.approx(g1["total_s_at_1"]) == pytest.approx(g["total_s_at_1"])


# ---------------------------------------------------------------- budget and safe_z

def test_the_budget_is_takt_minus_the_station_index(pass_raw):
    """D-014: the pallet enters st_pod over 0.25 m from its queue slot at 0.3 m/s, + 0.5 s stop."""
    doc = est(pass_raw)
    lay = pass_raw["layout"]
    index = 0.25 / lay["conveyor"]["belt_speed"] + lay["stop_unit_cycle_s"]
    g = doc["scaras"]["scara_pod"]
    assert g["index_s"] == pytest.approx(index)
    assert g["budget_s"] == pytest.approx(pass_raw["line"]["takt_s"] - index, abs=1e-3)
    assert g["load_pct"] == pytest.approx(100.0 * g["total_s"] / g["budget_s"], abs=0.1)
    assert doc["index_s_old"] == pytest.approx(0.5 / 0.3, abs=1e-3)


def test_a_load_under_takt_but_over_the_budget_fails(pass_raw):
    base = est(pass_raw)["scaras"]["scara_pod"]["total_s"]
    takt = pass_raw["line"]["takt_s"]
    # Leave a budget 1 s short of scara_pod's load, while the load stays under takt.
    pass_raw["layout"]["stop_unit_cycle_s"] = takt - base + 1.0 \
        - 0.25 / pass_raw["layout"]["conveyor"]["belt_speed"]
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
    idx = 0.25 / 0.3 + 0.5                  # from the queue slot, + the stop unit (D-014)
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
    assert row["total_s"] == pytest.approx(row["scara_s"] + 3.0 + 1.25 + 0.25 / 0.3 + 0.5,
                                           abs=1e-3)


def test_more_screws_than_spindles_take_more_passes(pass_raw):
    pass_raw["gang_heads"]["gh_top"].update(spindles=3, pattern_from=None)
    rows = em.station_times(spec_mod.parse_spec(pass_raw), est(pass_raw))
    step = rows["st_drive"]["steps"][0]
    assert step["passes"] == 3                                        # ceil(8 / 3)
    assert rows["st_drive"]["drive_s"] == pytest.approx(9.0)


def test_a_loader_without_a_scara_gets_its_load_duration_and_index(pass_raw):
    """D-016: the single-purpose loader's row is its load step's duration_s + its index."""
    from conftest import _sub_line_loader
    _sub_line_loader("buf_plate")(pass_raw)
    del pass_raw["scaras"]["scara_load"]
    spec = spec_mod.parse_spec(pass_raw)
    row = em.station_times(spec, est(pass_raw))["st_load"]
    assert row["scara_s"] == 0.0 and row["drive_s"] == 0.0 and row["actuate_s"] == 0.0
    assert row["load_s"] == pytest.approx(5.0)
    assert row["steps"] == [{"id": "load", "kind": "load", "time_s": 5.0}]
    assert row["total_s"] == pytest.approx(5.0 + row["index_s"], abs=1e-3)
    assert row["status"] == "PASS"

    pass_raw["sequence"][0]["duration_s"] = None
    row = em.station_times(spec_mod.parse_spec(pass_raw), est(pass_raw))["st_load"]
    assert row["total_s"] is None and row["unresolved"] == [
        "load: duration_s null (load, no SCARA)"]


def test_station_time_over_takt_names_its_parts(pass_raw):
    import check_line
    pass_raw["gang_heads"]["gh_top"]["drive_s"] = 19.0
    r = next(r for r in check_line.run_checks(pass_raw)[0] if r.name == "station_time")
    assert r.findings == [("FAIL", "st_drive: 20.33 s (drive 19.00, index 1.33) > takt 20 s")]


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


# ---------------------------------------------------------------- D-014

def test_calibration_keeps_v_max_and_solves_a_max():
    """Epson LS10-B (D-015): v_max 9.8 m/s is the vendor's J1+J2 maximum; the standard cycle
    (0.41 s for 300 mm across, 25 mm up and down, there and back) fixes a_max alone."""
    sc = spec_mod.StandardCycle(time_s=0.41, horizontal_m=0.3, vertical_m=0.025, payload_kg=2.0,
                                motion="round_trip_arch")
    c = em.calibrate(sc, 9.8)
    assert c["v_max"] == 9.8
    assert em.standard_cycle_time(sc, 9.8, c["a_max"]) == pytest.approx(0.41, abs=1e-4)
    # No leg reaches 9.8 m/s (v^2/a = 1.35 m > 0.3 m): every leg is triangular, 4 sqrt(d/a) per
    # leg pair, so a_max is the D-014 value whatever v_max is above it.
    a = c["a_max"]
    assert c["reaches_v_max"] == {"vertical": False, "horizontal": False}
    assert 2 * (2 * 2 * math.sqrt(0.025 / a) + 2 * math.sqrt(0.3 / a)) == pytest.approx(0.41,
                                                                                    abs=1e-4)
    assert a == pytest.approx(71.0444, abs=1e-3)
    # A v_max low enough to bind: a_max rises to make up for it, the cycle still fits.
    slow = em.calibrate(sc, 2.0)
    assert slow["a_max"] > a and slow["reaches_v_max"]["horizontal"]
    assert em.standard_cycle_time(sc, 2.0, slow["a_max"]) == pytest.approx(0.41, abs=1e-4)
    # Too slow at any acceleration: 0.7 m of travel at 1 m/s alone is 0.7 s.
    assert "error" in em.calibrate(sc, 1.0)


def test_stored_motion_that_misses_the_standard_cycle_fails(pass_raw):
    pass_raw["robot_models"]["test_scara"]["standard_cycle"] = {
        "time_s": 0.41, "horizontal_m": 0.3, "vertical_m": 0.025, "payload_kg": 2.0,
        "motion": "round_trip_arch"}
    doc = est(pass_raw)
    assert not doc["passed"]
    assert any("robot_models.test_scara.motion" in e and "re-run estimate_moves.py --calibrate"
               in e for e in doc["errors"])
    c = em.calibrate(spec_mod.parse_spec(pass_raw).robot_models["test_scara"].standard_cycle,
                     2.0)
    pass_raw["robot_models"]["test_scara"]["motion"]["v_max"] = 2.0
    pass_raw["robot_models"]["test_scara"]["motion"].update(v_max=c["v_max"], a_max=c["a_max"])
    doc = est(pass_raw)
    assert doc["calibration"]["test_scara"]["ok"] is True and doc["passed"]


def test_a_payload_over_the_standard_cycles_warns(pass_raw):
    """scara_spider lifts 0.3 kg of tool + 0.6 kg of carrier: over a 0.8 kg standard cycle."""
    sc = {"time_s": 0.41, "horizontal_m": 0.3, "vertical_m": 0.025, "payload_kg": 0.8,
          "motion": "round_trip_arch"}
    pass_raw["robot_models"]["test_scara"]["standard_cycle"] = sc
    doc = est(pass_raw)
    assert doc["scaras"]["scara_spider"]["warnings"] == [
        "place_spider: payload 0.900 kg (tools + carrier_spider) > the standard cycle's 0.8 kg: "
        "its times are optimistic"]
    assert doc["scaras"]["scara_place"]["warnings"] == []          # 0.3 + 0.05 kg


def test_around_a_drive_the_scara_parks_and_starts_again_from_park(pass_raw):
    """gh_top drives at st_place: scara_place's one step ends at park_pose and (the next drone's)
    starts from it — place -> park and park -> pick replace the two home legs."""
    from conftest import _slide_at_place
    _slide_at_place(0.5, 0.5)(pass_raw)
    spec = spec_mod.parse_spec(pass_raw)
    assert rg.park_steps(spec, "scara_place") == ({"place"}, {"place"})
    g = est(pass_raw)["scaras"]["scara_place"]
    step = g["steps"][0]
    assert (step["start"], step["end"]) == ("park", "park")
    assert set(step["cycle"]) >= {"park_to_pick", "place_to_park"}
    k = rg.kin(spec, "scara_place")
    park = rg.fk(k, 2.5, 2.281994)
    assert g["park_tcp"] == pytest.approx(list(park), abs=1e-4)
    assert step["dist_m"]["place_to_end"] == pytest.approx(math.hypot(park[0] - 0.5, park[1]),
                                                           abs=1e-4)


# ---------------------------------------------------------------- D-015


def test_a_park_leg_is_timed_in_joint_space(pass_raw):
    """D-015 (D-014 L): place -> park and park -> pick are J1/J2 rotations, each joint on its own
    trapezoid at v_max/a_max x derate over its link radius (J1: link1 + link2, J2: link2); the
    slower joint sets the time. The straight TCP line is not used for them."""
    from conftest import _slide_at_place
    _slide_at_place(0.5, 0.5)(pass_raw)
    spec = spec_mod.parse_spec(pass_raw)
    g = est(pass_raw)["scaras"]["scara_place"]
    step = g["steps"][0]
    k = rg.kin(spec, "scara_place")
    der = pass_raw["scara_defaults"]["cycle_derate"]
    v, a = 1.0 * der, 2.0 * der                       # test_scara motion x derate
    lim = rg.joint_limits_from_tcp(k, v, a)
    assert lim["j1"] == pytest.approx((v / 0.24, a / 0.24))
    assert lim["j2"] == pytest.approx((v / 0.12, a / 0.12))
    park = (2.5, 2.281994)
    for leg, ends in (("park_to_pick", rg.leg_ends(k, None, step["pick"][:2], ja=park)),
                      ("place_to_park", rg.leg_ends(k, step["place"][:2], None, jb=park))):
        (p1, p2), (q1, q2) = ends
        want = max(em.axis_time(q1 - p1, *lim["j1"]), em.axis_time(q2 - p2, *lim["j2"]))
        assert step["park_legs"][leg]["time_s"] == pytest.approx(want)
        assert step["cycle"][leg] == pytest.approx(want, abs=1e-4)
        assert step["park_legs"][leg]["dj1_deg"] == pytest.approx(math.degrees(q1 - p1), abs=0.01)
    # The straight line would be shorter: 0.11 m at 0.5 m/s, but the arm swings j1 ~91 deg.
    assert step["cycle"]["place_to_park"] > em.axis_time(step["dist_m"]["place_to_end"], v, a)


def test_cli_stations_estimates_only_that_run(tmp_path, pass_raw):
    spec = tmp_path / "spec.yaml"
    spec.write_text(yaml.safe_dump(pass_raw))
    report = tmp_path / "move_estimate.b.json"
    proc = subprocess.run([sys.executable, str(TOOLS / "estimate_moves.py"), "--spec", str(spec),
                           "--report", str(report), "--cad-reports", str(FIXTURES / "cad"),
                           "--stations", "st_spider", "st_pod", "--name", "b"],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    doc = json.loads(report.read_text())
    assert set(doc["scaras"]) == {"scara_spider", "scara_pod"}
    assert doc["stations"] == ["st_spider", "st_pod"]
    assert doc["station_index"]["st_spider"]["from"] == "slot 1 after st_test"
