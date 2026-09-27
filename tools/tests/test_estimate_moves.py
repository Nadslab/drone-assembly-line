"""tools/estimate_moves.py: the trapezoidal move model and the per-gantry tour against takt."""

import json
import math
import subprocess
import sys

import pytest
import yaml

import estimate_moves as em
import spec as spec_mod
from conftest import TOOLS


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


def test_one_cycle_is_the_eight_phases_with_four_settles(pass_raw):
    spec = spec_mod.parse_spec(pass_raw)
    g, d = spec.gantries["gantry_1"], spec.gantry_defaults
    pick, place = (0.5, 0.3, 0.1), (0.5, 0.0, 0.1)
    c = em.cycle(place, pick, place, g.axes, d)
    xy = em.axis_time(0.3, 1.0, 2.0)                 # only y moves
    z = em.axis_time(0.2, 0.5, 2.0)                  # safe_z 0.3 down to 0.1
    assert c["xy_to_pick"] == pytest.approx(xy, abs=1e-4)
    assert c["total"] == pytest.approx(2 * xy + 4 * (z + d.settle_s) + d.grip_s + d.release_s,
                                       abs=1e-3)


def test_qty_repeats_the_cycle_from_the_station(pass_raw):
    pass_raw["sequence"][1]["qty"] = 3             # place: 3 mid plates
    doc = em.estimate(spec_mod.parse_spec(pass_raw))
    step = next(s for s in doc["gantries"]["gantry_1"]["steps"] if s["id"] == "place")
    assert step["qty"] == 3
    assert step["time_s"] == pytest.approx(step["first_cycle"]["total"]
                                           + 2 * step["repeat_cycle"]["total"], abs=1e-3)


def test_each_gantry_only_counts_the_steps_in_its_zone(pass_raw):
    doc = em.estimate(spec_mod.parse_spec(pass_raw))
    assert [s["id"] for s in doc["gantries"]["gantry_1"]["steps"]] == ["load", "place"]
    assert [s["id"] for s in doc["gantries"]["gantry_2"]["steps"]] == ["place_spider",
                                                                       "place_pod"]
    assert doc["passed"] and doc["complete"]


def test_a_pick_above_safe_z_is_a_failure(pass_raw):
    pass_raw["gantry_defaults"]["safe_z"] = 0.12      # fd_mid_plate stands at 0.15
    doc = em.estimate(spec_mod.parse_spec(pass_raw))
    assert not doc["passed"]
    assert any("pick z=0.15 is above safe_z=0.12" in e for e in doc["errors"])


def test_cli_writes_the_report_and_exits_on_overload(tmp_path, pass_raw):
    def run(raw):
        spec = tmp_path / "spec.yaml"
        spec.write_text(yaml.safe_dump(raw))
        report = tmp_path / "move_estimate.json"
        proc = subprocess.run([sys.executable, str(TOOLS / "estimate_moves.py"), "--spec",
                               str(spec), "--report", str(report)], capture_output=True, text=True)
        return proc, json.loads(report.read_text())

    proc, doc = run(pass_raw)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert doc["passed"] is True
    assert set(doc["gantries"]["gantry_1"]) >= {"steps", "total_s", "load_pct", "status"}
    assert "gantry_1" in proc.stdout and "% of takt" in proc.stdout

    pass_raw["gantries"]["gantry_2"]["axes"]["x"]["v_max"] = 0.1
    proc, doc = run(pass_raw)
    assert proc.returncode == 1
    assert doc["passed"] is False and doc["gantries"]["gantry_2"]["status"] == "FAIL"
