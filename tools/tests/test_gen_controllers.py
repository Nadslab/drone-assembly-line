"""P5.3: the controller config, derived from the spec.

The load-bearing assertions are `test_each_gantry_controller_covers_exactly_the_axes_it_keeps`
and `test_an_opposed_head_gets_a_controller_over_both_banks` — the joint names here have to be the
ones urdf/line.urdf.xacro writes, and a controller over the wrong set of joints is a line that
either refuses to start or silently never commands an axis. tools/checks/descriptions.sh is what
compares the two derivations; these tests pin this side of it.
"""

import copy
import json

import pytest
import yaml

import gen_controllers as gc
import spec as spec_mod


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    monkeypatch.setattr(gc, "report_path",
                        lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json")


@pytest.fixture
def run(tmp_path, pass_raw):
    def go(raw=None, *extra, overlay=None):
        spec = tmp_path / "line_spec.yaml"
        spec.write_text(yaml.safe_dump(pass_raw if raw is None else raw))
        if overlay is not None:
            (tmp_path / "line_spec.dev.yaml").write_text(yaml.safe_dump(overlay))
        dev = "--dev" in extra
        out = tmp_path / "config" / f"controllers{'.dev' if dev else ''}.yaml"
        code = gc.main(["--spec", str(spec), "--out", str(out), *extra])
        name = f"gen_controllers{'.dev' if dev else ''}.json"
        doc = json.loads((tmp_path / name).read_text())
        return code, doc, (yaml.safe_load(out.read_text()) if out.exists() else None), out
    return go


def joints_of(config, controller):
    return config[controller]["ros__parameters"]["joints"]


# ---------------------------------------------------------------- what it writes

def test_the_header_says_it_is_generated(run):
    code, doc, _, out = run()
    assert code == 0, doc["errors"] + doc["missing"]
    assert out.read_text().startswith("# GENERATED from line_spec.yaml by gen_controllers.py")


def test_one_controller_per_gantry_one_per_head_and_the_broadcaster(run):
    code, doc, config, _ = run()
    assert code == 0, doc["errors"] + doc["missing"]
    manager = config["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == gc.UPDATE_RATE_HZ
    assert set(doc["controllers"]) == {"joint_state_broadcaster", "gantry_1_controller",
                                       "gantry_2_controller", "gh_top_controller"}
    assert manager["joint_state_broadcaster"]["type"] == gc.JSB_TYPE
    assert manager["gantry_1_controller"]["type"] == gc.JTC_TYPE
    assert manager["gantry_2_controller"]["type"] == gc.JTC_TYPE
    # The broadcaster takes whatever the hardware has, so it gets no joint list of its own.
    assert "joint_state_broadcaster" not in config


def test_each_gantry_controller_covers_exactly_the_axes_it_keeps(run, pass_raw):
    code, doc, config, _ = run()
    assert code == 0, doc["errors"] + doc["missing"]
    for gid in ("gantry_1", "gantry_2"):
        assert joints_of(config, f"{gid}_controller") == [f"{gid}_x", f"{gid}_y", f"{gid}_z",
                                                           f"{gid}_c"]

    no_yaw = copy.deepcopy(pass_raw)
    del no_yaw["gantries"]["gantry_1"]["axes"]["c"]   # "delete if no yaw needed" (line_spec.yaml)
    code, doc, config, _ = run(no_yaw)
    assert code == 0, doc["errors"] + doc["missing"]
    assert joints_of(config, "gantry_1_controller") == ["gantry_1_x", "gantry_1_y", "gantry_1_z"]
    assert "gantry_1_c" not in doc["joint_limits"]
    assert "gantry_2_c" in doc["joint_limits"]


def test_linear_axes_take_the_default_dynamics_and_c_does_not(run, pass_raw):
    """gantry_defaults.damping/friction are N*s/m and N: the rotary C axis never takes them."""
    code, doc, _, _ = run()
    assert code == 0, doc["errors"] + doc["missing"]
    limits = doc["joint_limits"]
    assert limits["gantry_2_x"]["damping"] == pass_raw["gantry_defaults"]["damping"]
    assert limits["gantry_2_z"]["friction"] == pass_raw["gantry_defaults"]["friction"]
    assert "damping" not in limits["gantry_2_c"]

    own = copy.deepcopy(pass_raw)
    own["gantries"]["gantry_2"]["axes"]["x"]["damping"] = 9.0
    code, doc, _, _ = run(own)
    assert doc["joint_limits"]["gantry_2_x"]["damping"] == 9.0


def test_a_head_controller_is_named_for_the_head_and_drives_its_stroke(run):
    code, doc, config, _ = run()
    assert code == 0, doc["errors"] + doc["missing"]
    assert joints_of(config, "gh_top_controller") == ["gh_top_stroke"]


def test_an_opposed_head_gets_a_controller_over_both_banks(run, pass_raw):
    """Two banks closing on the work are two joints, and one controller commands them together."""
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"].update(direction="horizontal", opposed=True, pattern_from=None)
    code, doc, config, _ = run(raw)
    assert code == 0, doc["errors"] + doc["missing"]
    assert joints_of(config, "gh_top_controller") == ["gh_top_stroke_1", "gh_top_stroke_2"]
    assert doc["joint_limits"]["gh_top_stroke_2"]["max"] == raw["gang_heads"]["gh_top"]["stroke"]


def test_tolerances_follow_the_joint_type(run):
    """D-002's tolerances are metres on a prismatic axis; the yaw axis takes radians."""
    code, doc, config, _ = run()
    assert code == 0, doc["errors"] + doc["missing"]
    constraints = config["gantry_1_controller"]["ros__parameters"]["constraints"]
    assert constraints["gantry_1_x"] == gc.TOLERANCE["prismatic"]
    assert constraints["gantry_1_c"] == gc.TOLERANCE["revolute"]
    assert constraints["goal_time"] == gc.GOAL_TIME_S


# ---------------------------------------------------------------- what it refuses

def test_a_null_limit_fails_naming_the_spec_key(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gantries"]["gantry_2"]["axes"]["y"]["v_max"] = None
    code, doc, config, out = run(raw)
    assert code == 1
    assert doc["missing"] == ["gantries.gantry_2.axes.y.v_max"]
    assert not out.exists(), "a generator that fails writes nothing"


def test_a_null_head_stroke_fails_naming_the_spec_key(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["stroke"] = None
    code, doc, _, _ = run(raw)
    assert code == 1
    assert "gang_heads.gh_top.stroke" in doc["missing"]


def test_an_opposed_head_that_is_not_horizontal_is_refused(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["opposed"] = True      # direction is still `down`
    code, doc, _, _ = run(raw)
    assert code == 1
    assert any("opposed is only a horizontal head" in e for e in doc["errors"])


def test_an_opposed_head_with_an_odd_spindle_count_is_refused(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"].update(direction="horizontal", opposed=True, spindles=7,
                                       pattern_from=None)
    code, doc, _, _ = run(raw)
    assert code == 1
    assert any("cannot be split between two opposed banks" in e for e in doc["errors"])


def test_a_head_no_step_drives_is_refused(run, pass_raw):
    """The head's station is where line.urdf.xacro mounts it, and the sequence is what says so."""
    raw = copy.deepcopy(pass_raw)
    next(s for s in raw["sequence"] if s.get("head") == "gh_top")["head"] = None
    code, doc, _, _ = run(raw)
    assert code == 1
    assert any("no single station drives it" in e for e in doc["errors"])


def test_a_pattern_that_does_not_match_the_spindle_count_is_refused(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["spindles"] = 6        # the pattern still has 8 holes
    code, doc, _, _ = run(raw)
    assert code == 1
    assert any("positions has 8 holes" in e for e in doc["errors"])


# ---------------------------------------------------------------- the dev overlay

def test_a_dev_run_is_marked_as_one_and_lists_its_placeholders(run, pass_raw):
    """No deliverable number may come from the overlay (CLAUDE.md P0), so a dev run says which
    values did, writes its own report and its own file."""
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["v_max"] = None
    code, doc, config, out = run(raw, "--dev",
                                 overlay={"gang_heads": {"gh_top": {"v_max": 0.25}}})
    assert code == 0, doc["errors"] + doc["missing"]
    assert doc["dev"] is True
    assert doc["placeholders"] == ["gang_heads.gh_top.v_max"]
    assert doc["joint_limits"]["gh_top_stroke"]["v_max"] == 0.25
    assert out.name == "controllers.dev.yaml"
