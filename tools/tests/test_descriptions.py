"""P5.3 check: the descriptions are compared against the spec, not against themselves.

tools/checks/descriptions.sh owns `xacro` and `check_urdf` (ROS tools, and a suite that shelled
out to them would need a sourced workspace); everything *decided* about a limit lives in
descriptions_check.py, so that is what is tested here, against hand-written URDF.

`test_a_joint_whose_velocity_is_not_the_specs_fails` and `test_a_spindle_moved_off_the_pattern_
fails` are the load-bearing ones: a check that cannot fail is not a check. The rest pins what
"top-level description" means, because that decides which files get expanded at all.
"""

import xml.etree.ElementTree as ET

import pytest
import yaml

import gen_controllers as gc
from checks import descriptions_check as dc

# One prismatic axis, exactly as urdf/gantry.urdf.xacro writes it for the fixture spec.
AXIS = """<joint name="gantry_1_y" type="prismatic">
  <parent link="Gantry_1_x"/><child link="Gantry_1_y"/>
  <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
  <limit effort="62.0" lower="-0.5" upper="0.5" velocity="1.0"/>
  <dynamics damping="5.0" friction="0.5"/>
</joint>"""

WANT = {"type": "prismatic", "spec_path": "gantries.gantry_1.axes.y", "min": -0.5, "max": 0.5,
        "v_max": 1.0, "a_max": 2.0, "damping": 5.0, "friction": 0.5}


def joint(xml=AXIS, **replace):
    element = ET.fromstring(xml)
    limit = element.find("limit")
    for key, value in replace.items():
        limit.set(key, str(value))
    return element


# ---------------------------------------------------------------- one joint against the spec

def test_a_joint_that_says_what_the_spec_says_passes():
    row = dc.compare_joint("gantry_1_y", WANT, joint())
    assert row["ok"], row["errors"]
    assert row["actual"]["effort"] == 62.0        # derived, so recorded rather than asserted


def test_a_joint_whose_velocity_is_not_the_specs_fails():
    row = dc.compare_joint("gantry_1_y", WANT, joint(velocity=0.42))
    assert not row["ok"]
    assert row["errors"] == ["gantry_1_y: velocity=0.42, spec says 1.0 "
                             "(gantries.gantry_1.axes.y)"]


@pytest.mark.parametrize("key, value", [("lower", -0.4), ("upper", 0.6), ("damping", 1.0),
                                        ("friction", 0.0)])
def test_every_limit_the_spec_gives_is_compared(key, value):
    element = ET.fromstring(AXIS)
    target = element.find("limit" if key in ("lower", "upper") else "dynamics")
    target.set(key, str(value))
    row = dc.compare_joint("gantry_1_y", WANT, element)
    assert not row["ok"]
    assert any(f"{key}={value}" in e for e in row["errors"])


def test_a_joint_the_description_does_not_have_fails():
    row = dc.compare_joint("gantry_1_c", dict(WANT, spec_path="gantries.gantry_1.axes.c"), None)
    assert not row["ok"]
    assert row["errors"] == ["gantry_1_c: the description has no such joint"]


def test_a_joint_with_no_effort_fails():
    """A zero-effort joint parses and then cannot hold its own weight up."""
    row = dc.compare_joint("gantry_1_y", WANT, joint(effort=0.0))
    assert not row["ok"]
    assert any("expected a positive force" in e for e in row["errors"])


def test_a_prismatic_joint_where_the_spec_wants_a_revolute_one_fails():
    element = joint()
    element.set("type", "revolute")
    row = dc.compare_joint("gantry_1_y", WANT, element)
    assert not row["ok"]
    assert any("spec says prismatic" in e for e in row["errors"])


# ---------------------------------------------------------------- the spindle frames

def spindle_joints(positions, name="GangHead_Top"):
    """The fixed joints urdf/gang_head.urdf.xacro writes, one per spindle."""
    joints = {}
    for i, (x, y) in enumerate(positions, start=1):
        joints[f"{name}_spindle_{i}_frame"] = ET.fromstring(
            f'<joint name="{name}_spindle_{i}_frame" type="fixed">'
            f'<origin xyz="{x} {y} 0" rpy="0 3.14159265 0"/></joint>')
    return joints


def test_a_spindle_per_hole_in_the_pattern_passes(pass_raw):
    positions = pass_raw["parts"]["bottom_plate"]["arm_holes"]["positions"]
    row = dc.compare_spindles(pass_raw, "gh_top", spindle_joints(positions))
    assert row["ok"], row["errors"]
    assert row["expected"] == row["actual"] == 8
    assert row["pattern_from"] == "bottom_plate.arm_holes"


def test_a_spindle_moved_off_the_pattern_fails(pass_raw):
    """The pattern is the plate's own screw positions: a head that drives 1 mm off misses."""
    positions = [list(p) for p in pass_raw["parts"]["bottom_plate"]["arm_holes"]["positions"]]
    positions[2][0] += 0.001
    row = dc.compare_spindles(pass_raw, "gh_top", spindle_joints(positions))
    assert not row["ok"]
    assert any("spindle 3 at" in e for e in row["errors"])


def test_too_few_spindle_frames_fails(pass_raw):
    positions = pass_raw["parts"]["bottom_plate"]["arm_holes"]["positions"][:6]
    row = dc.compare_spindles(pass_raw, "gh_top", spindle_joints(positions))
    assert not row["ok"]
    assert any("6 spindle frame(s), spec says 8" in e for e in row["errors"])


# ---------------------------------------------------------------- where the machines are bolted

def mount(joint_name, xyz):
    return {joint_name: ET.fromstring(f'<joint name="{joint_name}" type="fixed">'
                                      f'<origin xyz="{xyz}" rpy="0 0 0"/></joint>')}


GANTRIES = mount("gantry_1_mount", "0 0 0") | mount("gantry_2_mount", "1.9 0 0")


def test_every_gantry_is_bolted_at_its_spec_mount(pass_raw):
    joints = GANTRIES | mount("gh_top_mount", "1.0 0.0 0.16")
    rows = dc.compare_mounts(pass_raw, joints)
    assert all(r["ok"] for r in rows), [e for r in rows for e in r["errors"]]


def test_a_gantry_mounted_somewhere_else_fails(pass_raw):
    joints = GANTRIES | mount("gantry_1_mount", "0.5 0 0") | mount("gh_top_mount", "1.0 0.0 0.16")
    rows = dc.compare_mounts(pass_raw, joints)
    assert [r["what"] for r in rows if not r["ok"]] == ["gantry_1"]
    assert "gantries.gantry_1.mount is [0.0, 0.0, 0.0]" in rows[0]["errors"][0]


def test_a_head_standing_over_the_wrong_station_fails(pass_raw):
    """z is the head's own stand-off, so only x and y are the station's to give."""
    joints = GANTRIES | mount("gh_top_mount", "1.0 0.3 0.16")
    rows = dc.compare_mounts(pass_raw, joints)
    head = next(r for r in rows if r["what"] == "gh_top")
    assert not head["ok"]
    assert "st_drive's pose" in head["errors"][0]


# ---------------------------------------------------------------- the files, and the config

def test_only_a_robot_with_a_name_is_expanded():
    """A macro library is not a description: expanding it alone proves nothing."""
    robots, libraries = dc.top_level_xacros(dc.URDF_DIR)
    assert [p.name for p in robots] == ["line.urdf.xacro", "spike_axis.urdf.xacro"]
    assert sorted(libraries) == ["gang_head.urdf.xacro", "gantry.urdf.xacro",
                                 "inertia.urdf.xacro"]


def test_the_configured_joints_are_read_back_from_the_generated_yaml(tmp_path, pass_raw):
    limits = gc.joint_limits(gc.Values(pass_raw))
    config = tmp_path / "controllers.yaml"
    config.write_text(gc.render(gc.controllers_of(pass_raw), limits))
    assert dc.controller_joints(config) == ["gantry_1_x", "gantry_1_y", "gantry_1_z",
                                            "gantry_1_c", "gantry_2_x", "gantry_2_y",
                                            "gantry_2_z", "gantry_2_c", "gh_top_stroke"]
    # Every joint the description has is configured exactly once: that is the pairing the check
    # makes, and tools/gen_controllers.py is the other half of it.
    assert sorted(dc.controller_joints(config)) == sorted(limits)


def test_the_yaml_a_controller_manager_reads_is_valid_yaml(tmp_path, pass_raw):
    text = gc.render(gc.controllers_of(pass_raw), gc.joint_limits(gc.Values(pass_raw)))
    assert yaml.safe_load(text)["controller_manager"]["ros__parameters"]["update_rate"] == 100
