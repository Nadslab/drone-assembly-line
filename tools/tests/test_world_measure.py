"""P5.4 check: the running world is compared against the spec, not against itself.

tools/checks/world_smoke.sh owns the gz server; everything decided about a pose lives in
world_measure.py, so that is what is tested here — with the gz calls stubbed, so the suite stays
headless and fast. `test_a_model_at_the_wrong_pose_fails` is the load-bearing one: a check that
cannot fail is not a check.

`test_pose_v_is_parsed_with_protobuf_defaults` pins the other risk. One `gz topic` read answers
for every model in the world, and text protobuf omits zero-valued fields — so a pose of exactly
0 arrives as a *missing* line, and a parser that does not know that would silently read the
previous model's coordinate.
"""

import json

import pytest
import yaml

import spec as spec_mod
from checks import world_measure

POSE_V = """header {
  stamp {
    sec: 1
    nsec: 861000000
  }
}
pose {
  name: "Ground"
  id: 4
  position {
  }
  orientation {
    w: 1
  }
}
pose {
  name: "Station_Load"
  id: 8
  position {
    z: 0.1
  }
  orientation {
    w: 1
  }
}
pose {
  name: "Station_Load_link"
  id: 9
  position {
    z: -0.032
  }
  orientation {
    w: 1
  }
}
pose {
  name: "Feeder_MidPlate"
  id: 12
  position {
    x: 0.2
    y: 0.3
    z: 0.15
  }
  orientation {
    w: 1
  }
}
"""


def test_pose_v_is_parsed_with_protobuf_defaults():
    poses = world_measure.parse_pose_v(POSE_V)
    assert poses["Ground"] == [0.0, 0.0, 0.0]              # `position {}` is three zeros
    assert poses["Station_Load"] == [0.0, 0.0, 0.1]        # x and y omitted, not inherited
    assert poses["Feeder_MidPlate"] == [0.2, 0.3, 0.15]
    assert poses["Station_Load_link"] == [0.0, 0.0, -0.032]


def test_the_orientation_block_is_not_mistaken_for_a_position():
    """Both blocks have x, y and z fields; only one of them is where the model is."""
    text = POSE_V.replace("""  orientation {
    w: 1
  }
}
pose {
  name: "Station_Load_link\"""", """  orientation {
    x: 0.7
    y: 0.7
    z: 0.7
    w: 1
  }
}
pose {
  name: "Station_Load_link\"""")
    assert world_measure.parse_pose_v(text)["Station_Load"] == [0.0, 0.0, 0.1]


# ---------------------------------------------------------------- the comparison

@pytest.fixture
def world(tmp_path, monkeypatch, pass_raw):
    """A stubbed running world: whatever poses the test hands it, no gz involved."""
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)

    def go(live: dict, manifest_models: dict, tol_mm=1.0):
        monkeypatch.setattr(world_measure, "model_list", lambda t: list(live))
        monkeypatch.setattr(world_measure, "world_poses",
                            lambda w, names, t: {n: live.get(n) for n in names})
        return world_measure.check(pass_raw, {"models": manifest_models}, 1.0, tol_mm)
    return go


def manifest(name="Station_Load", spec_path="stations.st_load.pose", pose=(0.0, 0.0, 0.1)):
    return {name: {"kind": "station", "spec_path": spec_path, "pose": list(pose)}}


def test_a_model_at_its_spec_pose_passes(world, pass_raw):
    doc = world({"Station_Load": [0.0, 0.0, 0.1]}, manifest())
    assert doc["errors"] == []
    assert doc["models"]["Station_Load"]["status"] == "PASS"
    assert doc["models"]["Station_Load"]["from_"] == "stations.st_load.pose"


def test_a_model_at_the_wrong_pose_fails(world):
    doc = world({"Station_Load": [0.0, 0.0, 0.1042]}, manifest())
    assert doc["models"]["Station_Load"]["status"] == "FAIL"
    assert doc["models"]["Station_Load"]["error_mm"] == pytest.approx(4.2, abs=0.01)
    assert any("Station_Load" in e for e in doc["errors"])


def test_the_tolerance_is_the_one_it_is_given(world):
    live = {"Station_Load": [0.0, 0.0, 0.1008]}          # 0.8 mm out
    assert world(live, manifest(), tol_mm=1.0)["errors"] == []
    assert world(live, manifest(), tol_mm=0.5)["errors"] != []


def test_it_compares_against_the_spec_not_against_the_world_file(world):
    """A world that placed a model somewhere its own manifest agrees with is still wrong."""
    doc = world({"Station_Load": [0.0, 0.0, 0.5]},
                manifest(pose=(0.0, 0.0, 0.5)))          # manifest and world agree with each other
    assert doc["models"]["Station_Load"]["status"] == "FAIL"
    assert doc["models"]["Station_Load"]["expected_m"] == [0.0, 0.0, 0.1]      # the spec's


def test_a_cell_is_checked_at_its_station_pose_plus_cell_pitch(world, pass_raw):
    station = pass_raw["stations"]["st_test"]
    expected = list(station["pose"][:3])
    expected[world_measure.CELL_AXIS] += 2 * station["cell_pitch"]
    doc = world({"Station_Test_Cell3": expected},
                manifest("Station_Test_Cell3", "stations.st_test.pose", (0, 0, 0)))
    assert doc["models"]["Station_Test_Cell3"]["status"] == "PASS"
    assert "cell_pitch" in doc["models"]["Station_Test_Cell3"]["from_"]


def test_a_cell_at_the_wrong_pitch_fails(world, pass_raw):
    station = pass_raw["stations"]["st_test"]
    wrong = list(station["pose"][:3])
    wrong[world_measure.CELL_AXIS] += 2 * station["cell_pitch"] + 0.01
    doc = world({"Station_Test_Cell3": wrong},
                manifest("Station_Test_Cell3", "stations.st_test.pose", (0, 0, 0)))
    assert doc["models"]["Station_Test_Cell3"]["status"] == "FAIL"


def test_a_derived_model_falls_back_to_the_manifest_pose(world):
    """The frame and the conveyor have no pose of their own in the spec; they are still checked."""
    doc = world({"Frame": [1.0, 0.0, 0.0]}, {"Frame": {"kind": "frame", "spec_path": None,
                                                       "pose": [1.0, 0.0, 0.0]}})
    assert doc["models"]["Frame"]["status"] == "PASS"
    assert "manifest" in doc["models"]["Frame"]["from_"]


def test_a_model_that_did_not_load_fails(world):
    """gz resolves includes at parse time, so a model that is not listed never loaded."""
    doc = world({}, manifest())
    assert doc["missing_models"] == ["Station_Load"]
    assert any("did not load" in e for e in doc["errors"])


def test_a_model_the_manifest_does_not_know_about_is_reported(world):
    doc = world({"Station_Load": [0.0, 0.0, 0.1], "Ground": [0, 0, 0], "Stray": [1, 1, 1]},
                manifest())
    assert doc["extra_models"] == ["Stray"]              # Ground is the world's own
    assert doc["errors"] == []                           # worth saying, not a failure


# ---------------------------------------------------------------- CLI

def test_a_missing_manifest_fails_pointing_at_gen_world(tmp_path, monkeypatch, pass_raw):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    spec = tmp_path / "line_spec.yaml"
    spec.write_text(yaml.safe_dump(pass_raw))
    report = tmp_path / "world.json"
    code = world_measure.main(["--spec", str(spec), "--manifest", str(tmp_path / "nope.json"),
                               "--report", str(report)])
    assert code == 1
    doc = json.loads(report.read_text())
    assert doc["passed"] is False
    assert any("gen_world.py" in e for e in doc["errors"])
