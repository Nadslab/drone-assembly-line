"""P4 B5: one magazine per magazine feeder, built from the part it holds.

The load-bearing assertion is `test_the_key_stops_a_mirrored_part_and_lets_a_correct_one_through`:
the poka-yoke key (§7.5) is checked as volume the part and the rib both want, for a part seated
both ways round, rather than by trusting that a rib was drawn somewhere. The other one is
`test_the_pick_plane_is_the_rim`: the whole magazine exists to present a part at one fixed
coordinate (§16), so nothing of it may stand above that plane.
"""

import copy
import json

import pytest
import yaml

import spec as spec_mod
from cad import magazine

TOL_MM = 0.01


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    monkeypatch.setattr(magazine, "report_path",
                        lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json")


def run_raw(tmp_path, raw, *extra):
    """Run the generator over `raw` and return (exit code, report)."""
    spec = tmp_path / "line_spec.yaml"
    spec.write_text(yaml.safe_dump(raw))
    code = magazine.main(["--spec", str(spec), "--out", str(tmp_path / "out"), *extra])
    name = f"cad_magazines{'.dev' if '--dev' in extra else ''}.json"
    return code, json.loads((tmp_path / name).read_text())


@pytest.fixture
def run(tmp_path, pass_raw):
    def go(raw=None, *extra):
        return run_raw(tmp_path, pass_raw if raw is None else raw, *extra)
    return go


def keyed(raw, key=(0.008, 0.008)):
    """The fixture's magazine part, given a poka-yoke key."""
    out = copy.deepcopy(raw)
    out["parts"]["mid_plate"]["key"] = list(key)
    return out


# ---------------------------------------------------------------- what gets built

def test_one_magazine_per_magazine_feeder_and_nothing_else(run, pass_raw):
    """The spec decides which magazines exist: feeders of another type are not magazines."""
    raw = copy.deepcopy(pass_raw)
    raw["feeders"]["fd_top_plate"] = {"type": "magazine", "part": "top_plate",
                                      "pose": [0.4, 0.3, 0.15, 0.0], "capacity": 10,
                                      "refill_s": 60.0}
    raw["feeders"]["fd_standoff"] = {"type": "step", "part": "mid_plate",
                                     "pose": [0.6, 0.3, 0.1, 0.0], "capacity": 200,
                                     "refill_s": 300.0}
    code, doc = run(raw)
    assert code == 0, doc["errors"]
    assert sorted(doc["items"]) == ["fd_mid_plate", "fd_top_plate"]


def test_the_tube_is_the_part_plus_a_fit_clearance(run, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    d = doc["items"]["fd_mid_plate"]["derived"]
    env = pass_raw["parts"]["mid_plate"]["envelope"]
    assert d["pocket_mm"] == pytest.approx([env[i] * 1000.0 + 2.0 * magazine.FIT for i in (0, 1)],
                                           abs=TOL_MM)
    assert d["part_thickness_mm"] == pytest.approx(env[2] * 1000.0, abs=TOL_MM)


def test_capacity_is_what_makes_the_magazine_deep(run, pass_raw):
    """`feeders.<id>.capacity` is the spec's own number for how many parts it holds."""
    raw = copy.deepcopy(pass_raw)
    raw["feeders"]["fd_mid_plate"]["capacity"] = 5
    raw["feeders"]["fd_mid_plate"]["pose"][2] = 0.15
    code, doc = run(raw)
    assert code == 0, doc["errors"]
    d = doc["items"]["fd_mid_plate"]["derived"]
    thick = pass_raw["parts"]["mid_plate"]["envelope"][2] * 1000.0
    assert d["stack"]["depth_mm"] == pytest.approx(5 * (thick + magazine.SEPARATION), abs=TOL_MM)
    assert d["follower"]["travel_mm"] == pytest.approx(d["stack"]["depth_mm"], abs=TOL_MM)


def test_the_pick_plane_is_the_rim(run):
    """The gantry goes to `feeders.<id>.pose` every cycle (§16): nothing may stand above it."""
    code, doc = run()
    assert code == 0, doc["errors"]
    item = doc["items"]["fd_mid_plate"]
    assert item["derived"]["rim_z_mm"] == 0.0
    assert item["bbox_mm"]["max"][2] == pytest.approx(0.0, abs=TOL_MM)
    assert item["bbox_mm"]["min"][2] == pytest.approx(item["derived"]["base_z_mm"], abs=TOL_MM)


# ---------------------------------------------------------------- the poka-yoke key

def test_the_key_stops_a_mirrored_part_and_lets_a_correct_one_through(run, pass_raw):
    """§7.5: the key is only a key if the part cannot go in backwards.

    Checked as the volume the seated part and the rib both want — zero the right way round, and
    the mirrored corner's worth of material the wrong way round.
    """
    code, doc = run(keyed(pass_raw))
    assert code == 0, doc["errors"]
    key = doc["items"]["fd_mid_plate"]["derived"]["key"]
    thick = pass_raw["parts"]["mid_plate"]["envelope"][2] * 1000.0
    assert key["corner"] == "+x+y"
    assert key["seated_clash_mm3"] == 0.0
    assert key["flipped_clash_mm3"] == pytest.approx(
        (8.0 - magazine.KEY_CLEAR) ** 2 * thick, rel=1e-3)


def test_a_part_with_no_key_warns_rather_than_stopping_the_build(run):
    """Not every part needs one — a plate that is symmetric does not — but it is worth saying."""
    code, doc = run()
    assert code == 0, doc["errors"]
    assert doc["items"]["fd_mid_plate"]["derived"]["key"] is None
    assert any("parts.mid_plate.key" in w and "mirror image" in w for w in doc["warnings"])


def test_a_key_too_small_to_stop_anything_fails(run, pass_raw):
    code, doc = run(keyed(pass_raw, key=(0.001, 0.001)))
    assert code == 1
    assert "parts.mid_plate.key" in doc["errors"][0]


def test_the_key_rib_tracks_the_part_it_keys(run, pass_raw):
    """Change the part's key and the rib follows it — the rib is not a number of its own."""
    clashes = []
    for size in (0.006, 0.010):
        code, doc = run(keyed(pass_raw, key=(size, size)))
        assert code == 0, doc["errors"]
        clashes.append(doc["items"]["fd_mid_plate"]["derived"]["key"]["flipped_clash_mm3"])
    assert clashes[1] > clashes[0]


# ---------------------------------------------------------------- what the magazine must clear

def test_a_magazine_that_would_stand_below_the_floor_fails_naming_the_pose(run, pass_raw):
    """A pose that cannot hold the stack the spec asks for is the spec contradicting itself."""
    raw = copy.deepcopy(pass_raw)
    raw["feeders"]["fd_mid_plate"]["pose"] = [0.2, 0.3, 0.02, 0.0]      # 20 mm off the floor
    code, doc = run(raw)
    assert code == 1
    assert "feeders.fd_mid_plate.pose[2]" in doc["errors"][0]


def test_a_placeholder_pose_only_warns(tmp_path, pass_raw):
    """The overlay is never a deliverable number (CLAUDE.md): it has nothing to be wrong about."""
    raw = copy.deepcopy(pass_raw)
    raw["feeders"]["fd_mid_plate"]["pose"] = [0.2, 0.3, None, 0.0]
    (tmp_path / "line_spec.dev.yaml").write_text(
        yaml.safe_dump({"feeders": {"fd_mid_plate": {"pose": [None, None, 0.02, None]}}}))
    code, doc = run_raw(tmp_path, raw, "--dev")
    assert code == 0, doc["errors"]
    assert any("feeders.fd_mid_plate.pose[2]" in w for w in doc["warnings"])


def test_a_tool_that_has_to_reach_into_the_pocket_fails(run, pass_raw):
    """A finger tool comes down past the rim; the pocket has to be wider than its jaws."""
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["mid_plate"]["envelope"] = [0.05, 0.03, 0.003]      # narrower than the jaws
    raw["sequence"][1]["tool"] = "tool_carrier"
    code, doc = run(raw)
    assert code == 1
    assert "tool_carrier" in doc["errors"][0] and "pocket" in doc["errors"][0]


# ---------------------------------------------------------------- CLI

def test_writes_step_stl_and_report_per_feeder(run, tmp_path):
    code, doc = run()
    assert code == 0, doc["errors"]
    step = tmp_path / "out" / "magazine_fd_mid_plate.step"
    stl = tmp_path / "out" / "magazine_fd_mid_plate.stl"
    assert step.is_file() and stl.is_file()
    item = doc["items"]["fd_mid_plate"]
    assert doc["passed"] is True and doc["errors"] == [] and item["status"] == "PASS"
    assert item["outputs"] == {"step": str(step), "stl": str(stl)}
    assert item["mass_kg"]["total"] > 0.0
    assert stl.stat().st_size < 5 * 1024 * 1024                  # P0: no STL over 5 MB
    assert item["mesh"]["solid_bbox_drift_mm"] <= 0.2


def test_the_material_note_travels_in_the_step_header(run, tmp_path):
    assert run()[0] == 0
    header = (tmp_path / "out" / "magazine_fd_mid_plate.step").read_text()[:2000]
    assert magazine.MATERIAL.split(":")[0] in header.replace("\n", " ")


@pytest.mark.parametrize("key, blank", [
    ("parts.mid_plate.envelope[2]",
     lambda raw: raw["parts"]["mid_plate"]["envelope"].__setitem__(2, None)),
    ("feeders.fd_mid_plate.capacity",
     lambda raw: raw["feeders"]["fd_mid_plate"].__setitem__("capacity", None)),
    ("feeders.fd_mid_plate.pose[2]",
     lambda raw: raw["feeders"]["fd_mid_plate"]["pose"].__setitem__(2, None))])
def test_null_spec_value_fails_and_names_the_key(run, tmp_path, pass_raw, key, blank, capsys):
    raw = copy.deepcopy(pass_raw)
    blank(raw)
    code, doc = run(raw)
    assert code == 1
    assert any(key in m for m in doc["missing"])
    assert key in capsys.readouterr().err
    assert not (tmp_path / "out").exists()


def test_dev_run_is_quarantined_in_its_own_folder(tmp_path, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["feeders"]["fd_mid_plate"]["capacity"] = None
    (tmp_path / "line_spec.dev.yaml").write_text(
        yaml.safe_dump({"feeders": {"fd_mid_plate": {"capacity": 10}}}))
    code, doc = run_raw(tmp_path, raw, "--dev")
    assert code == 0, doc["errors"]
    assert (tmp_path / "out" / "dev" / "magazine_fd_mid_plate.step").is_file()
    assert not (tmp_path / "out" / "magazine_fd_mid_plate.step").exists()
    assert doc["dev"] is True and doc["placeholders"] == ["feeders.fd_mid_plate.capacity"]
    assert not (tmp_path / "cad_magazines.json").exists()


def test_a_spec_with_no_magazine_feeder_builds_nothing_and_passes(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["feeders"]["fd_mid_plate"]["type"] = "step"
    code, doc = run(raw)
    assert code == 0 and doc["items"] == {}
