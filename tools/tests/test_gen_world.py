"""P5.1: the main-line world, laid out from the spec.

The load-bearing assertions are `test_every_include_is_at_its_spec_pose` — the world exists to put
equipment where the spec says it is — and `test_an_include_with_no_model_fails_before_the_world_is
_written`: gz resolves an include when it parses the world, so a world naming a model that is not
there does not load at all, and that has to be caught here rather than in the smoke check.
"""

import copy
import json
import xml.etree.ElementTree as ET

import pytest
import yaml

import gen_world
import spec as spec_mod

TOL = 1e-6
PLUGINS = ["Physics", "UserCommands", "SceneBroadcaster", "Contact"]


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    monkeypatch.setattr(gen_world, "report_path",
                        lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json")


@pytest.fixture
def models(tmp_path, pass_raw):
    """The models tree gen_models.py would have written: gen_world only needs the directories.

    Both trees, because a dev run reads `models/dev/` and a real one `models/`.
    """
    root = tmp_path / "models"
    for base in (root, root / "dev"):
        for ident in list(pass_raw["parts"]) + ["fd_mid_plate"]:
            (base / ident).mkdir(parents=True, exist_ok=True)
    return root


@pytest.fixture
def run(tmp_path, pass_raw, models):
    def go(raw=None, *extra):
        spec = tmp_path / "line_spec.yaml"
        spec.write_text(yaml.safe_dump(pass_raw if raw is None else raw))
        out = tmp_path / "worlds" / f"main_line{'.dev' if '--dev' in extra else ''}.sdf"
        code = gen_world.main(["--spec", str(spec), "--models", str(models), "--out", str(out),
                               *extra])
        name = f"gen_world{'.dev' if '--dev' in extra else ''}.json"
        return code, json.loads((tmp_path / name).read_text()), out
    return go


def world_of(path):
    return ET.fromstring(path.read_text()).find("world")


def pose_of(element):
    return [float(c) for c in element.find("pose").text.split()[:3]]


# ---------------------------------------------------------------- the world itself

def test_the_header_says_it_is_generated(run):
    code, doc, out = run()
    assert code == 0, doc["errors"]
    assert "GENERATED from line_spec.yaml by gen_world.py — do not edit" in out.read_text()


def test_the_world_carries_the_four_plugins_the_line_runs_on(run):
    """Physics, SceneBroadcaster and UserCommands drive it; Contact lets a contact sensor exist."""
    code, doc, out = run()
    assert code == 0, doc["errors"]
    names = [p.get("name") for p in world_of(out).findall("plugin")]
    assert [n.rsplit("::", 1)[-1] for n in names] == PLUGINS


# ---------------------------------------------------------------- poses come from the spec

def test_every_include_is_at_its_spec_pose(run, pass_raw):
    code, doc, out = run()
    assert code == 0, doc["errors"]
    includes = {i.find("name").text: i for i in world_of(out).findall("include")}
    assert pose_of(includes["Feeder_MidPlate"]) == pytest.approx(
        pass_raw["feeders"]["fd_mid_plate"]["pose"][:3], abs=TOL)
    assert includes["Feeder_MidPlate"].find("uri").text == "model://fd_mid_plate"
    # The pallet stands at station 1 — the station with the lowest index, not the first in yaml.
    assert pose_of(includes["Pallet_1"]) == pytest.approx(
        pass_raw["stations"]["st_load"]["pose"][:3], abs=TOL)


def test_every_station_and_buffer_is_at_its_spec_pose(run, pass_raw):
    code, doc, out = run()
    assert code == 0, doc["errors"]
    models = {m.get("name"): m for m in world_of(out).findall("model")}
    assert pose_of(models["Station_Drive"]) == pytest.approx(
        pass_raw["stations"]["st_drive"]["pose"][:3], abs=TOL)
    assert pose_of(models["Buffer_Plate"]) == pytest.approx(
        pass_raw["buffers"]["buf_plate"]["pose"][:3], abs=TOL)
    assert doc["models"]["Station_Drive"]["spec_path"] == "stations.st_drive.pose"


def test_a_station_with_cells_gets_one_enclosure_per_cell_at_cell_pitch(run, pass_raw):
    """`count` parallel copies of a station stand beside the line, `cell_pitch` apart (§15)."""
    code, doc, out = run()
    assert code == 0, doc["errors"]
    station = pass_raw["stations"]["st_test"]
    cells = [m for m in world_of(out).findall("model") if m.get("name").startswith("Station_Test_")]
    assert len(cells) == station["count"]
    for k, cell in enumerate(sorted(cells, key=lambda m: m.get("name"))):
        expected = list(station["pose"][:3])
        expected[gen_world.CELL_AXIS] += k * station["cell_pitch"]
        assert cell.get("name") == f"Station_Test_Cell{k + 1}"
        assert pose_of(cell) == pytest.approx(expected, abs=TOL)


def test_the_cells_move_when_cell_pitch_does(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["stations"]["st_test"]["cell_pitch"] = 0.8
    code, doc, out = run(raw)
    assert code == 0, doc["errors"]
    axis = gen_world.CELL_AXIS
    poses = [doc["models"][f"Station_Test_Cell{k}"]["pose"][axis] for k in (1, 2)]
    assert poses[1] - poses[0] == pytest.approx(0.8, abs=TOL)


def test_a_station_with_cells_but_no_cell_pitch_fails_naming_it(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["stations"]["st_test"]["cell_pitch"] = None
    code, doc, _ = run(raw)
    assert code == 1
    assert "stations.st_test.cell_pitch" in doc["missing"]


# ---------------------------------------------------------------- what the world must not do

def test_an_include_with_no_model_fails_before_the_world_is_written(run, models, tmp_path):
    """gz resolves an include at parse time: a missing model is a world that will not load."""
    import shutil
    shutil.rmtree(models / "fd_mid_plate")
    code, doc, out = run()
    assert code == 1
    assert any("Feeder_MidPlate" in e and "gen_models.py" in e for e in doc["errors"])
    assert any("magazine" in e for e in doc["errors"])
    assert not out.exists()


def test_a_feeder_with_no_model_is_an_envelope_instead_of_an_include(run, pass_raw, models):
    import shutil
    shutil.rmtree(models / "fd_mid_plate")
    raw = copy.deepcopy(pass_raw)
    raw["feeders"]["fd_mid_plate"]["type"] = "step"          # a step feeder has no model
    code, doc, out = run(raw)
    assert code == 0, doc["errors"]
    assert doc["models"]["Feeder_MidPlate"]["kind"] == "feeder"
    assert pose_of({m.get("name"): m for m in world_of(out).findall("model")}["Feeder_MidPlate"]) \
        == pytest.approx(raw["feeders"]["fd_mid_plate"]["pose"][:3], abs=TOL)


def test_a_feeder_with_no_pose_is_skipped_and_said_so(run, pass_raw):
    """A blow feeder rides its gang head (§16) — there is no floor position to place."""
    code, doc, _ = run()
    assert code == 0, doc["errors"]
    assert any("fd_screw" in s for s in doc["skipped"])
    assert "Feeder_Screw" not in doc["models"]


def test_two_models_with_the_same_name_fail(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["buffers"]["buf_plate2"] = dict(raw["buffers"]["buf_plate"])
    raw["sources"]["src_plate2"] = {"output": "bottom_plate", "buffer": "buf_plate2",
                                    "interval_s": 20.0, "buffer_capacity": 3}
    raw["buffers"]["buf_plate2"]["pose"] = [0.9, 0.3, 0.1, 0.0]
    code, doc, _ = run(raw)
    assert code == 0, doc["errors"]                 # different names: fine
    raw["stations"]["st_load"]["index"] = 1
    raw["buffers"]["buf_Plate"] = raw["buffers"].pop("buf_plate2")   # collides after PascalCase
    raw["sources"]["src_plate2"]["buffer"] = "buf_Plate"
    code, doc, _ = run(raw)
    assert code == 1
    assert any("Buffer_Plate" in e for e in doc["errors"])


def test_the_conveyor_ends_where_the_pallet_starts(run, tmp_path):
    """A pallet stands *on* the conveyor: its body hangs below the station pose it is placed at."""
    (tmp_path / "cad_pallet.json").write_text(json.dumps(
        {"bbox_mm": {"min": [0.0, 0.0, -12.0], "max": [200.0, 200.0, 68.0],
                     "size": [200.0, 200.0, 80.0]}}))
    code, doc, out = run()
    assert code == 0, doc["errors"]
    conveyor = {m.get("name"): m for m in world_of(out).findall("model")}["Conveyor"]
    box_z = float(conveyor.find(".//collision/pose").text.split()[2])
    height = float(conveyor.find(".//collision/geometry/box/size").text.split()[2])
    assert box_z + height / 2.0 == pytest.approx(-0.012, abs=TOL)     # top face, 12 mm below


# ---------------------------------------------------------------- CLI

@pytest.mark.parametrize("key, blank", [
    ("stations.st_drive.pose[0]",
     lambda raw: raw["stations"]["st_drive"]["pose"].__setitem__(0, None)),
    ("buffers.buf_plate.pose[2]",
     lambda raw: raw["buffers"]["buf_plate"]["pose"].__setitem__(2, None)),
    ("gantry.mount[1]", lambda raw: raw["gantry"]["mount"].__setitem__(1, None))])
def test_null_spec_value_fails_and_names_the_key(run, pass_raw, key, blank, capsys):
    raw = copy.deepcopy(pass_raw)
    blank(raw)
    code, doc, out = run(raw)
    assert code == 1
    assert key in doc["missing"]
    assert key in capsys.readouterr().err
    assert not out.exists()


def test_dev_run_is_quarantined_in_its_own_file(tmp_path, pass_raw, run):
    raw = copy.deepcopy(pass_raw)
    raw["buffers"]["buf_plate"]["pose"] = [None, 0.3, 0.1, 0.0]
    (tmp_path / "line_spec.dev.yaml").write_text(
        yaml.safe_dump({"buffers": {"buf_plate": {"pose": [0.5, None, None, None]}}}))
    code, doc, out = run(raw, "--dev")
    assert code == 0, doc["errors"]
    assert out.name == "main_line.dev.sdf" and out.is_file()
    assert doc["dev"] is True and doc["placeholders"] == ["buffers.buf_plate.pose[0]"]
    assert not (tmp_path / "gen_world.json").exists()
