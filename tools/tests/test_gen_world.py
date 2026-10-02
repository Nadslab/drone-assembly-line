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
        # --minimal always writes to its own fixed, "never a deliverable" names, independent of
        # --dev — see gen_world.main.
        if "--minimal" in extra:
            out = tmp_path / "worlds" / "main_line.minimal.dev.sdf"
            name = "gen_world.minimal.dev.json"
        else:
            out = tmp_path / "worlds" / f"main_line{'.dev' if '--dev' in extra else ''}.sdf"
            name = f"gen_world{'.dev' if '--dev' in extra else ''}.json"
        code = gen_world.main(["--spec", str(spec), "--models", str(models), "--out", str(out),
                               *extra])
        return code, json.loads((tmp_path / name).read_text()), out
    return go


def world_of(path):
    return ET.fromstring(path.read_text()).find("world")


def pose_of(element):
    return [float(c) for c in element.find("pose").text.split()[:3]]


def raised(pose, raw):
    """A spec pose as the world states it: the spec works on the datum plane, the world on the
    floor, and `line.work_height` is the one offset between them."""
    x, y, z = pose[:3]
    return [x, y, z + raw["line"]["work_height"]]


def box_z(model):
    """(min z, max z) of a static primitive in world coordinates."""
    z = pose_of(model)[2] + float(model.find(".//collision/pose").text.split()[2])
    half = float(model.find(".//collision/geometry/box/size").text.split()[2]) / 2.0
    return z - half, z + half


def box_size(model):
    return [float(c) for c in model.find(".//collision/geometry/box/size").text.split()]


def alpha_of(model):
    return float(model.find(".//visual/material/ambient").text.split()[-1])


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
        raised(pass_raw["feeders"]["fd_mid_plate"]["pose"], pass_raw), abs=TOL)
    assert includes["Feeder_MidPlate"].find("uri").text == "model://fd_mid_plate"
    # The pallet stands at station 1 — the station with the lowest index, not the first in yaml.
    assert pose_of(includes["Pallet_1"]) == pytest.approx(
        raised(pass_raw["stations"]["st_load"]["pose"], pass_raw), abs=TOL)


def test_every_station_and_buffer_is_at_its_spec_pose(run, pass_raw):
    code, doc, out = run()
    assert code == 0, doc["errors"]
    models = {m.get("name"): m for m in world_of(out).findall("model")}
    assert pose_of(models["Station_Drive"]) == pytest.approx(
        raised(pass_raw["stations"]["st_drive"]["pose"], pass_raw), abs=TOL)
    assert pose_of(models["Buffer_Plate"]) == pytest.approx(
        raised(pass_raw["buffers"]["buf_plate"]["pose"], pass_raw), abs=TOL)
    assert doc["models"]["Station_Drive"]["spec_path"] == "stations.st_drive.pose"


def test_a_station_with_cells_gets_one_enclosure_per_cell_at_cell_pitch(run, pass_raw):
    """`count` parallel copies of a station stand beside the line, `cell_pitch` apart (§15)."""
    code, doc, out = run()
    assert code == 0, doc["errors"]
    station = pass_raw["stations"]["st_test"]
    cells = [m for m in world_of(out).findall("model") if m.get("name").startswith("Station_Test_")]
    assert len(cells) == station["count"]
    for k, cell in enumerate(sorted(cells, key=lambda m: m.get("name"))):
        expected = raised(station["pose"], pass_raw)
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
        == pytest.approx(raised(raw["feeders"]["fd_mid_plate"]["pose"], raw), abs=TOL)


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


def test_the_conveyor_ends_where_the_pallet_starts(run, pass_raw, tmp_path):
    """A pallet stands *on* the conveyor: its body hangs below the station pose it is placed at."""
    (tmp_path / "cad_pallet.json").write_text(json.dumps(
        {"bbox_mm": {"min": [0.0, 0.0, -12.0], "max": [200.0, 200.0, 68.0],
                     "size": [200.0, 200.0, 80.0]}}))
    code, doc, out = run()
    assert code == 0, doc["errors"]
    conveyor = {m.get("name"): m for m in world_of(out).findall("model")}["Conveyor"]
    top = box_z(conveyor)[1]
    station = raised(pass_raw["stations"]["st_load"]["pose"], pass_raw)
    assert top == pytest.approx(station[2] - 0.012, abs=TOL)      # top face, 12 mm below


def test_the_frame_stands_on_the_floor_and_carries_the_conveyor(run, pass_raw):
    """The line is held at `line.work_height`, so the frame reaches the floor — not a slab
    hanging off the datum plane, which is what put the whole line underground."""
    code, doc, out = run()
    assert code == 0, doc["errors"]
    models = {m.get("name"): m for m in world_of(out).findall("model")}
    frame_bottom, frame_top = box_z(models["Frame"])
    conveyor_bottom, _ = box_z(models["Conveyor"])
    assert frame_bottom == pytest.approx(0.0, abs=TOL)            # on the ground plane
    assert frame_top == pytest.approx(conveyor_bottom, abs=TOL)   # no gap, no overlap


def test_the_frame_reaches_under_the_parallel_cells(run, pass_raw):
    """Cells step away from their station by `cell_pitch`, so the outermost one is further out
    than any station pose: a frame spanning only the poses would not be under it."""
    code, doc, out = run()
    assert code == 0, doc["errors"]
    axis = gen_world.CELL_AXIS
    frame = doc["models"]["Frame"]
    station = pass_raw["stations"]["st_test"]
    far = station["pose"][axis] + (station["count"] - 1) * station["cell_pitch"]
    assert frame["pose"][axis] + frame["size_m"][axis] / 2.0 >= far + gen_world.FRAME_MARGIN


# ---------------------------------------------------------------- Buffer/Feeder support legs

def test_a_floating_buffer_marker_gets_a_leg_to_the_floor(run, pass_raw):
    code, doc, out = run()
    assert code == 0, doc["errors"]
    models = {m.get("name"): m for m in world_of(out).findall("model")}
    assert "Buffer_Plate_Leg" in models
    leg_bottom, leg_top = box_z(models["Buffer_Plate_Leg"])
    marker_bottom, _ = box_z(models["Buffer_Plate"])
    assert leg_bottom == pytest.approx(0.0, abs=TOL)               # world floor
    assert leg_top == pytest.approx(marker_bottom, abs=TOL)        # meets the marker's underside
    size = box_size(models["Buffer_Plate_Leg"])
    assert size[0] == pytest.approx(gen_world.LEG_SECTION[0], abs=TOL)
    assert size[1] == pytest.approx(gen_world.LEG_SECTION[1], abs=TOL)


def test_a_feeder_marker_also_gets_a_leg_but_an_include_does_not(run, pass_raw, models):
    import shutil
    shutil.rmtree(models / "fd_mid_plate")
    raw = copy.deepcopy(pass_raw)
    raw["feeders"]["fd_mid_plate"]["type"] = "step"     # now an envelope, not a model
    code, doc, out = run(raw)
    assert code == 0, doc["errors"]
    models_el = {m.get("name"): m for m in world_of(out).findall("model")}
    assert "Feeder_MidPlate_Leg" in models_el
    includes = {i.find("name").text for i in world_of(out).findall("include")}
    assert "Feeder_MidPlate_Leg" not in includes


def test_legs_are_present_whether_or_not_minimal_is_on(run):
    code, doc, out = run(None, "--minimal")
    assert code == 0, doc["errors"]
    assert "Buffer_Plate_Leg" in {m.get("name") for m in world_of(out).findall("model")}


# ---------------------------------------------------------------- --minimal

def test_minimal_collapses_the_cells_into_one_group_spanning_their_footprint(run, pass_raw):
    code, doc, out = run(None, "--minimal")
    assert code == 0, doc["errors"]
    models = {m.get("name"): m for m in world_of(out).findall("model")}
    assert "Station_Test_Group" in models
    assert not any(n.startswith("Station_Test_Cell") for n in models)
    station = pass_raw["stations"]["st_test"]
    axis = gen_world.CELL_AXIS
    side = gen_world.CELL_FILL * station["cell_pitch"]
    span = (station["count"] - 1) * station["cell_pitch"] + side
    size = box_size(models["Station_Test_Group"])
    assert size[axis] == pytest.approx(span, abs=TOL)
    # centred over the whole run of cells, same as the 5 enclosures it replaces
    mid = raised(station["pose"], pass_raw)
    mid[axis] += (station["count"] - 1) * station["cell_pitch"] / 2.0
    assert pose_of(models["Station_Test_Group"]) == pytest.approx(mid, abs=TOL)


def test_minimal_frame_still_spans_what_the_five_cells_would_have(run):
    """The collapse is a drawing simplification — the frame envelope underneath must not shrink."""
    code1, doc1, _ = run()
    code2, doc2, _ = run(None, "--minimal")
    assert code1 == 0 and code2 == 0
    assert doc1["models"]["Frame"]["pose"] == pytest.approx(doc2["models"]["Frame"]["pose"], abs=TOL)
    assert doc1["models"]["Frame"]["size_m"] == pytest.approx(doc2["models"]["Frame"]["size_m"],
                                                               abs=TOL)


def test_minimal_fades_envelopes_but_not_the_conveyor(run):
    code, doc, out = run(None, "--minimal")
    assert code == 0, doc["errors"]
    models = {m.get("name"): m for m in world_of(out).findall("model")}
    for name in ("Frame", "Station_Drive", "Station_Test_Group", "Buffer_Plate",
                 "Buffer_Plate_Leg"):
        assert alpha_of(models[name]) == pytest.approx(gen_world.MINIMAL_ALPHA, abs=TOL), name
    assert alpha_of(models["Conveyor"]) == pytest.approx(1.0, abs=TOL)


def test_normal_mode_keeps_every_primitive_fully_opaque(run):
    code, doc, out = run()
    assert code == 0, doc["errors"]
    models = {m.get("name"): m for m in world_of(out).findall("model")}
    for name in ("Frame", "Station_Drive", "Buffer_Plate"):
        assert alpha_of(models[name]) == pytest.approx(1.0, abs=TOL), name


def test_minimal_leaves_includes_fully_opaque(run):
    """`<include>`s carry no material of their own to fade — they render as their real model."""
    code, doc, out = run(None, "--minimal")
    assert code == 0, doc["errors"]
    assert out.read_text().count("<material>") == len(
        [m for m in world_of(out).findall("model")])   # one per primitive, none per include


def test_minimal_default_output_paths_are_fixed_regardless_of_dev(tmp_path, pass_raw, models,
                                                                   monkeypatch):
    """No --out: --minimal always names its own files, so it can't collide with a real world."""
    monkeypatch.setattr(gen_world, "PKG", tmp_path / "pkg")
    spec = tmp_path / "line_spec.yaml"
    spec.write_text(yaml.safe_dump(pass_raw))
    (tmp_path / "line_spec.dev.yaml").write_text(yaml.safe_dump({}))   # --dev needs the overlay
    out = tmp_path / "pkg" / "worlds" / "main_line.minimal.dev.sdf"
    for extra in (["--minimal"], ["--minimal", "--dev"]):
        code = gen_world.main(["--spec", str(spec), "--models", str(models), *extra])
        assert code == 0
        assert out.is_file()
        doc = json.loads((tmp_path / "gen_world.minimal.dev.json").read_text())
        assert doc["passed"] and doc["minimal"] is True
        assert not (tmp_path / "pkg" / "worlds" / "main_line.dev.sdf").exists()
        assert not (tmp_path / "pkg" / "worlds" / "main_line.sdf").exists()


def test_a_spec_with_no_work_height_fails_naming_it(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["line"]["work_height"] = None
    code, doc, out = run(raw)
    assert code == 1
    assert "line.work_height" in doc["missing"]
    assert not out.exists()


# ---------------------------------------------------------------- CLI

@pytest.mark.parametrize("key, blank", [
    ("stations.st_drive.pose[0]",
     lambda raw: raw["stations"]["st_drive"]["pose"].__setitem__(0, None)),
    ("buffers.buf_plate.pose[2]",
     lambda raw: raw["buffers"]["buf_plate"]["pose"].__setitem__(2, None)),
    ("buffers.buf_pod.pose[1]",
     lambda raw: raw["buffers"]["buf_pod"]["pose"].__setitem__(1, None))])
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
