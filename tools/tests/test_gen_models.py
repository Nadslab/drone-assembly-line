"""P5.2: a Gazebo model per part and magazine, computed from the spec and the cad reports.

The load-bearing assertions are `test_inertia_is_the_box_formula_over_mass_and_envelope` — an
inertia that is not the part's own is a sim that lies about how it moves — and
`test_a_part_with_no_mesh_yet_falls_back_to_its_primitive`: gz resolves a mesh URI when it parses
the world, so a model naming a mesh that is not there is a world that will not load at all.
"""

import copy
import json
import xml.etree.ElementTree as ET

import pytest
import yaml

import gen_models
import spec as spec_mod

TOL = 1e-9


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    monkeypatch.setattr(gen_models, "report_path",
                        lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json")


def write_cad_report(tmp_path, name, *, size, min_z=0.0, mass=1.0, stl=None, items=None):
    """A minimal stand-in for what a tools/cad generator writes, in millimetres."""
    box = {"min": [0.0, 0.0, min_z], "max": [size[0], size[1], min_z + size[2]], "size": list(size)}
    body = {"bbox_mm": box, "mass_kg": {"total": mass}, "outputs": {"stl": str(stl)} if stl else {}}
    doc = {"passed": True, "items": {k: {"status": "PASS", **body} for k in items}} if items \
        else {"passed": True, **body}
    (tmp_path / f"{name}.json").write_text(json.dumps(doc))


@pytest.fixture
def cad(tmp_path):
    """The cad reports gen_models reads for every build123d part and magazine."""
    stl = tmp_path / "cad_out" / "pallet.stl"
    stl.parent.mkdir(parents=True, exist_ok=True)
    stl.write_bytes(b"solid x\nendsolid x\n")
    write_cad_report(tmp_path, "cad_pallet", size=(200.0, 200.0, 80.0), min_z=-12.0, mass=2.0,
                     stl=stl)
    write_cad_report(tmp_path, "cad_carrier_spider", size=(227.0, 227.0, 105.0), mass=0.9)
    write_cad_report(tmp_path, "cad_carrier_pod", size=(109.0, 89.0, 79.0), mass=0.12)
    write_cad_report(tmp_path, "cad_magazines", size=(194.0, 194.0, 118.0), min_z=-118.0,
                     mass=2.9, items=["fd_mid_plate"])
    return stl


@pytest.fixture
def run(tmp_path, pass_raw, cad):
    def go(raw=None, *extra):
        spec = tmp_path / "line_spec.yaml"
        spec.write_text(yaml.safe_dump(pass_raw if raw is None else raw))
        code = gen_models.main(["--spec", str(spec), "--models", str(tmp_path / "models"),
                                "--meshes", str(tmp_path / "meshes"), *extra])
        name = f"gen_models{'.dev' if '--dev' in extra else ''}.json"
        return code, json.loads((tmp_path / name).read_text())
    return go


def sdf_of(tmp_path, ident, dev=False):
    path = tmp_path / "models" / ("dev" if dev else "") / ident / "model.sdf"
    return ET.fromstring(path.read_text()), path


# ---------------------------------------------------------------- what gets built

def test_one_model_per_part_and_per_magazine(run, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    assert set(doc["models"]) == set(pass_raw["parts"]) | {"fd_mid_plate"}


def test_names_follow_the_naming_table(run, tmp_path):
    code, doc = run()
    assert code == 0, doc["errors"]
    assert doc["models"]["carrier_spider"]["name"] == "Carrier_Spider"
    assert doc["models"]["bottom_plate"]["name"] == "Bottom_Plate"
    assert doc["models"]["fd_mid_plate"]["name"] == "Feeder_MidPlate"
    root, _ = sdf_of(tmp_path, "pallet")
    assert root.find("model").get("name") == "Pallet"


def test_equipment_is_static_and_a_part_is_not(run):
    """A part is carried, placed and attached to (P7), so it needs physics; a magazine does not."""
    code, doc = run()
    assert code == 0, doc["errors"]
    assert doc["models"]["fd_mid_plate"]["static"] is True
    assert doc["models"]["pallet"]["static"] is False


# ---------------------------------------------------------------- the physics numbers

def test_inertia_is_the_box_formula_over_mass_and_envelope(run, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    m = doc["models"]["mid_plate"]
    part = pass_raw["parts"]["mid_plate"]
    mass, (x, y, z) = part["mass"], part["envelope"]
    assert m["mass_kg"] == pytest.approx(mass)
    assert m["inertia_kgm2"] == pytest.approx(
        [mass * (y * y + z * z) / 12.0, mass * (x * x + z * z) / 12.0,
         mass * (x * x + y * y) / 12.0], rel=1e-5)


def test_a_cylinder_part_gets_the_cylinder_formula(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["standoff"] = {"source": "primitive", "shape": "cylinder",
                                "dims": [0.003, 0.012], "mass": 0.002}
    code, doc = run(raw)
    assert code == 0, doc["errors"]
    m = doc["models"]["standoff"]
    r, h, mass = 0.003, 0.012, 0.002
    assert m["shape"] == "cylinder"
    assert m["inertia_kgm2"] == pytest.approx(
        [mass * (3 * r * r + h * h) / 12.0, mass * (3 * r * r + h * h) / 12.0,
         mass * r * r / 2.0], rel=1e-5)


def test_a_plate_is_centred_on_its_datum_midpoint(run, pass_raw):
    """The plate frame's origin is the round datum, so the plate itself sits off to one side."""
    code, doc = run()
    assert code == 0, doc["errors"]
    d = pass_raw["parts"]["mid_plate"]["datums"]
    env = pass_raw["parts"]["mid_plate"]["envelope"]
    assert doc["models"]["mid_plate"]["centre_m"] == pytest.approx(
        [(d["round"][0] + d["diamond"][0]) / 2.0, (d["round"][1] + d["diamond"][1]) / 2.0,
         env[2] / 2.0], abs=1e-6)


def test_a_generated_part_uses_the_bounding_box_its_generator_measured(run):
    """The pallet hangs below its own origin: the collision box has to sit where the solid is."""
    code, doc = run()
    assert code == 0, doc["errors"]
    pallet = doc["models"]["pallet"]
    assert pallet["size_m"] == pytest.approx([0.2, 0.2, 0.08], abs=1e-9)
    assert pallet["centre_m"][2] == pytest.approx((-0.012 + 0.068) / 2.0, abs=1e-9)
    assert pallet["mass_from"] == "parts.pallet.mass"


def test_a_mass_the_spec_states_wins_over_a_measured_one(run, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    assert doc["models"]["pallet"]["mass_kg"] == pytest.approx(pass_raw["parts"]["pallet"]["mass"])
    assert doc["models"]["pallet"]["mass_from"] == "parts.pallet.mass"


def test_a_null_mass_falls_back_to_what_the_generator_measured(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["pallet"]["mass"] = None
    code, doc = run(raw)
    assert code == 0, doc["errors"]
    assert doc["models"]["pallet"]["mass_kg"] == pytest.approx(2.0)      # the cad report's
    assert doc["models"]["pallet"]["mass_from"].startswith("reports/cad_pallet")


# ---------------------------------------------------------------- meshes and frames

def test_the_mesh_is_millimetres_scaled_into_metres(run, tmp_path):
    code, doc = run()
    assert code == 0, doc["errors"]
    root, path = sdf_of(tmp_path, "pallet")
    mesh = root.find(".//visual/geometry/mesh")
    assert mesh.find("scale").text.split() == ["0.001"] * 3
    uri = mesh.find("uri").text
    assert not uri.startswith("/")                       # relative to the model file
    assert (path.parent / uri).resolve().is_file()


def test_a_generated_mesh_is_copied_into_the_package(run, tmp_path):
    """cad_out/ is not installed with the package, so the model cannot point at it."""
    code, doc = run()
    assert code == 0, doc["errors"]
    assert (tmp_path / "meshes" / "pallet.stl").is_file()
    assert "cad_out" not in doc["models"]["pallet"]["mesh_uri"]


def test_a_part_with_no_mesh_yet_falls_back_to_its_primitive(run, tmp_path):
    """A world naming a mesh that is not there does not load, so a model may never name one."""
    code, doc = run()
    assert code == 0, doc["errors"]
    assert doc["models"]["mid_plate"]["mesh_uri"] is None
    assert any("no mesh built for mid_plate" in w for w in doc["warnings"])
    root, _ = sdf_of(tmp_path, "mid_plate")
    assert root.find(".//visual/geometry/mesh") is None
    assert root.find(".//visual/geometry/box") is not None


def test_pick_and_datum_frames_come_from_the_spec(run, tmp_path, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    frames = doc["models"]["mid_plate"]["frames"]
    assert set(frames) == {"Mid_Plate_pick", "Mid_Plate_datum_round", "Mid_Plate_datum_diamond"}
    pick = pass_raw["parts"]["mid_plate"]["pick"]
    assert [float(c) for c in frames["Mid_Plate_pick"].split()[:3]] == pytest.approx(pick[:3])
    dia = pass_raw["parts"]["mid_plate"]["datums"]["diamond"]
    assert [float(c) for c in frames["Mid_Plate_datum_diamond"].split()[:3]] == pytest.approx(
        [dia[0], dia[1], 0.0])
    root, _ = sdf_of(tmp_path, "mid_plate")
    assert {f.get("name") for f in root.findall(".//frame")} == set(frames)


# ---------------------------------------------------------------- CLI

def test_writes_a_model_config_beside_every_model(run, tmp_path):
    code, doc = run()
    assert code == 0, doc["errors"]
    for ident, model in doc["models"].items():
        config = ET.fromstring((tmp_path / "models" / ident / "model.config").read_text())
        assert config.find("name").text == model["name"]
        assert config.find("sdf").text == "model.sdf"
    assert "do not edit" in (tmp_path / "models" / "pallet" / "model.sdf").read_text()


@pytest.mark.parametrize("key, blank", [
    ("parts.mid_plate.envelope[1]",
     lambda raw: raw["parts"]["mid_plate"]["envelope"].__setitem__(1, None)),
    ("parts.mid_plate.mass", lambda raw: raw["parts"]["mid_plate"].__setitem__("mass", None))])
def test_null_spec_value_fails_and_names_the_key(run, tmp_path, pass_raw, key, blank, capsys):
    raw = copy.deepcopy(pass_raw)
    blank(raw)
    code, doc = run(raw)
    assert code == 1
    assert any(key in m for m in doc["missing"])
    assert key in capsys.readouterr().err
    assert not (tmp_path / "models").exists()


def test_a_part_whose_generator_has_not_run_fails_naming_the_generator(run, tmp_path, pass_raw):
    (tmp_path / "cad_carrier_pod.json").unlink()
    code, doc = run()
    assert code == 1
    assert any("cad_carrier_pod.json" in m and "pod_tray.py" in m for m in doc["missing"])


def test_dev_run_is_quarantined_in_its_own_tree(tmp_path, pass_raw, run, cad):
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["mid_plate"]["mass"] = None
    (tmp_path / "line_spec.dev.yaml").write_text(
        yaml.safe_dump({"parts": {"mid_plate": {"mass": 0.05}}}))
    write_cad_report(tmp_path, "cad_pallet.dev", size=(200.0, 200.0, 80.0), min_z=-12.0, mass=2.0)
    write_cad_report(tmp_path, "cad_carrier_spider.dev", size=(1.0, 1.0, 1.0), mass=0.9)
    write_cad_report(tmp_path, "cad_carrier_pod.dev", size=(1.0, 1.0, 1.0), mass=0.1)
    write_cad_report(tmp_path, "cad_magazines.dev", size=(1.0, 1.0, 1.0), mass=2.0,
                     items=["fd_mid_plate"])
    code, doc = run(raw, "--dev")
    assert code == 0, doc["errors"]
    assert (tmp_path / "models" / "dev" / "pallet" / "model.sdf").is_file()
    assert not (tmp_path / "models" / "pallet").exists()
    assert doc["dev"] is True and doc["placeholders"] == ["parts.mid_plate.mass"]
    assert not (tmp_path / "gen_models.json").exists()
