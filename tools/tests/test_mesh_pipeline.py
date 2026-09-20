import json
import shutil

import pytest
import trimesh
import yaml
from build123d import Box, Cylinder, Pos, export_stl

import mesh_pipeline as mp
import spec as spec_mod

SPEC_TEXT = """\
schema_version: 1   # keep me
line:
  takt_s: 20.0
  units: {length: m, mass: kg, time: s, angle: rad}
  origin: pallet_datum_round

parts:
  bottom_plate:
    source: solidworks            # solidworks | build123d | primitive
    mesh: meshes/drone/bottom_plate.stl
    envelope: [null, null, null]  # x,y,z m — filled by mesh_pipeline.py
    datums: {round: [0.0, 0.0], diamond: [null, null]}
  side_plate: {source: solidworks, mesh: meshes/drone/side_plate.stl, envelope: [0.1, null, 0.003]}

cad: {datum_d: 0.003}
fasteners: {}
feeders: {}
sources: {}
buffers: {}
gantry:
  axes:
    x: {min: 0.0, max: 3.0, v_max: 1.0, a_max: 2.0}
    y: {min: -0.5, max: 0.5, v_max: 1.0, a_max: 2.0}
    z: {min: 0.0, max: 0.6, v_max: 0.5, a_max: 2.0}
  tools: {}
  mount: [0, 0, 0]
gang_heads: {}
scaras: {}
conveyor: {pitch: null, index_s: null}
stations: {}
sequence: []
"""


def plate(path, size=(200.0, 200.0, 3.0), hole_at=(0.0, 0.0), hole_d=3.0):
    """A plate centred on the origin with one datum hole, exported as mm STL."""
    p = Box(*size)
    if hole_at is not None:
        p = p - Pos(hole_at[0], hole_at[1], 0) * Cylinder(hole_d / 2, size[2] * 2)
    export_stl(p, str(path))
    return path


@pytest.fixture
def work(tmp_path):
    spec = tmp_path / "line_spec.yaml"
    spec.write_text(SPEC_TEXT)
    src = tmp_path / "raw"
    src.mkdir()
    return spec, src, tmp_path / "out"


def run(work, *extra):
    spec, src, out = work
    return mp.main(["--spec", str(spec), "--src", str(src), "--out", str(out), *extra])


def report(dev=False):
    return json.loads(spec_mod.report_path("meshes", dev).read_text())


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    monkeypatch.setattr(mp, "report_path", lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json")
    monkeypatch.setattr(spec_mod, "report_path", mp.report_path)


def test_fills_null_envelope_and_preserves_comments(work):
    spec, src, out = work
    plate(src / "bottom_plate.stl")
    assert run(work) == 0
    text = spec.read_text()
    assert "envelope: [0.2, 0.2, 0.003]  # x,y,z m — filled by mesh_pipeline.py" in text
    assert text.splitlines()[0] == "schema_version: 1   # keep me"
    assert "# solidworks | build123d | primitive" in text
    # everything except the one edited line is byte-identical
    diff = [(a, b) for a, b in zip(SPEC_TEXT.splitlines(), text.splitlines()) if a != b]
    assert len(diff) == 1
    r = report()["meshes"]["bottom_plate"]
    assert r["envelope_status"] == ["filled"] * 3
    assert r["origin"]["error_m"] < 1e-6
    assert (out / "meshes/drone/bottom_plate.stl").is_file()
    assert report()["spec_updated"] is True


def test_rerun_is_a_match_and_leaves_spec_untouched(work):
    spec, src, _ = work
    plate(src / "bottom_plate.stl")
    run(work)
    before = spec.read_bytes()
    assert run(work) == 0
    assert spec.read_bytes() == before
    assert report()["meshes"]["bottom_plate"]["envelope_status"] == ["match"] * 3
    assert report()["spec_updated"] is False


def test_partial_envelope_fills_only_nulls_and_fails_on_mismatch(work):
    spec, src, out = work
    plate(src / "bottom_plate.stl")
    plate(src / "side_plate.stl", size=(100.0, 40.0, 3.0), hole_at=None)
    assert run(work) == 0
    assert "envelope: [0.1, 0.04, 0.003]" in spec.read_text().replace("{source", "\n{source") \
        or yaml.safe_load(spec.read_text())["parts"]["side_plate"]["envelope"] == [0.1, 0.04, 0.003]

    plate(src / "side_plate.stl", size=(101.0, 40.0, 3.0), hole_at=None)
    before = spec.read_bytes()
    (out / "meshes/drone/side_plate.stl").unlink()
    assert run(work) == 1
    r = report()["meshes"]["side_plate"]
    assert r["status"] == "FAIL" and "envelope.x" in r["messages"][0]
    assert not (out / "meshes/drone/side_plate.stl").exists()     # a failing part writes no mesh
    assert spec.read_bytes() == before


def test_failure_anywhere_blocks_spec_writeback(work):
    spec, src, _ = work
    plate(src / "bottom_plate.stl")                                 # would fill three nulls
    plate(src / "side_plate.stl", size=(150.0, 40.0, 3.0), hole_at=None)   # x mismatch
    before = spec.read_bytes()
    assert run(work) == 1
    assert spec.read_bytes() == before
    assert report()["spec_updated"] is False


def test_origin_off_datum_fails(work):
    spec, src, _ = work
    plate(src / "bottom_plate.stl", hole_at=(1.0, 0.0))
    assert run(work) == 1
    m = report()["meshes"]["bottom_plate"]
    assert m["status"] == "FAIL"
    assert m["origin"]["error_m"] == pytest.approx(0.001, abs=1e-5)


def test_missing_hole_fails(work):
    _, src, _ = work
    plate(src / "bottom_plate.stl", hole_at=None)
    assert run(work) == 1
    assert "no Ø3 mm hole" in " ".join(report()["meshes"]["bottom_plate"]["messages"])


def test_wrong_diameter_hole_is_not_the_datum(work):
    _, src, _ = work
    plate(src / "bottom_plate.stl", hole_d=3.4)
    assert run(work) == 1


def test_metre_export_is_rejected(work):
    _, src, _ = work
    plate(src / "bottom_plate.stl", size=(0.2, 0.2, 0.003), hole_d=0.003)
    assert run(work) == 1
    assert "millimetres" in " ".join(report()["meshes"]["bottom_plate"]["messages"])


def test_missing_raw_stl_warns_but_passes(work):
    assert run(work) == 0
    assert report()["meshes"]["bottom_plate"]["status"] == "WARN"


def test_missing_src_dir_fails(work):
    spec, src, out = work
    shutil.rmtree(src)
    assert run(work) == 1


def test_unclaimed_stl_is_reported(work):
    _, src, _ = work
    plate(src / "mystery.stl")
    run(work)
    assert [p.endswith("mystery.stl") for p in report()["unclaimed"]] == [True]


def test_decimation_shrinks_and_keeps_bbox(work):
    spec, src, out = work
    sphere = trimesh.creation.icosphere(subdivisions=6, radius=30.0)   # ~82k faces, 60 mm
    sphere.export(src / "side_plate.stl")
    assert run(work, "--max-faces", "5000") == 1                       # envelope 0.1 != 0.06
    text = spec.read_text().replace("envelope: [0.1, null, 0.003]", "envelope: [null, null, null]")
    spec.write_text(text)
    assert run(work, "--max-faces", "5000") == 0
    m = report()["meshes"]["side_plate"]
    assert m["faces_before"] > 80000 and m["faces_after"] <= 5100
    assert m["size_bytes"] == (out / "meshes/drone/side_plate.stl").stat().st_size < 5 * 1024 * 1024
    assert m["decimation_bbox_drift_mm"] <= 0.2
    out_mesh = trimesh.load(out / "meshes/drone/side_plate.stl")
    assert out_mesh.extents == pytest.approx([60.0] * 3, abs=0.2)      # still millimetres


def test_size_limit_forces_extra_decimation(work, monkeypatch):
    _, src, out = work
    monkeypatch.setattr(mp, "MAX_BYTES", 40_000)                       # ~800 faces
    trimesh.creation.icosphere(subdivisions=5, radius=30.0).export(src / "side_plate.stl")
    (work[0]).write_text(SPEC_TEXT.replace("envelope: [0.1, null, 0.003]", "envelope: [null, null, null]"))
    assert run(work, "--max-faces", "50000") == 0
    assert (out / "meshes/drone/side_plate.stl").stat().st_size < 40_000


def test_dev_mode_never_writes_spec_and_reports_separately(work):
    spec, src, _ = work
    plate(src / "bottom_plate.stl")
    before = spec.read_bytes()
    (spec.with_name("line_spec.dev.yaml")).write_text("schema_version: 1\n")
    assert run(work, "--dev") == 0
    assert spec.read_bytes() == before
    assert report(dev=True)["dev"] is True


def test_real_spec_patch_changes_only_the_null_characters(tmp_path):
    """Against the real line_spec.yaml: exactly one line differs, comments and alignment intact."""
    real = spec_mod.DEFAULT_SPEC.read_text()
    p = tmp_path / "line_spec.yaml"
    p.write_text(real)
    mp.write_envelopes(p, {"mid_plate": {0: 0.2, 2: 0.003}, "bottom_plate": {1: 0.15}})
    old, new = real.splitlines(), p.read_text().splitlines()
    assert len(old) == len(new)
    changed = [(a, b) for a, b in zip(old, new) if a != b]
    assert len(changed) == 2
    assert "envelope: [0.2, null, 0.003]" not in changed[1][1]        # spacing was [null,null,null]
    assert "envelope: [0.2,null,0.003]" in changed[1][1]
    assert "# x,y,z m — filled by mesh_pipeline.py, checked vs STL" in changed[0][1]
    assert spec_mod.parse_spec(yaml.safe_load(p.read_text()))


def test_patch_refuses_a_non_null_target(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text("parts:\n  a:\n    envelope: [0.1, null, null]\n")
    with pytest.raises(OSError, match="not a literal"):
        mp.write_envelopes(p, {"a": {0: 0.2}})
