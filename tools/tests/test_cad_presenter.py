"""P4 B6: the screw presenter is the gang head's own pattern, bored into a block.

The load-bearing assertion is `test_every_bore_is_a_hole_of_the_pattern_the_head_names`: the bores
are pinned to `gang_heads.<head>.pattern_from` — the plate's own screw pattern — to 0.01 mm, so a
presenter cannot drift from the plate its head screws into. The rest is the block having to be
possible: enough wall between counterbores, and a stroke long enough to push the screw out.
"""

import copy
import json

import pytest
import yaml

import spec as spec_mod
from cad import screw_presenter as presenter

TOL_MM = 0.01


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    monkeypatch.setattr(presenter, "report_path",
                        lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json")


def run_raw(tmp_path, raw, *extra):
    spec = tmp_path / "line_spec.yaml"
    spec.write_text(yaml.safe_dump(raw))
    code = presenter.main(["--spec", str(spec), "--out", str(tmp_path / "out"), *extra])
    name = f"cad_presenters{'.dev' if '--dev' in extra else ''}.json"
    return code, json.loads((tmp_path / name).read_text())


@pytest.fixture
def run(tmp_path, pass_raw):
    def go(raw=None, *extra):
        return run_raw(tmp_path, pass_raw if raw is None else raw, *extra)
    return go


def item(doc):
    return doc["items"]["fd_screw"]


# ---------------------------------------------------------------- position is what matters

def test_every_bore_is_a_hole_of_the_pattern_the_head_names(run, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    d = item(doc)["derived"]
    pattern = {tuple(round(c * 1000.0, 6) for c in h)
               for h in pass_raw["parts"]["bottom_plate"]["arm_holes"]["positions"]}
    assert d["pattern_from"] == "bottom_plate.arm_holes"
    assert {tuple(round(c, 6) for c in b) for b in d["bores_mm"]} == pattern


def test_moving_a_plate_hole_moves_its_bore(run, pass_raw):
    before = item(run()[1])["derived"]["bores_mm"]
    moved = copy.deepcopy(pass_raw)
    moved["parts"]["bottom_plate"]["arm_holes"]["positions"][0][1] += 0.004        # 4 mm
    after = item(run(moved)[1])["derived"]["bores_mm"]
    changed = [a for a in after if a not in before]
    assert len(changed) == 1
    assert changed[0][1] == pytest.approx(before[0][1] + 4.0, abs=TOL_MM)


def test_the_block_is_as_thick_as_the_screw_is_long(run, pass_raw):
    """`fasteners.<id>.length` sizes the block: the screw, room for its head, a lead for the bit."""
    code, doc = run()
    assert code == 0, doc["errors"]
    d = item(doc)["derived"]
    length = pass_raw["fasteners"]["m3_x8"]["length"] * 1000.0
    assert d["screw_length_mm"] == pytest.approx(length, abs=TOL_MM)
    assert d["counterbore"]["floor_mm"] == pytest.approx(length, abs=TOL_MM)
    assert d["block_size_mm"][2] == pytest.approx(
        length + presenter.HEAD_H + presenter.LEAD, abs=TOL_MM)


def test_a_longer_screw_makes_a_thicker_block(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["fasteners"]["m3_x8"]["length"] = 0.016
    thin, thick = (item(run(r)[1])["derived"]["block_size_mm"][2] for r in (pass_raw, raw))
    assert thick == pytest.approx(thin + 8.0, abs=TOL_MM)


# ---------------------------------------------------------------- what the spec has to agree on

def test_a_spindle_count_that_does_not_match_the_pattern_fails(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["spindles"] = 6
    code, doc = run(raw)
    assert code == 1
    assert "gang_heads.gh_top.spindles" in doc["errors"][0]
    assert "bottom_plate.arm_holes" in doc["errors"][0]


def test_a_stroke_too_short_to_push_the_screw_out_fails(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["stroke"] = 0.005                  # 5 mm, less than the block
    code, doc = run(raw)
    assert code == 1
    assert "gang_heads.gh_top.stroke" in doc["errors"][0]


def test_bores_too_close_to_leave_a_wall_fail_naming_the_pair(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    holes = raw["parts"]["bottom_plate"]["arm_holes"]["positions"]
    holes[1] = [holes[0][0] + 0.004, holes[0][1]]                  # 4 mm apart, Ø6.2 counterbores
    code, doc = run(raw)
    assert code == 1
    assert "counterbore" in doc["errors"][0]
    assert item(doc)["derived"]["closest_bores"]["pitch_mm"] == pytest.approx(4.0, abs=TOL_MM)


def test_screws_driven_from_below_fail(run, pass_raw):
    """The presenter holds screws by gravity; a head driving upward needs a different one."""
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["direction"] = "up"
    code, doc = run(raw)
    assert code == 1
    assert "gang_heads.gh_top.direction" in doc["errors"][0]


def test_a_fastener_that_is_not_m3_fails(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["fasteners"]["m4_x8"] = raw["fasteners"].pop("m3_x8")
    raw["feeders"]["fd_screw"]["part"] = "m4_x8"
    raw["sequence"][2]["fastener"] = "m4_x8"
    code, doc = run(raw)
    assert code == 1
    assert "fasteners.m4_x8" in doc["errors"][0]


def test_a_fastener_whose_feeder_does_not_exist_is_reported(run, pass_raw):
    """Nothing presents it, so nothing drives it — worth saying, even though it builds nothing."""
    raw = copy.deepcopy(pass_raw)
    raw["fasteners"]["m3_x12"] = {"length": 0.012, "feeder": "fd_screw_m3x12"}
    code, doc = run(raw)
    assert code == 0, doc["errors"]
    assert any("m3_x12" in w and "no presenter" in w for w in doc["warnings"])


# ---------------------------------------------------------------- CLI

def test_writes_step_stl_and_report(run, tmp_path):
    code, doc = run()
    assert code == 0, doc["errors"]
    out = tmp_path / "out"
    step, stl = out / "presenter_fd_screw.step", out / "presenter_fd_screw.stl"
    assert step.is_file() and stl.is_file()
    assert item(doc)["outputs"] == {"step": str(step), "stl": str(stl)}
    assert item(doc)["mass_kg"]["total"] > 0.0
    assert stl.stat().st_size < 5 * 1024 * 1024                  # P0: no STL over 5 MB
    assert presenter.MATERIAL.split(":")[0] in step.read_text()[:2000].replace("\n", " ")


def test_a_null_length_is_caught_by_the_schema_first(run, pass_raw):
    """`fasteners.<id>.length` is required, so a null there never reaches the geometry."""
    raw = copy.deepcopy(pass_raw)
    raw["fasteners"]["m3_x8"]["length"] = None
    code, doc = run(raw)
    assert code == 1
    assert any("fasteners.m3_x8.length" in e for e in doc["errors"])


@pytest.mark.parametrize("key, blank", [
    ("gang_heads.gh_top.stroke",
     lambda raw: raw["gang_heads"]["gh_top"].__setitem__("stroke", None)),
    ("gang_heads.gh_top.spindles",
     lambda raw: raw["gang_heads"]["gh_top"].__setitem__("spindles", None))])
def test_null_spec_value_fails_and_names_the_key(run, tmp_path, pass_raw, key, blank, capsys):
    raw = copy.deepcopy(pass_raw)
    blank(raw)
    code, doc = run(raw)
    assert code == 1
    assert any(key in m for m in doc["missing"])
    assert key in capsys.readouterr().err
    assert not (tmp_path / "out").exists()


def test_a_pattern_that_names_nothing_fails_naming_it(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["pattern_from"] = "bottom_plate.holes"
    code, doc = run(raw)
    assert code == 1
    assert any("parts.bottom_plate.holes" in m and "pattern_from" in m for m in doc["missing"])


def test_dev_run_is_quarantined_in_its_own_folder(tmp_path, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["stroke"] = None
    (tmp_path / "line_spec.dev.yaml").write_text(
        yaml.safe_dump({"gang_heads": {"gh_top": {"stroke": 0.05}}}))
    code, doc = run_raw(tmp_path, raw, "--dev")
    assert code == 0, doc["errors"]
    assert (tmp_path / "out" / "dev" / "presenter_fd_screw.step").is_file()
    assert not (tmp_path / "out" / "presenter_fd_screw.step").exists()
    assert doc["dev"] is True and doc["placeholders"] == ["gang_heads.gh_top.stroke"]
    assert not (tmp_path / "cad_presenters.json").exists()
