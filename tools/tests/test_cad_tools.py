"""P4 B7: the gantry end-effectors, built from `gantry.tools` and the parts they pick.

Two load-bearing assertions. `test_the_cups_stay_inside_the_smallest_part_the_tool_picks` is the
vacuum tool's: a cup over the edge of a part never seals, and which parts a tool picks is the
sequence's to say. `test_the_carrier_tool_can_close_on_the_bosses_the_fixtures_build` is the other
half of tools/cad/gripper.py — the fixtures size their pick boss against `dims`, so the tool built
from those same `dims` has to be able to grip what they build.
"""

import copy
import json

import pytest
import yaml

import spec as spec_mod
from cad import pod_tray, spider_carrier, tools

TOL_MM = 0.01


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    monkeypatch.setattr(tools, "report_path",
                        lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json")


def run_raw(tmp_path, raw, *extra):
    spec = tmp_path / "line_spec.yaml"
    spec.write_text(yaml.safe_dump(raw))
    code = tools.main(["--spec", str(spec), "--out", str(tmp_path / "out"), *extra])
    name = f"cad_tools{'.dev' if '--dev' in extra else ''}.json"
    return code, json.loads((tmp_path / name).read_text())


@pytest.fixture
def run(tmp_path, pass_raw):
    def go(raw=None, *extra):
        return run_raw(tmp_path, pass_raw if raw is None else raw, *extra)
    return go


# ---------------------------------------------------------------- what gets built

def test_one_end_effector_per_gantry_tool(run, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    assert sorted(doc["items"]) == sorted(pass_raw["gantry"]["tools"])
    assert doc["items"]["tool_plate"]["derived"]["grip"] == "vacuum"
    assert doc["items"]["tool_carrier"]["derived"]["grip"] == "finger"


def test_the_tool_is_the_envelope_the_spec_gives_it(run, pass_raw):
    """`gantry.tools.<id>.dims` is what the fixtures are checked against, so it is the tool."""
    code, doc = run()
    assert code == 0, doc["errors"]
    for tid, spec_tool in pass_raw["gantry"]["tools"].items():
        d = doc["items"][tid]["derived"]
        assert d["dims_mm"] == pytest.approx([c * 1000.0 for c in spec_tool["dims"]], abs=TOL_MM)
        assert doc["items"][tid]["bbox_mm"]["min"][2] == pytest.approx(0.0, abs=TOL_MM)


def test_a_tool_only_handles_the_parts_the_sequence_gives_it(run, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    assert doc["items"]["tool_plate"]["derived"]["handles"] == ["mid_plate"]
    assert sorted(doc["items"]["tool_carrier"]["derived"]["handles"]) == ["carrier_pod",
                                                                         "carrier_spider"]


# ---------------------------------------------------------------- the vacuum tool

def test_the_cups_stay_inside_the_smallest_part_the_tool_picks(run, pass_raw):
    """A cup hanging over the edge of a part never seals, so the part bounds the pattern."""
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["mid_plate"]["envelope"] = [0.09, 0.09, 0.003]     # smaller than the tool
    code, doc = run(raw)
    assert code == 0, doc["errors"]
    cups = doc["items"]["tool_plate"]["derived"]["cups"]
    assert cups["inside_footprint_mm"] == pytest.approx([90.0, 90.0], abs=TOL_MM)
    for x, y in cups["centres_mm"]:
        for c in (x, y):
            assert abs(c) + tools.CUP_D / 2.0 <= 90.0 / 2.0 - tools.CUP_INSET + TOL_MM


def test_a_part_too_small_for_a_cup_fails_naming_it(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["mid_plate"]["envelope"] = [0.03, 0.03, 0.003]
    code, doc = run(raw)
    assert code == 1
    assert "mid_plate" in doc["errors"][0] and "seal" in doc["errors"][0]


def test_a_part_the_cups_cannot_lift_fails_naming_its_mass(run, pass_raw):
    """Cup area x working vacuum, over a lifting safety factor, against the part's own weight."""
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["mid_plate"]["mass"] = 20.0
    code, doc = run(raw)
    assert code == 1
    assert "parts.mid_plate.mass" in doc["errors"][0]


def test_the_lift_capacity_is_the_cups_it_actually_has(run, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    d = doc["items"]["tool_plate"]["derived"]
    area = d["cups"]["count"] * 3.141592653589793 * (tools.CUP_D / 2.0) ** 2
    assert d["lift"]["capacity_n"] == pytest.approx(
        area * tools.VACUUM_KPA / 1000.0 / tools.LIFT_SAFETY, abs=1e-3)
    assert d["lift"]["demand_n"] == pytest.approx(
        pass_raw["parts"]["mid_plate"]["mass"] * 9.80665, abs=1e-3)


# ---------------------------------------------------------------- the finger tool

def test_the_carrier_tool_can_close_on_the_bosses_the_fixtures_build(run):
    """B3/B4 size their pick boss against these same `dims`; B7 has to be able to grip it."""
    code, doc = run()
    assert code == 0, doc["errors"]
    low, high = doc["items"]["tool_carrier"]["derived"]["jaws"]["grips_boss_d_mm"]
    for boss in (spider_carrier.BOSS_D, pod_tray.BOSS_D):
        assert low <= boss <= high


def test_jaws_that_could_only_grip_a_pin_fail(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gantry"]["tools"]["tool_carrier"]["dims"] = [0.012, 0.012, 0.03]
    code, doc = run(raw)
    assert code == 1
    assert "gantry.tools.tool_carrier.dims" in doc["errors"][0]


def test_a_tool_nothing_picks_with_is_still_built_with_a_warning(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gantry"]["tools"]["tool_spare"] = {"grip": "finger", "dims": [0.06, 0.06, 0.03]}
    code, doc = run(raw)
    assert code == 0, doc["errors"]
    assert doc["items"]["tool_spare"]["status"] == "PASS"
    assert any("tool_spare" in w for w in doc["warnings"])


# ---------------------------------------------------------------- CLI

def test_writes_step_stl_and_report_per_tool(run, tmp_path):
    code, doc = run()
    assert code == 0, doc["errors"]
    for tid in ("tool_plate", "tool_carrier"):
        step, stl = tmp_path / "out" / f"{tid}.step", tmp_path / "out" / f"{tid}.stl"
        assert step.is_file() and stl.is_file()
        item = doc["items"][tid]
        assert item["outputs"] == {"step": str(step), "stl": str(stl)}
        assert item["mass_kg"]["total"] > 0.0
        assert stl.stat().st_size < 5 * 1024 * 1024              # P0: no STL over 5 MB
        assert tools.MATERIAL.split(":")[0] in step.read_text()[:2000].replace("\n", " ")


@pytest.mark.parametrize("key, blank", [
    ("gantry.tools.tool_plate.dims[2]",
     lambda raw: raw["gantry"]["tools"]["tool_plate"]["dims"].__setitem__(2, None)),
    ("parts.mid_plate.envelope[0]",
     lambda raw: raw["parts"]["mid_plate"]["envelope"].__setitem__(0, None)),
    ("parts.mid_plate.mass",
     lambda raw: raw["parts"]["mid_plate"].__setitem__("mass", None))])
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
    raw["gantry"]["tools"]["tool_plate"]["dims"] = [0.12, 0.12, None]
    (tmp_path / "line_spec.dev.yaml").write_text(
        yaml.safe_dump({"gantry": {"tools": {"tool_plate": {"dims": [None, None, 0.02]}}}}))
    code, doc = run_raw(tmp_path, raw, "--dev")
    assert code == 0, doc["errors"]
    assert (tmp_path / "out" / "dev" / "tool_plate.step").is_file()
    assert not (tmp_path / "out" / "tool_plate.step").exists()
    assert doc["dev"] is True and doc["placeholders"] == ["gantry.tools.tool_plate.dims[2]"]
    assert not (tmp_path / "cad_tools.json").exists()
