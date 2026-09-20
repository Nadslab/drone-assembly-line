"""P4 check: `build_all.py` regenerates every STEP/STL, inside the plan's 30 s budget.

The load-bearing assertion is `test_every_generator_runs_and_every_output_lands`: the point of the
file is that one command rebuilds the whole of P4, so the test pins the step list to the
generators and checks each one's files exist afterwards — not that it printed something. The
others are the ways it is allowed to fail: a generator failing, the budget, the 5 MB rule.
"""

import copy
import json

import pytest
import yaml

import spec as spec_mod
from cad import build_all

GENERATORS = [name for name, _, _ in build_all.GENERATORS]


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    """Every report — build_all's own and each generator's — lands in tmp_path."""
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    paths = lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json"   # noqa: E731
    monkeypatch.setattr(build_all, "report_path", paths)
    for _, _, module in build_all.GENERATORS:
        monkeypatch.setattr(module, "report_path", paths)
    monkeypatch.setattr(build_all.mesh_pipeline, "report_path", paths)


@pytest.fixture
def run(tmp_path, pass_raw):
    def go(raw=None, *extra):
        spec = tmp_path / "line_spec.yaml"
        spec.write_text(yaml.safe_dump(pass_raw if raw is None else raw))
        code = build_all.main(["--spec", str(spec), "--out", str(tmp_path / "out"),
                               "--src", str(tmp_path / "raw"), *extra])
        name = f"cad_all{'.dev' if '--dev' in extra else ''}.json"
        return code, json.loads((tmp_path / name).read_text())
    return go


# ---------------------------------------------------------------- the whole build

def test_every_generator_runs_and_every_output_lands(run, tmp_path):
    code, doc = run()
    assert code == 0, doc["errors"]
    assert [s["name"] for s in doc["steps"]] == GENERATORS + ["mesh_pipeline"]
    assert {s["name"]: s["status"] for s in doc["steps"]} == {
        **{name: "PASS" for name in GENERATORS}, "mesh_pipeline": "SKIP"}
    assert doc["outputs"], "the build produced no files"
    for path in doc["outputs"]:
        assert (tmp_path / "out" / path.rsplit("/", 1)[-1]).is_file()
    assert {p.rsplit(".", 1)[-1] for p in doc["outputs"]} == {"step", "stl"}


def test_it_fits_the_budget_the_plan_gives_it(run):
    """P4: all of it in under 30 s, which is why the generators run in one process."""
    code, doc = run()
    assert code == 0, doc["errors"]
    assert doc["elapsed_s"] < doc["budget_s"] == 30.0


def test_a_build_over_budget_fails_and_says_so(run):
    code, doc = run(None, "--budget-s", "0.0")
    assert code == 1
    assert any("budget" in e for e in doc["errors"])
    assert all(s["status"] in ("PASS", "SKIP") for s in doc["steps"])     # everything still ran


def test_no_output_breaks_the_5_mb_rule(run):
    """CLAUDE.md P0. Every STL is measured, not assumed, so the rule is visible in one report."""
    code, doc = run()
    assert code == 0, doc["errors"]
    stls = [p for p in doc["outputs"] if p.endswith(".stl")]
    assert doc["sizes"] and set(doc["sizes"]) == set(stls)
    assert max(doc["sizes"].values()) < build_all.MAX_BYTES


# ---------------------------------------------------------------- how it is allowed to fail

def test_a_failing_generator_fails_the_build_and_names_it(run, pass_raw):
    """A carrier the gantry cannot lift clear: spider_carrier fails, the build reports which."""
    raw = copy.deepcopy(pass_raw)
    raw["gantry"]["axes"]["z"]["max"] = 0.05
    code, doc = run(raw)
    assert code == 1
    assert any("spider_carrier" in e for e in doc["errors"])
    step = next(s for s in doc["steps"] if s["name"] == "spider_carrier")
    assert step["status"] == "FAIL" and step["code"] == 1


def test_one_failing_generator_does_not_stop_the_others(run, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["spindles"] = 6          # the presenter's pattern check
    code, doc = run(raw)
    assert code == 1
    status = {s["name"]: s["status"] for s in doc["steps"]}
    assert status["screw_presenter"] == "FAIL"
    assert status["pallet"] == status["tools"] == "PASS"


def test_the_warnings_of_every_generator_reach_the_run_report(run, pass_raw):
    code, doc = run()
    assert code == 0, doc["errors"]
    assert any(w.startswith("magazine: fd_mid_plate: parts.mid_plate.key") for w in doc["warnings"])


# ---------------------------------------------------------------- the raw-STL folder

def test_missing_raw_stl_is_a_skip_not_a_failure(run, tmp_path):
    """The build123d half has to stay runnable before any SolidWorks export exists."""
    assert not (tmp_path / "raw").exists()
    code, doc = run()
    assert code == 0, doc["errors"]
    step = next(s for s in doc["steps"] if s["name"] == "mesh_pipeline")
    assert step["status"] == "SKIP" and "raw" in step["note"]
    assert any("mesh_pipeline" in w for w in doc["warnings"])


def test_require_meshes_turns_that_skip_into_a_failure(run):
    code, doc = run(None, "--require-meshes")
    assert code == 1
    step = next(s for s in doc["steps"] if s["name"] == "mesh_pipeline")
    assert step["status"] == "FAIL"
    assert any("mesh_pipeline" in e for e in doc["errors"])


def test_mesh_pipeline_runs_when_the_raw_folder_is_there(run, tmp_path):
    (tmp_path / "raw").mkdir()
    code, doc = run()
    assert code == 0, doc["errors"]
    step = next(s for s in doc["steps"] if s["name"] == "mesh_pipeline")
    assert step["status"] == "PASS"
    assert json.loads((tmp_path / "meshes.json").read_text())["passed"] is True


# ---------------------------------------------------------------- dev

def test_dev_run_is_quarantined_and_gathers_every_placeholder(tmp_path, pass_raw, run):
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["esc"]["z_in_stack"] = None
    raw["gang_heads"]["gh_top"]["stroke"] = None
    (tmp_path / "line_spec.dev.yaml").write_text(yaml.safe_dump(
        {"parts": {"esc": {"z_in_stack": 0.045}}, "gang_heads": {"gh_top": {"stroke": 0.05}}}))
    code, doc = run(raw, "--dev")
    assert code == 0, doc["errors"]
    assert doc["dev"] is True
    assert sorted(doc["placeholders"]) == ["gang_heads.gh_top.stroke", "parts.esc.z_in_stack"]
    assert (tmp_path / "out" / "dev" / "pallet.step").is_file()
    assert not (tmp_path / "out" / "pallet.step").exists()
    assert not (tmp_path / "cad_all.json").exists()
