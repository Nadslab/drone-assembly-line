import copy
import json
import subprocess
import sys
import time

import pytest
import yaml

import check_line
import spec as spec_mod
from conftest import FAILING, FIXTURES, TOOLS

CHECK_NAMES = [name for name, _ in check_line.CHECKS]


def by_name(results):
    return {r.name: r for r in results}


def test_every_check_has_a_failing_fixture():
    assert {c for c, *_ in FAILING} == set(CHECK_NAMES)


def test_passing_spec_passes_every_check(pass_raw):
    results, passed = check_line.run_checks(pass_raw)
    assert passed
    assert {r.name: r.status for r in results} == {n: "PASS" for n in CHECK_NAMES}


def test_failing_fixture_trips_only_its_check(failing_case):
    check, raw, expected = failing_case
    results, passed = check_line.run_checks(raw)
    got = by_name(results)
    assert got[check].status == expected, got[check].findings
    assert passed == (expected != "FAIL")


def test_null_is_warn_unresolved_not_fail(pass_raw):
    pass_raw["parts"]["mid_plate"]["mass"] = None
    results, passed = check_line.run_checks(pass_raw)
    r = by_name(results)["unresolved"]
    assert passed
    assert r.status == "WARN"
    assert r.info["paths"] == ["parts.mid_plate.mass"]
    assert r.findings == [("WARN", "UNRESOLVED parts.mid_plate.mass")]


def test_line_balance_flags_bottleneck(pass_raw):
    r = by_name(check_line.run_checks(pass_raw)[0])["line_balance"]
    assert r.info["bottleneck"] == "st_test"  # 90 s / 5 cells = 18 s of 20 s takt
    assert r.info["utilization_pct"]["st_test"] == pytest.approx(90.0)


def test_test_bank_uses_cell_count(pass_raw):
    # 90 s over 5 cells = 18 s <= 20 s takt passes; 4 cells = 22.5 s fails.
    pass_raw["stations"]["st_test"]["count"] = 4
    r = by_name(check_line.run_checks(pass_raw)[0])["takt"]
    assert r.status == "FAIL"


def test_parked_steps_are_excluded_from_takt(pass_raw):
    pass_raw["sequence"].append({"id": "extra", "station": "st_drive", "capability": "drive",
                                 "duration_s": 500.0, "requires": ["drive"], "status": "parked"})
    assert by_name(check_line.run_checks(pass_raw)[0])["takt"].status == "PASS"


# ------------------------------------------------------------------ spec.py


def test_unknown_key_is_an_error(pass_raw):
    pass_raw["sequence"][0]["duraton_s"] = 5.0
    with pytest.raises(spec_mod.SpecError, match="duraton_s"):
        spec_mod.parse_spec(pass_raw)


def test_unparseable_spec_fails_schema_and_skips_the_rest(pass_raw):
    pass_raw["line"]["takt_s"] = -1
    results, passed = check_line.run_checks(pass_raw)
    got = by_name(results)
    assert not passed
    assert got["schema"].status == "FAIL"
    assert all(got[n].findings[0][0] == "SKIP" for n in CHECK_NAMES[1:])


def test_source_specific_fields_are_required(pass_raw):
    del pass_raw["parts"]["mid_plate"]["mesh"]
    with pytest.raises(spec_mod.SpecError, match="mesh"):
        spec_mod.parse_spec(pass_raw)


def test_repo_spec_parses_and_lists_unresolved():
    raw = spec_mod.load_raw()
    spec_mod.parse_spec(raw)  # exact §3 example must parse
    paths = list(check_line._walk_nulls(raw))
    assert "gantry.axes.x.min" in paths
    assert "sequence[0].duration_s" in paths
    assert "parts.bottom_plate.datums.diamond[0]" in paths


# ------------------------------------------------------------------ CLI


def run_cli(tmp_path, raw):
    spec_path = tmp_path / "spec.yaml"
    spec_path.write_text(yaml.safe_dump(raw))
    report = tmp_path / "out" / "line_check.json"
    t0 = time.monotonic()
    proc = subprocess.run(
        [sys.executable, str(TOOLS / "check_line.py"), "--spec", str(spec_path),
         "--report", str(report)], capture_output=True, text=True)
    return proc, json.loads(report.read_text()), time.monotonic() - t0


def test_cli_pass(tmp_path, pass_raw):
    proc, report, elapsed = run_cli(tmp_path, pass_raw)
    assert proc.returncode == 0, proc.stdout
    assert report["passed"] is True
    assert report["unresolved"] == []
    assert elapsed < 5


def test_cli_fail_exits_nonzero(tmp_path, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["gang_heads"]["gh_top"]["direction"] = "up"
    proc, report, _ = run_cli(tmp_path, raw)
    assert proc.returncode == 1
    assert report["passed"] is False
    assert report["checks"]["no_flip"]["status"] == "FAIL"


def test_cli_missing_spec_writes_failed_report(tmp_path):
    report = tmp_path / "r.json"
    proc = subprocess.run(
        [sys.executable, str(TOOLS / "check_line.py"), "--spec", str(tmp_path / "nope.yaml"),
         "--report", str(report)], capture_output=True, text=True)
    assert proc.returncode == 1
    assert json.loads(report.read_text())["passed"] is False


def test_fixture_file_is_valid_yaml():
    assert yaml.safe_load((FIXTURES / "pass_spec.yaml").read_text())["schema_version"] == 1


# ------------------------------------------------------------------ dev overlay


def test_overlay_fills_nulls_but_never_overrides_real_values():
    base = {"a": 1.0, "b": None, "c": [None, 2.0, None], "d": {"x": None, "y": 3}}
    overlay = {"a": 9.0, "b": 5.0, "c": [7.0, 8.0, None], "d": {"x": 4, "y": 99}}
    m = spec_mod.merge_overlay(base, overlay)
    assert m.raw == {"a": 1.0, "b": 5.0, "c": [7.0, 2.0, None], "d": {"x": 4, "y": 3}}
    assert {p["path"] for p in m.placeholders} == {"b", "c[0]", "d.x"}
    assert {p["path"] for p in m.shadowed} == {"a", "c[1]", "d.y"}
    assert base["b"] is None  # inputs are not mutated


def test_overlay_cannot_add_entities_or_resize_lists():
    with pytest.raises(spec_mod.SpecError, match="new entity"):
        spec_mod.merge_overlay({"feeders": {}}, {"feeders": {"fd_new": {"type": "step"}}})
    with pytest.raises(spec_mod.SpecError, match="items"):
        spec_mod.merge_overlay({"c": [None, None]}, {"c": [1.0]})
    with pytest.raises(spec_mod.SpecError, match="does not match"):
        spec_mod.merge_overlay({"s": [{"id": "a"}]}, {"s": [{"id": "b"}]})


def test_overlay_may_add_optional_fields_to_existing_entities():
    m = spec_mod.merge_overlay({"parts": {"p": {"mass": None}}},
                               {"parts": {"p": {"datums": {"round": [0.0, 0.0]}}}})
    assert m.raw["parts"]["p"]["datums"] == {"round": [0.0, 0.0]}
    assert [p["path"] for p in m.placeholders] == ["parts.p.datums.round[0]",
                                                   "parts.p.datums.round[1]"]


def test_repo_dev_overlay_resolves_every_null_and_shadows_nothing():
    m = spec_mod.load_merged(dev=True)
    assert list(check_line._walk_nulls(m.raw)) == []
    assert m.shadowed == []
    assert m.placeholders
    spec_mod.parse_spec(m.raw)
    base = spec_mod.load_raw()
    # Every real base value survives the merge unchanged.
    real = {p: v for p, v in spec_mod._leaves(base, "")}
    merged = {p: v for p, v in spec_mod._leaves(m.raw, "")}
    assert all(merged[p] == v for p, v in real.items())


def test_repo_dev_overlay_marks_every_placeholder_line():
    text = (spec_mod.DEFAULT_SPEC.with_name("line_spec.dev.yaml")).read_text().splitlines()
    for n, line in enumerate(text, 1):
        body = line.strip()
        if not body or body.startswith("#") or body.endswith(":"):
            continue
        if body.lstrip("- ").startswith("id:"):
            continue
        assert "# PLACEHOLDER" in line, f"line {n} lacks '# PLACEHOLDER': {line}"


def test_repo_dev_overlay_passes_all_checks_but_the_known_dangling_reference():
    results, _ = check_line.run_checks(spec_mod.load_raw(dev=True))
    bad = {r.name: r.findings for r in results if r.status != "PASS"}
    assert set(bad) == {"schema"}
    assert all("fd_screw_m3x12" in msg for _, msg in bad["schema"])


def test_cli_dev_writes_dev_report_with_placeholders(tmp_path):
    report = tmp_path / "dev.json"
    proc = subprocess.run(
        [sys.executable, str(TOOLS / "check_line.py"), "--dev", "--report", str(report)],
        capture_output=True, text=True)
    doc = json.loads(report.read_text())
    assert doc["dev"] is True
    assert len(doc["placeholders"]) > 100
    assert {"path", "value"} <= set(doc["placeholders"][0])
    assert doc["unresolved"] == []
    assert "DEV MODE" in proc.stdout


def test_dev_report_path_is_separate():
    assert spec_mod.report_path("x", dev=True).name == "x.dev.json"
    assert spec_mod.report_path("x").name == "x.json"
