import copy
import json
import subprocess
import sys
import time

import pytest
import yaml

import check_line
import spec as spec_mod
from conftest import FAILING, FIXTURES, TOOLS, _sub_line_loader

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


# ------------------------------------------------------------------ zones (D-008)

ZONE_CASES = {c[1]: c for c in FAILING if c[0] == "schema"}


@pytest.mark.parametrize("case, message", [
    ("zone_names_unknown_station", "gantries.gantry_1.zone: station 'st_nowhere' does not exist"),
    ("station_in_two_zones", "station 'st_place' is already in gantry_1's zone"),
    ("gantry_step_in_no_zone", "sequence.place: moves a part with a gantry but station 'st_place' "
                               "is in no gantry's zone"),
    ("gantry_id_not_numbered", "gantries.gantry_b: a gantry id is gantry_<n>"),
    ("one_tool_id_two_designs", "gantries.gantry_2.tools.tool_plate differs from "
                                "gantries.gantry_1.tools.tool_plate"),
    ("tool_not_on_the_zone_gantry", "'tool_carrier' is not on gantry_1, which serves st_place"),
    ("place_and_drive_in_one_step", "sequence.place: moves part 'mid_plate' with a gantry and "
                                    "drives fasteners"),
])
def test_each_zone_rule_fails_for_its_own_reason(pass_raw, case, message):
    raw = copy.deepcopy(pass_raw)
    ZONE_CASES[case][2](raw)
    schema = by_name(check_line.run_checks(raw)[0])["schema"]
    assert schema.status == "FAIL"
    assert any(message in msg for _, msg in schema.findings), schema.findings


def test_a_null_zone_defers_the_coverage_rule(pass_raw):
    """Until every zone is known, a station outside the known ones may be in the unknown one."""
    pass_raw["gantries"]["gantry_1"]["zone"] = None
    results, passed = check_line.run_checks(pass_raw)
    got = by_name(results)
    assert passed
    assert got["schema"].status == "PASS"
    assert got["reach"].status == "WARN"
    assert got["gantry_load"].status == "WARN"


def test_reach_is_judged_against_the_zone_gantry_only(pass_raw):
    """buf_plate serves both zones, so it is checked once against each gantry that draws on it."""
    r = by_name(check_line.run_checks(pass_raw)[0])["reach"]
    assert r.status == "PASS"
    pass_raw["feeders"]["fd_mid_plate"]["pose"] = [1.0, 0.3, 0.15, 0.0]
    r = by_name(check_line.run_checks(pass_raw)[0])["reach"]
    assert r.findings == [("FAIL", "feeder fd_mid_plate: x=1.000 m (gantry_1 frame) outside "
                                   "[0, 0.6]")]


FIXTURE = {c[1]: c[2] for c in FAILING}


@pytest.mark.parametrize("case, message", [
    ("post_in_the_conveyor", "gantry_1 post 1 at x,y [-0.1, 0.0, -0.2, -0.1] stands in the "
                             "conveyor (50.0 mm deep)"),
    ("post_inside_the_clearance", "gantry_1 post 1 is 50.0 mm from the conveyor edge, less than "
                                  "gantry_frame.conveyor_clearance 100 mm"),
])
def test_gantry_straddle_says_which_post_and_why(pass_raw, case, message):
    FIXTURE[case](pass_raw)
    r = by_name(check_line.run_checks(pass_raw)[0])["gantry_straddle"]
    assert ("FAIL", message) in r.findings, r.findings


def test_the_old_gantry_3_travel_puts_its_posts_in_the_belt(dev_cad_reports):
    """The D-008 zone-only y travel (0.4-0.9) is what the layout caught: D-009's frame rule
    widened it to 0.25. Put it back and gantry_straddle has to fail on gantry_3 alone."""
    raw = spec_mod.load_raw(dev=True)
    raw["gantries"]["gantry_3"]["axes"]["y"]["min"] = 0.4
    r = by_name(check_line.run_checks(raw, cad_dir=dev_cad_reports, dev=True)[0])[
        "gantry_straddle"]
    assert r.status == "FAIL"
    assert {m.split()[0] for _, m in r.findings} == {"gantry_3"}


def test_safe_z_failure_names_what_it_is_made_of(pass_raw):
    FIXTURE["safe_z_above_the_z_travel"](pass_raw)
    r = by_name(check_line.run_checks(pass_raw)[0])["safe_z"]
    assert r.findings == [("FAIL", "gantry_2: safe_z=0.372 (station st_pod 0.237 + "
                                   "carrier_spider 0.105 + margin 0.03) is outside its z travel "
                                   "[0, 0.3]")]


def test_a_nest_demands_a_whole_set_per_pick(pass_raw):
    """fd_mid_plate supplies 20/60 x 20 = 6.67 parts per takt: 6 per pick fits, 7 does not."""
    for per_pick, status in ((6, "PASS"), (7, "FAIL")):
        pass_raw["feeders"]["fd_mid_plate"]["per_pick"] = per_pick
        assert by_name(check_line.run_checks(pass_raw)[0])["feeder_rate"].status == status


def test_gantry_load_reports_every_gantry_against_takt(pass_raw):
    r = by_name(check_line.run_checks(pass_raw)[0])["gantry_load"]
    assert set(r.info["load_pct"]) == {"gantry_1", "gantry_2"}
    assert all(0 < p <= 100 for p in r.info["load_pct"].values())


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
    assert "gantries.gantry_1.axes.x.min" in paths
    assert "gantries.gantry_3.zone" in paths
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


@pytest.fixture(scope="module")
def dev_cad_reports():
    """The real dev builds of the build123d parts, which safe_z reads (built if missing)."""
    missing = [p for p in ("pallet", "carrier_spider", "carrier_pod")
               if not (spec_mod.REPO / "reports" / f"cad_{p}.dev.json").is_file()]
    if missing:
        subprocess.run([sys.executable, str(TOOLS / "cad" / "build_all.py"), "--dev"],
                       check=True, capture_output=True)
    return spec_mod.REPO / "reports"


def test_repo_dev_overlay_state_after_d010(dev_cad_reports):
    """D-010's recorded result. The pallet loader took load_pallet off gantry_1, which now
    passes (~12.2 s of 17.2 s); gantry_2 passes. gantry_3 took the side plates and is over
    budget (~18.3 s): the goal was gantry_load PASS on every gantry, and it does not hold yet.
    `actors` fails on the two stations where a gantry places and a gang head drives (open gap
    in D-010). Everything else passes."""
    results, _ = check_line.run_checks(spec_mod.load_raw(dev=True), cad_dir=dev_cad_reports,
                                       dev=True)
    got = by_name(results)
    bad = {r.name for r in results if r.status != "PASS"}
    assert bad == {"gantry_load", "actors"}, {n: got[n].findings for n in bad}
    fails = [m for lvl, m in got["gantry_load"].findings if lvl == "FAIL"]
    assert [m.split(":")[0] for m in fails] == ["gantry_3"]
    assert got["gantry_load"].info["load_pct"]["gantry_1"] <= 100.0
    assert got["gantry_load"].info["load_pct"]["gantry_2"] <= 100.0
    assert sorted(m.split(":")[0] for lvl, m in got["actors"].findings if lvl == "FAIL") == [
        "st_fit_side_plates", "st_fit_top_plate"]
    # The three drive steps whose screws are not chosen yet are UNRESOLVED, not failures.
    assert sorted(m.split()[1].rstrip(":") for lvl, m in got["actors"].findings
                  if lvl == "WARN") == ["drive_top_plate", "fit_side_plates", "seat_esc"]


def test_repo_spec_every_feeder_and_buffer_is_drawn_and_every_drive_has_a_driver():
    """The two gaps the D-010 audit was written to find stay fixed, on the base spec too."""
    results, _ = check_line.run_checks(spec_mod.load_raw())
    msgs = [m for _, m in by_name(results)["actors"].findings]
    assert not [m for m in msgs if "draws from it" in m or "has no driver" in m], msgs
    actors = by_name(results)["actors"].info["station_actors"]
    assert actors["st_pallet_loader"] == ["sub_line"]
    assert actors["st_mount_servo"] == ["parked"]


def test_index_time_is_derived_never_stated(pass_raw):
    """D-010: conveyor.index_s is gone; the index time is pitch / belt_speed."""
    s = spec_mod.parse_spec(pass_raw)
    assert spec_mod.index_s(s) == pytest.approx(0.5 / 0.3)
    pass_raw["conveyor"]["index_s"] = 2.0
    with pytest.raises(spec_mod.SpecError, match="index_s"):
        spec_mod.parse_spec(pass_raw)


@pytest.mark.parametrize("case, message", [
    ("station_without_actor", "st_test: no actor for test"),
    ("two_actor_types_at_one_station", "st_place: 2 actor types (driver, gantry)"),
    ("drive_step_without_driver", "drive: drive step at st_drive has no driver"),
    ("feeder_never_drawn_from", "feeder fd_spare: no active step draws from it"),
    ("buffer_never_drawn_from", "buffer buf_spare: no active step draws from it"),
    ("feeder_holds_another_part", "place: takes 'top_plate' from fd_mid_plate, which holds "
                                  "'mid_plate'"),
    ("buffer_filled_with_another_part", "load: takes 'mid_plate' from buf_plate, which no "
                                        "source fills with it"),
    ("part_from_nowhere", "place: part 'mid_plate' comes from nowhere"),
    ("sub_line_source_does_not_exist", "stations.st_load.source -> buffer/source 'buf_nope' "
                                       "does not exist"),
    ("sub_line_without_source", "st_load: a sub_line station needs `source`"),
])
def test_each_actor_rule_fails_for_its_own_reason(pass_raw, case, message):
    FIXTURE[case](pass_raw)
    r = by_name(check_line.run_checks(pass_raw)[0])["actors"]
    assert r.status == "FAIL"
    assert any(message in m for lvl, m in r.findings if lvl == "FAIL"), r.findings


def test_a_sub_line_loader_needs_no_gantry(pass_raw):
    """st_load as a sub-line loader fed from buf_plate: actors passes and gantry_1 loses load."""
    _sub_line_loader("buf_plate")(pass_raw)
    pass_raw["gantries"]["gantry_1"]["zone"] = ["st_place"]
    results, passed = check_line.run_checks(pass_raw)
    got = by_name(results)
    assert got["actors"].status == "PASS", got["actors"].findings
    assert got["actors"].info["station_actors"]["st_load"] == ["sub_line"]
    assert passed


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
