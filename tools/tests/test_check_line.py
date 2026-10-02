import copy
import json
import subprocess
import sys
import time

import pytest
import yaml

import check_line
import spec as spec_mod
from conftest import FAILING, FIXTURES, TOOLS, _drive_at_place, _sub_line_loader

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


# ------------------------------------------------------------------ SCARAs (D-012)

SCHEMA_CASES = {c[1]: c for c in FAILING if c[0] == "schema"}


@pytest.mark.parametrize("case, message", [
    ("scara_id_is_not_its_station", "scaras.scara_put: the SCARA at st_place is scara_place"),
    ("scara_station_does_not_exist", "scaras.scara_place.station: station 'st_nowhere' does not "
                                     "exist"),
    ("robot_step_at_a_station_without_scara", "sequence.place: moves a part with a robot but "
                                              "station 'st_place' has no SCARA (scara_place)"),
    ("scara_tool_does_not_exist", "scaras.scara_pod.tools: tool 'tool_nope' does not exist"),
    ("scara_model_does_not_exist", "scaras.scara_pod.model: robot model 'nope' does not exist"),
    ("tool_not_on_the_station_scara", "'tool_carrier' is not on scara_place, the SCARA at "
                                      "st_place"),
    ("place_and_drive_in_one_step", "sequence.place: moves part 'mid_plate' with a SCARA and "
                                    "drives fasteners"),
    ("actuate_step_without_device", "sequence.pins: an `actuate` step names the `device`"),
    ("station_in_no_milestone", "milestones: station 'st_pod' is in no milestone"),
    ("station_in_two_milestones", "milestones.back: station 'st_drive' is already in milestone "
                                  "'front'"),
    ("milestones_out_of_line_order", "milestones: read in order they give"),
])
def test_each_schema_rule_fails_for_its_own_reason(pass_raw, case, message):
    raw = copy.deepcopy(pass_raw)
    SCHEMA_CASES[case][2](raw)
    schema = by_name(check_line.run_checks(raw)[0])["schema"]
    assert schema.status == "FAIL"
    assert any(message in msg for _, msg in schema.findings), schema.findings


FIXTURE = {c[1]: c[2] for c in FAILING}


@pytest.mark.parametrize("check, case, level, message", [
    ("anchors", "station_with_an_absolute_pose", "FAIL",
     "stations.st_drive: absolute pose; place it by `anchor: {after, gap}`"),
    ("anchors", "anchored_to_the_wrong_station", "FAIL",
     "stations.st_drive.anchor.after is 'st_load', but the station before it in line order is "
     "'st_place'"),
    ("anchors", "feeder_with_an_absolute_pose", "FAIL",
     "feeders.fd_mid_plate: absolute pose; place it by `offset`"),
    ("scara_reach", "pick_beyond_reach", "FAIL",
     "scara_place: pick fd_mid_plate at (0.3, 0.6) is outside its reach: 0.412 m from the base, "
     "beyond reach 0.24"),
    ("scara_reach", "pick_inside_r_min", "FAIL",
     "scara_place: pick fd_mid_plate at (0.3, 0.3) is outside its reach: j2"),
    ("scara_reach", "pick_outside_j1", "FAIL",
     "scara_place: pick fd_mid_plate at (0.3, 0.3) is outside its reach: j1"),
    ("scara_reach", "pick_above_the_stroke", "FAIL",
     "scara_place: pick fd_mid_plate at z 0.7 is outside its z range 0..0.6"),
    ("safe_z", "safe_z_above_the_z_stroke", "FAIL",
     "scara_spider: place_spider needs safe_z=0.235 (buffer buf_fixture 0.100 + carrier_spider "
     "0.105 + margin 0.03), above its z range top 0.22"),
    ("payload", "over_payload_rated", "FAIL",
     "scara_spider: 0.900 kg (tools 0.300 + carrier_spider 0.600) > payload_rated 0.5 kg"),
    ("wrist_inertia", "over_inertia_rated", "WARN",
     "scara_spider: 0.0053 kg m^2 about J4 (tools 0.0002 + carrier_spider 0.0052) > "
     "inertia_rated 0.001"),
    ("scara_clearance", "base_in_the_conveyor", "FAIL",
     "scara_place: base at (0.5, 0.15) overlaps the conveyor by 50.0 mm"),
    ("scara_clearance", "base_on_a_buffer", "FAIL",
     "scara_place: base at (0.5, 0.24) overlaps the buffer buf_plate"),
    ("scara_clearance", "two_bases_overlap", "FAIL",
     "scara_load and scara_place: bases overlap by 50.0 mm"),
    ("scara_clearance", "envelopes_overlap", "WARN",
     "scara_load and scara_place: reach envelopes overlap by 100 mm; shared zone x"),
    ("arm_pin_clearance", "tooling_hole_next_to_a_screw", "FAIL",
     "drone.arm_tooling_holes[0] [0.09, 0.04] is 3.43 mm from arm screw hole 0"),
    ("arm_pin_clearance", "not_one_pin_per_arm", "FAIL",
     "drone.arm_tooling_holes has 3 hole(s) for 4 arm(s); one pin per arm"),
])
def test_each_scara_rule_says_what_and_where(pass_raw, check, case, level, message):
    FIXTURE[case](pass_raw)
    r = by_name(check_line.run_checks(pass_raw)[0])[check]
    assert any(lvl == level and message in m for lvl, m in r.findings), r.findings


def test_a_shared_reach_zone_names_the_interlock(pass_raw):
    FIXTURE["envelopes_overlap"](pass_raw)
    r = by_name(check_line.run_checks(pass_raw)[0])["scara_clearance"]
    assert [z["scaras"] for z in r.info["shared_zones"]] == [["scara_load", "scara_place"],
                                                              ["scara_spider", "scara_pod"]]
    assert all("needs an interlock" in m for lvl, m in r.findings if lvl == "WARN")


def test_reach_records_every_radius_and_joint_solution(pass_raw):
    r = by_name(check_line.run_checks(pass_raw)[0])["scara_reach"]
    assert r.status == "PASS"
    got = r.info["targets"]["scara_place"]
    assert {k: v["radius_m"] for k, v in got.items()} == {"pick fd_mid_plate": 0.2088,
                                                          "place st_place": 0.24}
    assert got["place st_place"]["j1_j2_deg"] == [[0.0, 0.0]]       # straight out at full reach


def test_safe_z_only_counts_what_is_under_the_path(pass_raw):
    """D-013: scara_place's reach circle holds buf_plate (st_load's, 0.1 m), but its paths
    (fd_mid_plate -> st_place -> home) never pass over it; only the magazine it picks from is."""
    import robot_geometry as rg
    spec = spec_mod.parse_spec(pass_raw)
    sz = rg.safe_z(spec, "scara_place", rg.CadHeights())
    names = dict(sz.items)
    assert "feeder fd_mid_plate" in names and "buffer buf_plate" not in names
    assert sz.value == pytest.approx(0.15 + 0.003 + 0.03)


def test_a_tool_switch_adds_its_disc_and_its_time(pass_raw):
    """A dual-tool end-effector (D-013): scara_place's steps alternate tools, so each starts with
    a 180 deg J4 turn, timed pi / (j4_speed x plan scale) and counted in its step."""
    import estimate_moves
    import robot_geometry as rg
    pass_raw["tools"]["tool_spare"] = {"grip": "vacuum", "dims": [0.05, 0.05, 0.02], "mass": 0.2,
                                       "offset_r": 0.03}
    pass_raw["scaras"]["scara_place"]["tools"].append("tool_spare")
    pass_raw["sequence"].insert(2, {"id": "place_more", "station": "st_place",
                                    "capability": "place", "part": "mid_plate",
                                    "from": "fd_mid_plate", "tool": "tool_spare",
                                    "duration_s": 1.0, "requires": ["place"]})
    pass_raw["sequence"][3]["requires"] = ["place_more"]
    spec = spec_mod.parse_spec(pass_raw)
    assert rg.tool_switches(spec, "scara_place") == ["place", "place_more"]
    doc = estimate_moves.estimate(spec, rg.CadHeights())
    steps = {s["id"]: s for s in doc["scaras"]["scara_place"]["steps"]}
    switch = 3.141592653589793 / (10.0 * 0.5)
    assert steps["place"]["tool_switch_s"] == pytest.approx(switch, abs=1e-4)
    assert steps["place"]["time_s"] == pytest.approx(steps["place"]["cycle"]["total"] + switch,
                                                     abs=1e-3)


def test_payload_and_wrist_inertia_report_every_scara(pass_raw):
    got = by_name(check_line.run_checks(pass_raw)[0])
    p, w = got["payload"].info["scaras"], got["wrist_inertia"].info["scaras"]
    assert p["scara_spider"]["payload_kg"] == pytest.approx(0.9)
    assert w["scara_spider"]["inertia"] == pytest.approx(0.005333, abs=1e-6)


def test_safe_z_counts_a_waiting_gang_head_only_when_it_is_under_a_path(pass_raw):
    """gh_top on a slide at st_place waits slide.stroke out on -y (scara_place is on +y): 0.25 m
    out it is clear of the SCARA's paths, 0.05 m out it is under the place."""
    import robot_geometry as rg
    FIXTURE["slide_too_slow"](pass_raw)
    spec = spec_mod.parse_spec(pass_raw)
    cad = rg.CadHeights()
    hb = rg.head_box(spec, "gh_top", cad)
    assert hb.slide_from == "-y" and hb.rect.y1 == pytest.approx(-0.15)
    assert not any("gh_top" in n for n, _ in rg.safe_z(spec, "scara_place", cad).items)
    pass_raw["gang_heads"]["gh_top"]["slide"]["stroke"] = 0.05
    items = dict(rg.safe_z(spec_mod.parse_spec(pass_raw), "scara_place", cad).items)
    assert items["gang head gh_top (retracted on its slide)"] == pytest.approx(hb.z1)


def test_slide_time_is_part_of_the_station_time(pass_raw):
    FIXTURE["slide_too_slow"](pass_raw)
    r = by_name(check_line.run_checks(pass_raw)[0])["station_time"]
    row = r.info["stations"]["st_place"]
    assert row["slide_s"] == pytest.approx(18.0)
    assert row["total_s"] == pytest.approx(row["scara_s"] + 3.0 + 18.0 + 0.5 / 0.3, abs=1e-3)
    assert r.status == "FAIL" and "slide 18.00" in r.findings[0][1]


def test_a_nest_demands_a_whole_set_per_pick(pass_raw):
    """fd_mid_plate supplies 20/60 x 20 = 6.67 parts per takt: 6 per pick fits, 7 does not."""
    for per_pick, status in ((6, "PASS"), (7, "FAIL")):
        pass_raw["feeders"]["fd_mid_plate"]["per_pick"] = per_pick
        assert by_name(check_line.run_checks(pass_raw)[0])["feeder_rate"].status == status


def test_robot_load_reports_every_scara_against_takt(pass_raw):
    r = by_name(check_line.run_checks(pass_raw)[0])["robot_load"]
    assert set(r.info["load_pct"]) == {"scara_load", "scara_place", "scara_spider", "scara_pod"}
    assert all(0 < p <= 100 for p in r.info["load_pct"].values())


def test_a_milestone_is_checked_on_its_own(pass_raw):
    sub = spec_mod.milestone_subset(pass_raw, "back")
    assert list(sub["stations"]) == ["st_test", "st_spider", "st_pod"]
    assert set(sub["scaras"]) == {"scara_spider", "scara_pod"}
    assert set(sub["buffers"]) == {"buf_fixture", "buf_pod"} and sub["feeders"] == {}
    assert set(sub["return_paths"]) == {"rt_fixture"}
    assert sub["gang_heads"] == {}
    # Every step here required one upstream (load, drive): that work is done before the milestone.
    assert [s["requires"] for s in sub["sequence"]] == [[], [], ["place_spider"],
                                                        ["remove_spider"]]
    results, passed = check_line.run_checks(sub)
    assert passed, [(r.name, r.findings) for r in results if r.status != "PASS"]
    with pytest.raises(spec_mod.SpecError, match="milestone 'nope' does not exist"):
        spec_mod.milestone_subset(pass_raw, "nope")


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
    assert "robot_models.epson_ls10b_700_z300.link1" in paths
    assert "scaras.scara_fit_top_plate.offset[1]" in paths
    assert "stations.st_build_frame.anchor.gap" in paths
    assert "drone.arm_tooling_holes[0][0]" in paths
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


def test_repo_dev_overlay_records_the_d013_result(dev_cad_reports):
    """D-013 phase 1 on the dev overlay: station A (st_build_frame) is converted and clean; the
    stations after it still stand in the D-012 pattern, where gh_top_plate waits on its slide
    under scara_fit_top_plate's path (safe_z FAIL), and two neighbour pairs share reach (WARN).
    actors keeps open gap B (WARN)."""
    results, passed = check_line.run_checks(spec_mod.load_raw(dev=True), cad_dir=dev_cad_reports,
                                            dev=True)
    got = by_name(results)
    assert not passed
    assert {r.name: r.status for r in results if r.status != "PASS"} == {
        "actors": "WARN", "scara_clearance": "WARN", "safe_z": "FAIL"}
    assert [m.split(":")[0] for lvl, m in got["safe_z"].findings if lvl == "FAIL"] == \
        ["scara_fit_top_plate"]
    assert all(p <= 100.0 for p in got["robot_load"].info["load_pct"].values())
    assert sorted(m.split()[1].rstrip(":") for lvl, m in got["actors"].findings
                  if lvl == "WARN") == ["drive_top_plate", "fit_side_plates", "seat_esc"]


def test_repo_dev_frame_milestone_passes(dev_cad_reports):
    raw = spec_mod.milestone_subset(spec_mod.load_raw(dev=True), "frame")
    results, passed = check_line.run_checks(raw, cad_dir=dev_cad_reports, dev=True)
    assert passed, [(r.name, r.findings) for r in results if r.status == "FAIL"]
    assert list(raw["stations"]) == ["st_load_pallet", "st_build_frame"]


def test_repo_spec_every_feeder_and_buffer_is_drawn_and_every_drive_has_a_driver():
    """The two gaps the D-010 audit was written to find stay fixed, on the base spec too."""
    results, _ = check_line.run_checks(spec_mod.load_raw())
    msgs = [m for _, m in by_name(results)["actors"].findings]
    assert not [m for m in msgs if "draws from it" in m or "has no driver" in m], msgs
    actors = by_name(results)["actors"].info["station_actors"]
    assert actors["st_load_pallet"] == "sub_line"
    assert actors["st_mount_servo"] == "parked"
    assert actors["st_fit_side_plates"] == "scara + gh_side_plates"
    assert actors["st_fit_top_plate"] == "scara + gh_top_plate"


def test_index_time_is_derived_never_stated(pass_raw):
    """D-010: conveyor.index_s is gone; the index time is pitch / belt_speed."""
    s = spec_mod.parse_spec(pass_raw)
    assert spec_mod.index_s(s) == pytest.approx(0.5 / 0.3)
    pass_raw["conveyor"]["index_s"] = 2.0
    with pytest.raises(spec_mod.SpecError, match="index_s"):
        spec_mod.parse_spec(pass_raw)


@pytest.mark.parametrize("case, message", [
    ("station_without_actor", "st_test: no actor for test"),
    ("two_primary_actors", "st_place: 2 primary actors (scara, test)"),
    ("gang_head_not_fastening_what_the_scara_placed",
     "st_place: 2 primary actors (driver, scara); a station has exactly one (a drive step"),
    ("gang_head_named_at_two_stations", "gang head gh_top is named at 2 stations (st_drive, "
                                        "st_place)"),
    ("two_gang_heads_at_a_scara_station", "st_place: 2 gang heads fasten what the SCARA placed "
                                           "(gh_top, gh_two)"),
    ("two_drivers_at_a_driver_station", "st_drive: 2 drivers (gh_top, gh_two)"),
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


def test_a_gang_head_fastening_what_the_scara_placed_shares_its_station(pass_raw):
    """D-011, D-012: a SCARA places, the gang head mounted at that station screws it down."""
    _drive_at_place(pass_raw)
    r = by_name(check_line.run_checks(pass_raw)[0])["actors"]
    assert r.status == "PASS", r.findings
    assert r.info["station_actors"]["st_place"] == "scara + gh_top"


def test_a_sub_line_loader_needs_no_scara(pass_raw):
    """st_load as a sub-line loader fed from buf_plate: actors passes, and scara_load goes."""
    _sub_line_loader("buf_plate")(pass_raw)
    del pass_raw["scaras"]["scara_load"]
    results, passed = check_line.run_checks(pass_raw)
    got = by_name(results)
    assert got["actors"].status == "PASS", got["actors"].findings
    assert got["actors"].info["station_actors"]["st_load"] == "sub_line"
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


def test_anchors_resolve_to_line_poses_and_carry_their_placeholders(tmp_path, pass_raw):
    """D-013: a station's pose is its anchor chain; an owned entity's is its station's pose plus
    its offset. A pose computed from a placeholder is a placeholder, component by component."""
    pass_raw["stations"]["st_place"]["anchor"]["gap"] = None
    spec_path = tmp_path / "line_spec.yaml"
    spec_path.write_text(yaml.safe_dump(pass_raw))
    (tmp_path / "line_spec.dev.yaml").write_text(
        yaml.safe_dump({"stations": {"st_place": {"anchor": {"gap": 0.7}}}}))
    m = spec_mod.load_merged(spec_path, dev=True)
    assert m.raw["stations"]["st_drive"]["pose"] == [1.2, 0.0, 0.0, 0.0]
    assert m.raw["feeders"]["fd_mid_plate"]["pose"] == pytest.approx([0.5, 0.3, 0.15, 0.0])
    ph = {p["path"] for p in m.placeholders}
    assert {"stations.st_place.pose[0]", "stations.st_drive.pose[0]",
            "feeders.fd_mid_plate.pose[0]", "scaras.scara_place.pose[0]"} <= ph
    assert "feeders.fd_mid_plate.pose[1]" not in ph          # y does not depend on an x gap
    # A pose written next to an anchor is refused: poses are derived.
    pass_raw["stations"]["st_place"]["pose"] = [0.5, 0.0, 0.0, 0.0]
    spec_path.write_text(yaml.safe_dump(pass_raw))
    with pytest.raises(spec_mod.SpecError, match="both `anchor` and `pose`"):
        spec_mod.load_merged(spec_path)
