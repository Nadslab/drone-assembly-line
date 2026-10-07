import copy
import json
import math
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
    """gh_top on a slide at st_place waits slide.stroke out on -y (scara_place is on +y): 0.30 m
    out it is clear of the SCARA's paths, 0.05 m out it is under the place."""
    import robot_geometry as rg
    FIXTURE["slide_too_slow"](pass_raw)
    spec = spec_mod.parse_spec(pass_raw)
    cad = rg.CadHeights()
    hb = rg.head_box(spec, "gh_top", cad)
    assert hb.slide_from == "-y" and hb.rect.y1 == pytest.approx(-0.20)
    assert not any("gh_top" in n for n, _ in rg.safe_z(spec, "scara_place", cad).items)
    pass_raw["gang_heads"]["gh_top"]["slide"]["stroke"] = 0.05
    items = dict(rg.safe_z(spec_mod.parse_spec(pass_raw), "scara_place", cad).items)
    assert items["gang head gh_top (retracted on its slide)"] == pytest.approx(hb.z1)


def test_slide_time_is_part_of_the_station_time(pass_raw):
    FIXTURE["slide_too_slow"](pass_raw)
    r = by_name(check_line.run_checks(pass_raw)[0])["station_time"]
    row = r.info["stations"]["st_place"]
    assert row["slide_s"] == pytest.approx(18.0)
    # D-014: the index is the 0.25 m from st_place's queue slot / 0.3 m/s + the 0.5 s stop.
    assert row["index_s"] == pytest.approx(0.25 / 0.3 + 0.5)
    assert row["total_s"] == pytest.approx(row["scara_s"] + 3.0 + 18.0 + row["index_s"], abs=1e-3)
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
    assert "robot_models.epson_ls10b_700_z300.motion.a_max" in paths
    assert "robot_models.epson_ls10b_700_z300.motion.v_max" not in paths   # Epson spec (D-015)
    assert "robot_models.epson_ls10b_700_z300.inertia_max" not in paths    # Epson spec (D-015)
    assert "robot_models.epson_ls10b_700_z300.link1" not in paths      # measured (D-014)
    assert "scaras.scara_build_frame.park_pose.j1" in paths
    assert "layout.slot_gap" in paths
    assert "scara_defaults.cycle_derate" in paths
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


# D-016: the operator-refilled magazines at A and E have no park pose that keeps their refill
# column clear at the full fold (easing j2 for a magazine is not allowed). Flagged, not tuned away.
D016_OPEN = {
    "st_build_frame": [
        "scara_build_frame: parked: its tools crosses fd_mid_plate's refill envelope (magazine) "
        "by 130.8 mm in plan",
        "scara_build_frame: place_mid_plate place_to_park: its tools crosses fd_mid_plate's "
        "refill envelope (magazine) by 130.8 mm in plan"],
    "st_fit_top_plate": [
        "scara_fit_top_plate: parked: its link2 crosses fd_top_plate's refill envelope (magazine) "
        "by 61.3 mm in plan",
        "scara_fit_top_plate: place_top_plate place_to_park: its link2 crosses fd_top_plate's "
        "refill envelope (magazine) by 80.5 mm in plan"]}


def d016_fails(results, stations) -> None:
    """The only FAILs are D-016's open magazine refill crossings at `stations`."""
    fails = [(r.name, m) for r in results for lvl, m in r.findings if lvl == "FAIL"]
    assert fails == [("refill_clearance", m) for st in stations for m in D016_OPEN[st]], fails


def test_repo_dev_overlay_records_the_d016_result(dev_cad_reports):
    """D-016 on the dev overlay: five SCARA stations, every one inside takt, every safe_z inside
    the Epson's 0.30 m stroke, no shared reach zone, payload and wrist inertia under rating; every
    parked arm clear of its working head in plan, every path 20 mm clear of the waiting heads and
    their slide frames. Refill columns now cover the operator-refilled magazines too: B, C and D
    clear theirs, A (fd_mid_plate) and E (fd_top_plate) do not (D016_OPEN). Gap B stays open
    (WARN: four drive steps name no fastener), and A's spider in its carrier is over the standard
    cycle's 2 kg."""
    results, passed = check_line.run_checks(spec_mod.load_raw(dev=True), cad_dir=dev_cad_reports,
                                            dev=True)
    got = by_name(results)
    assert not passed
    d016_fails(results, ["st_build_frame", "st_fit_top_plate"])
    assert {r.name: r.status for r in results if r.status != "PASS"} == {
        "actors": "WARN", "robot_load": "WARN", "refill_clearance": "FAIL"}
    assert [m for lvl, m in got["robot_load"].findings if lvl == "WARN"] == [
        "scara_build_frame: place_spider: payload 2.100 kg (tools + carrier_spider) > the "
        "standard cycle's 2 kg: its times are optimistic"]
    stations = {"st_build_frame", "st_seat_esc", "st_fit_pod", "st_fit_side_plates",
                "st_fit_top_plate"}
    assert set(got["park_clearance"].info["stations"]) == set(got["slide_clearance"].info[
        "stations"]) == set(got["path_clearance"].info["stations"]) == stations
    # B, C, D re-chosen with the magazines (D-016); A and E keep theirs (no pose clears).
    park = got["park_clearance"].info["stations"]
    assert all(park[st]["clearance_m"] >= 0.03 for st in stations - {"st_fit_top_plate"})
    assert park["st_fit_top_plate"]["clearance_m"] == pytest.approx(0.0118)
    assert all(d["clearance_m"] >= 0.02 for d in got["path_clearance"].info["stations"].values())
    refill = got["refill_clearance"].info["stations"]
    assert set(refill) == stations
    assert {st: sorted(d["envelopes"]) for st, d in refill.items()} == {
        "st_build_frame": ["buf_spider", "fd_mid_plate"], "st_seat_esc": ["fd_standoff"],
        "st_fit_pod": ["buf_pod"], "st_fit_side_plates": ["fd_side_plate"],
        "st_fit_top_plate": ["fd_top_plate"]}
    assert all(d["head_clearance_m"] >= 0.0 for d in refill.values())
    assert sorted(m.split()[1].rstrip(":") for lvl, m in got["actors"].findings
                  if lvl == "WARN") == ["drive_esc", "drive_pod", "drive_side_plates",
                                        "drive_top_plate"]
    assert set(got["robot_load"].info["load_pct"]) == {
        "scara_build_frame", "scara_seat_esc", "scara_fit_pod", "scara_fit_side_plates",
        "scara_fit_top_plate"}
    assert got["scara_clearance"].info["shared_zones"] == []
    assert got["robot_load"].info["calibration"]["epson_ls10b_700_z300"]["v_max"] == 9.8


def test_repo_dev_batch_1_is_checked_on_its_own(dev_cad_reports):
    """D-015 batch 1: loader, A and B — across the frame | electronics boundary. B's index is still
    the real gap from A's last queue slot, and the partition between the two milestones stays."""
    raw = spec_mod.station_subset(spec_mod.load_raw(dev=True),
                                  ["st_load_pallet", "st_build_frame", "st_seat_esc"])
    assert raw["milestones"] == {"frame": ["st_load_pallet", "st_build_frame"],
                                 "electronics": ["st_seat_esc"]}
    assert set(raw["buffers"]) == {"buf_plate", "buf_spider"}
    assert set(raw["return_paths"]) == {"rt_carrier_spider"}
    assert set(raw["gang_heads"]) == {"gh_arm_sandwich", "gh_esc"}
    results, passed = check_line.run_checks(raw, cad_dir=dev_cad_reports, dev=True)
    d016_fails(results, ["st_build_frame"])
    full = spec_mod.station_index(spec_mod.load_spec(dev=True))
    cut = spec_mod.station_index(spec_mod.parse_spec(raw))
    assert {s: cut[s]["index_s"] for s in cut} == {s: full[s]["index_s"] for s in cut}


@pytest.mark.parametrize("name, open_at", [("frame", ["st_build_frame"]), ("electronics", []),
                                           ("closeout", ["st_fit_top_plate"])])
def test_repo_dev_every_milestone_is_checked_on_its_own(dev_cad_reports, name, open_at):
    """Each milestone on its own fails only on D-016's open magazine crossings inside it."""
    raw = spec_mod.milestone_subset(spec_mod.load_raw(dev=True), name)
    results, passed = check_line.run_checks(raw, cad_dir=dev_cad_reports, dev=True)
    assert passed == (not open_at)
    d016_fails(results, open_at)


def test_repo_dev_batch_2_is_checked_on_its_own(dev_cad_reports):
    """D-016 batch 2: C, D, E. Their sources stand at y -0.27 (fd_top_plate 13 mm clear of the
    0.32 m pallet lane); C and D park clear of every refill column, E does not."""
    raw = spec_mod.station_subset(spec_mod.load_raw(dev=True),
                                  ["st_fit_pod", "st_fit_side_plates", "st_fit_top_plate"])
    spec = spec_mod.parse_spec(raw)
    half = spec.layout.pallet_block.width / 2.0
    import robot_geometry as rg
    for fid in ("buf_pod", "fd_side_plate", "fd_top_plate"):
        assert -half - rg.source_rect(spec, fid).y1 >= 0.010 - rg.EPS
    results, passed = check_line.run_checks(raw, cad_dir=dev_cad_reports, dev=True)
    d016_fails(results, ["st_fit_top_plate"])


def test_repo_dev_frame_milestone_is_cut_to_its_stations():
    raw = spec_mod.milestone_subset(spec_mod.load_raw(dev=True), "frame")
    assert list(raw["stations"]) == ["st_load_pallet", "st_build_frame"]


def test_repo_spec_every_feeder_and_buffer_is_drawn_and_every_drive_has_a_driver():
    """The two gaps the D-010 audit was written to find stay fixed, on the base spec too."""
    results, _ = check_line.run_checks(spec_mod.load_raw())
    msgs = [m for _, m in by_name(results)["actors"].findings]
    assert not [m for m in msgs if "draws from it" in m or "has no driver" in m], msgs
    actors = by_name(results)["actors"].info["station_actors"]
    assert actors["st_load_pallet"] == "sub_line"
    assert actors["st_build_frame"] == "scara + gh_arm_sandwich"
    assert actors["st_seat_esc"] == "scara + gh_esc"
    assert actors["st_fit_pod"] == "scara + gh_pod"
    assert actors["st_fit_side_plates"] == "scara + gh_side_plates"   # mount_servo is parked
    assert actors["st_fit_top_plate"] == "scara + gh_top_plate"


def test_index_time_is_derived_never_stated(pass_raw):
    """D-010: conveyor.index_s is gone. D-014: pitch / belt_speed is only the old index, kept to
    compare with the per-station one."""
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


# ------------------------------------------------------------------ D-014


def test_queue_slots_fill_each_gap_at_slot_pitch_and_set_the_index(pass_raw):
    """0.5 m gaps at a 0.25 m slot pitch (0.20 pallet + 0.05 gap): one slot each, and the pallet
    enters from it over 0.25 m. The first station is entered from the in-feed slot."""
    s = spec_mod.parse_spec(pass_raw)
    gaps = spec_mod.queue_gaps(s)
    assert [(g["after"], g["slots"], g["enter_m"]) for g in gaps] == [
        ("st_load", 1, 0.25), ("st_place", 1, 0.25), ("st_drive", 1, 0.25),
        ("st_test", 1, 0.25), ("st_spider", 1, 0.25)]
    idx = spec_mod.station_index(s)
    assert idx["st_place"]["index_s"] == pytest.approx(0.25 / 0.3 + 0.5)
    assert idx["st_place"]["from"] == "slot 1 after st_load"
    assert idx["st_load"]["from"] == "in-feed slot (pallet return)"
    # A 1.2 m gap holds 3 slots (4 x 0.25 would leave no pitch before the station): 0.45 m in.
    pass_raw["stations"]["st_place"]["anchor"]["gap"] = 1.2
    g = spec_mod.queue_gaps(spec_mod.parse_spec(pass_raw))[0]
    assert (g["slots"], g["enter_m"]) == (3, pytest.approx(0.45))
    # No room for a slot: the pallet comes straight from the station before.
    pass_raw["stations"]["st_place"]["anchor"]["gap"] = 0.4
    g = spec_mod.queue_gaps(spec_mod.parse_spec(pass_raw))[0]
    assert (g["slots"], g["enter_m"]) == (0, pytest.approx(0.4))
    assert spec_mod.station_index(spec_mod.parse_spec(pass_raw))["st_place"]["from"] == "st_load"


def test_a_milestone_keeps_the_gap_into_its_first_station(pass_raw):
    sub = spec_mod.milestone_subset(pass_raw, "back")
    assert sub["entry"]["after"] == "st_drive"
    idx = spec_mod.station_index(spec_mod.parse_spec(sub))
    assert idx["st_test"]["index_s"] == pytest.approx(0.25 / 0.3 + 0.5)


@pytest.mark.parametrize("check, case, level, message", [
    ("park_clearance", "parked_arm_in_the_working_head", "FAIL",
     "scara_place parked at j1 0.0°, j2 0.0°: its tools overlaps gh_top head (working) in plan "
     "by 84.9 mm"),
    # D-015: plan, not 3D — link2 is 2 mm above the head, but over it in plan by 30 mm.
    ("park_clearance", "parked_arm_in_the_working_head", "FAIL",
     "its link2 overlaps gh_top head (working) in plan by 30.0 mm (3D 2.0 mm)"),
    ("path_clearance", "arm_path_past_a_tall_rail", "FAIL",
     "scara_place: place pick_to_place: its tools comes 15.1 mm from gh_top rail; needs 20 mm"),
    ("refill_clearance", "parked_over_a_refilled_buffer", "FAIL",
     "scara_place: parked: its tools crosses buf_mid's refill envelope (buffer) by 84.9 mm in "
     "plan"),
    ("wrist_inertia", "over_inertia_max", "FAIL", "> inertia_max 0.001"),
    ("park_clearance", "null_park_pose", "WARN",
     "UNRESOLVED scaras.scara_place.park_pose is null (st_place drives a gang head)"),
    ("slide_clearance", "part_lands_on_the_retracted_head", "FAIL",
     "st_place: place carries mid_plate (band 0.141 m, at safe_z 0.301) -141.4 mm from gh_top "
     "retracted on its slide; needs 20 mm"),
])
def test_each_d014_rule_says_what_and_where(pass_raw, check, case, level, message):
    FIXTURE[case](pass_raw)
    r = by_name(check_line.run_checks(pass_raw)[0])[check]
    assert any(lvl == level and message in m for lvl, m in r.findings), r.findings


def test_a_head_on_a_slide_parks_its_scara_and_both_clearances_pass(pass_raw):
    """gh_top 0.30 m out on its slide at st_place: the parked arm (j1 2.5, folded) is 0.110 m
    from the head's working envelope in plan (D-015; 0.343 m in 3D), the mid plate's band 58.6 mm
    from the waiting head, the arm's paths 30.5 mm from its rail."""
    from conftest import _slide_at_place
    _slide_at_place(0.5, 0.5)(pass_raw)
    got = by_name(check_line.run_checks(pass_raw)[0])
    assert got["park_clearance"].status == got["slide_clearance"].status == "PASS"
    assert got["path_clearance"].status == "PASS"
    park = got["park_clearance"].info["stations"]["st_place"]
    assert park["clearance_m"] == pytest.approx(0.110, abs=1e-4)
    assert park["clearance_3d_m"] == pytest.approx(0.3432, abs=1e-4)
    assert got["path_clearance"].info["stations"]["st_place"]["clearance_m"] == \
        pytest.approx(0.0305, abs=1e-4)
    assert got["slide_clearance"].info["stations"]["st_place"]["clearance_m"] == \
        pytest.approx(0.0586, abs=1e-4)
    heads = got["park_clearance"].info["stations"]["st_place"]["heads"]
    assert heads == ["gh_top"]


def test_the_slide_frame_runs_from_the_working_head_to_a_post_beyond_the_waiting_one(pass_raw):
    import robot_geometry as rg
    from conftest import _slide_at_place
    _slide_at_place(0.5, 0.5)(pass_raw)
    hb = rg.head_box(spec_mod.parse_spec(pass_raw), "gh_top", rg.CadHeights())
    rail, post = dict((n, (r, z0, z1)) for n, r, z0, z1 in hb.frame).values()
    assert (rail[0].y0, rail[0].y1) == pytest.approx((-0.1 - 0.3 - 0.1, -0.1))
    assert (rail[1], rail[2]) == pytest.approx((hb.z1 - 0.03, hb.z1 + 0.03))
    assert (post[0].y0, post[0].y1) == pytest.approx((-0.5, -0.4))
    assert post[1] == pytest.approx(-pass_raw["line"]["work_height"]) and post[2] == rail[2]
    assert [n for n, *_ in hb.working_envelope()] == ["head (working)", "rail", "post"]


def test_park_choice_is_the_nearest_clearing_j1(pass_raw):
    import robot_geometry as rg
    from conftest import _parked_over_the_head
    _parked_over_the_head(pass_raw)
    spec = spec_mod.parse_spec(pass_raw)
    c = rg.choose_park(spec, "scara_place", rg.CadHeights())
    assert c["clearance_m"] >= rg.PARK_CLEAR_M
    # One step nearer the place pose does not clear.
    nearer = c["j1"] - math.copysign(math.radians(rg.PARK_STEP_DEG), c["j1"] - c["place_j"][0])
    pc = rg.park_clearance(spec, "scara_place", (nearer, c["j2"]), rg.CadHeights())
    assert pc["clearance_m"] < rg.PARK_CLEAR_M


# ------------------------------------------------------------------ D-015


def test_a_station_run_across_milestones_keeps_its_entry_and_both_milestones(pass_raw):
    """--stations st_drive st_test: the run crosses front | back. st_drive is still entered from
    st_place's queue slot, and each milestone keeps the stations of the run that are in it."""
    sub = spec_mod.station_subset(pass_raw, ["st_test", "st_drive"])     # any order
    assert list(sub["stations"]) == ["st_drive", "st_test"]
    assert sub["entry"]["after"] == "st_place"
    assert sub["milestones"] == {"front": ["st_drive"], "back": ["st_test"]}
    assert sub["pallet_in"] == ["bottom_plate", "mid_plate"]
    s = spec_mod.parse_spec(sub)
    assert spec_mod.station_index(s)["st_drive"]["index_s"] == pytest.approx(0.25 / 0.3 + 0.5)
    # A milestone is the same cut by name.
    assert spec_mod.milestone_subset(pass_raw, "back") == spec_mod.station_subset(
        pass_raw, ["st_test", "st_spider", "st_pod"])
    # The line's first station has no entry: it keeps the in-feed slot.
    head = spec_mod.station_subset(pass_raw, ["st_load"])
    assert "entry" not in head
    assert spec_mod.station_index(spec_mod.parse_spec(head))["st_load"]["from"] == \
        "in-feed slot (pallet return)"


@pytest.mark.parametrize("stations, message", [
    (["st_load", "st_drive"], "are not consecutive in line order"),
    (["st_nowhere"], "station 'st_nowhere' does not exist"),
    ([], "no station to cut the spec down to"),
])
def test_a_station_run_must_be_consecutive_and_exist(pass_raw, stations, message):
    with pytest.raises(spec_mod.SpecError) as exc:
        spec_mod.station_subset(pass_raw, stations)
    assert any(message in e for e in exc.value.errors), exc.value.errors


def test_the_station_list_takes_commas_or_spaces():
    assert spec_mod.parse_station_list(["a,b", "c"]) == ["a", "b", "c"]
    assert spec_mod.parse_station_list(None) is None


def test_cli_stations_writes_its_own_report(tmp_path, pass_raw):
    spec = tmp_path / "spec.yaml"
    spec.write_text(yaml.safe_dump(pass_raw))
    report = tmp_path / "line_check.batch1.json"
    proc = subprocess.run([sys.executable, str(TOOLS / "check_line.py"), "--spec", str(spec),
                           "--stations", "st_place,st_drive", "--name", "batch1", "--report",
                           str(report), "--cad-reports", str(FIXTURES / "cad")],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "STATIONS batch1: st_place, st_drive" in proc.stdout
    assert json.loads(report.read_text())["passed"] is True
    proc = subprocess.run([sys.executable, str(TOOLS / "check_line.py"), "--spec", str(spec),
                           "--stations", "st_place", "--milestone", "front"],
                          capture_output=True, text=True)
    assert proc.returncode == 2 and "exclusive" in proc.stderr


def test_a_pick_from_the_refilled_buffer_is_excluded_not_crossed(pass_raw):
    """The leg out of park that picks from buf_mid ends in its refill column by design: it is
    listed as excluded, the parked arm and the leg into park are what is checked."""
    import robot_geometry as rg
    from conftest import _refilled_buffer_under_the_park
    _refilled_buffer_under_the_park(pass_raw)
    rc = rg.refill_clearance(spec_mod.parse_spec(pass_raw), "scara_place", rg.CadHeights())
    assert rc["excluded"] == ["place park_to_pick: picks from buf_mid"]
    assert set(rc["envelopes"]["buf_mid"]) == {"kind", "parked", "path", "head"}
    assert rc["envelopes"]["buf_mid"]["path"]["at"].startswith("place place_to_park")


def test_a_buffer_no_source_refills_has_no_refill_envelope(pass_raw):
    """buf_fixture (carriers coming back on their own loop) is not refilled while the SCARA
    parks; fd_mid_plate, a magazine an operator refills, is (D-016); the blow feeder is not."""
    import robot_geometry as rg
    from conftest import _slide_at_place
    _slide_at_place(0.5, 0.5)(pass_raw)
    s = spec_mod.parse_spec(pass_raw)
    envs, missing = rg.refill_envelopes(s, "scara_place")
    assert missing == [] and {e: v[2] for e, v in envs.items()} == {"fd_mid_plate": "magazine"}
    assert rg.refill_envelopes(s, "scara_spider") == ({}, [])


def test_an_operator_magazine_under_the_park_fails_and_the_head_is_checked_too(pass_raw):
    """D-016: fd_mid_plate moved under scara_place's park pose (TCP 0.452, 0.328): the parked
    arm crosses its refill column. Moved under gh_top's working box instead, the head does."""
    import check_line
    from conftest import _slide_at_place
    _slide_at_place(0.5, 0.5)(pass_raw)
    pass_raw["feeders"]["fd_mid_plate"]["offset"] = [-0.05, 0.33, 0.15, 0.0]
    r = next(r for r in check_line.run_checks(pass_raw)[0] if r.name == "refill_clearance")
    assert r.status == "FAIL"
    assert any(m.startswith("scara_place: parked: ") and "fd_mid_plate's refill envelope "
               "(magazine)" in m for _, m in r.findings)
    pass_raw["feeders"]["fd_mid_plate"]["offset"] = [0.0, 0.0, 0.15, 0.0]
    r = next(r for r in check_line.run_checks(pass_raw)[0] if r.name == "refill_clearance")
    assert any(m.startswith("st_place: gh_top head (working) stands over fd_mid_plate's refill "
                            "envelope (magazine)") for _, m in r.findings), r.findings


def test_choose_park_never_eases_the_fold_for_a_magazine(pass_raw):
    """D-016: with fd_mid_plate (operator-refilled) under every folded park pose, the chooser
    names the station and the magazine instead of easing j2; the same footprint as a buffer
    a source refills is allowed to ease it (D-015)."""
    import robot_geometry as rg
    from conftest import _refilled_buffer_under_the_park
    _refilled_buffer_under_the_park(pass_raw)
    raw = yaml.safe_load(yaml.safe_dump(pass_raw))
    s = spec_mod.parse_spec(raw)
    assert "error" not in rg.choose_park(s, "scara_place", rg.CadHeights())
    del raw["sources"]["src_mid"]
    raw["feeders"]["fd_mid"] = {"type": "magazine", "part": "mid_plate", "station": "st_place",
                                "offset": raw["buffers"].pop("buf_mid")["offset"],
                                "capacity": 20, "refill_s": 60.0}
    raw["layout"]["feeder_footprints"]["fd_mid"] = raw["layout"]["feeder_footprints"].pop(
        "buf_mid")
    next(st for st in raw["sequence"] if st["id"] == "place")["from"] = "fd_mid"
    c = rg.choose_park(spec_mod.parse_spec(raw), "scara_place", rg.CadHeights())
    assert c.get("magazine_blocked") is True, c
    assert c["error"].startswith("scara_place: no park pose at j2 ") and "fd_mid" in c["error"]


def test_a_park_leg_runs_z_with_the_joints_and_a_place_goes_down_to_it():
    import robot_geometry as rg
    # place -> park, 4 samples: safe_z 0.2 at the place (down to the place z 0.05), top 0.6 at park.
    assert rg._leg_tcp_z("place_to_park", 0, 4, 0.2, 0.0, 0.05, 0.6) == pytest.approx((0.05, 0.3))
    assert rg._leg_tcp_z("place_to_park", 2, 4, 0.2, 0.0, 0.05, 0.6) == pytest.approx((0.3, 0.5))
    assert rg._leg_tcp_z("place_to_park", 4, 4, 0.2, 0.0, 0.05, 0.6) == pytest.approx((0.5, 0.6))
    assert rg._leg_tcp_z("park_to_pick", 4, 4, 0.2, 0.01, 0.05, 0.6) == pytest.approx((0.01, 0.3))
    assert rg._leg_tcp_z("pick_to_place", 2, 4, 0.2, 0.01, 0.05, 0.6) == pytest.approx((0.2, 0.2))


def test_choose_park_eases_the_fold_only_when_no_folded_pose_works(pass_raw):
    """With buf_mid under the folded park side, no j2-at-limit pose keeps the parked arm and its
    park legs out of the refill column; the chooser eases j2 and every rule then holds."""
    import robot_geometry as rg
    from conftest import _refilled_buffer_under_the_park
    _refilled_buffer_under_the_park(pass_raw)
    s = spec_mod.parse_spec(pass_raw)
    c = rg.choose_park(s, "scara_place", rg.CadHeights())
    assert "error" not in c, c
    t = rg.with_park(s, "scara_place", (c["j1"], c["j2"]))
    assert rg.park_clearance(t, "scara_place", (c["j1"], c["j2"]), rg.CadHeights())[
        "clearance_m"] >= rg.PARK_CLEAR_M
    assert rg.refill_clearance(t, "scara_place", rg.CadHeights())["clearance_m"] >= 0.0
    assert rg.path_clearance(t, "scara_place", rg.CadHeights())["clearance_m"] >= rg.PATH_CLEAR_M
    from conftest import _slide_at_place
    raw = yaml.safe_load((FIXTURES / "pass_spec.yaml").read_text())
    _slide_at_place(0.5, 0.5)(raw)
    assert rg.choose_park(spec_mod.parse_spec(raw), "scara_place", rg.CadHeights())[
        "fold_eased_deg"] == 0.0
