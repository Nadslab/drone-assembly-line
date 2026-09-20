"""P4 B3-B4: the spider carrier and the pod tray are computed from the spec, not drawn.

The load-bearing assertions are in `test_every_pocket_centre_comes_from_its_own_holes`: each arm
pocket is measured off the cutter the build actually used, and pinned to the arm's own screw holes
in `parts.bottom_plate.arm_holes` to 0.01 mm — by geometry (distance, collinearity, which way is
outboard), not by re-running the generator's arithmetic. `test_carrier_fits_the_gantry_z_travel`
is the other one: a fixture the gantry cannot lift clear is not a fixture.
"""

import copy
import json
import math
import random

import pytest
import yaml

import spec as spec_mod
from cad import gripper, pod_tray, spider_carrier

TOL_MM = 0.01
GENERATORS = {"carrier_spider": spider_carrier, "carrier_pod": pod_tray}


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    for module in GENERATORS.values():
        monkeypatch.setattr(module, "report_path",
                            lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json")


@pytest.fixture
def work(tmp_path, pass_raw):
    """(write_spec, run, report): a spec file, the two CLIs, and whatever they wrote."""
    spec = tmp_path / "line_spec.yaml"

    def write(raw=None):
        spec.write_text(yaml.safe_dump(raw if raw is not None else pass_raw))
        return spec

    def run(part="carrier_spider", *extra):
        return GENERATORS[part].main(["--spec", str(write()), "--out", str(tmp_path / "out"),
                                      *extra])

    def report(part="carrier_spider", dev=False):
        return json.loads((tmp_path / f"cad_{part}{'.dev' if dev else ''}.json").read_text())

    write()
    return write, run, report


def run_raw(tmp_path, raw, part="carrier_spider", *extra):
    """Run one generator over `raw` and return (exit code, report)."""
    spec = tmp_path / "line_spec.yaml"
    spec.write_text(yaml.safe_dump(raw))
    code = GENERATORS[part].main(["--spec", str(spec), "--out", str(tmp_path / "out"), *extra])
    name = f"cad_{part}{'.dev' if '--dev' in extra else ''}.json"
    return code, json.loads((tmp_path / name).read_text())


def plate_centre(raw):
    d = raw["parts"]["bottom_plate"]["datums"]
    return tuple((d["round"][i] + d["diamond"][i]) / 2.0 * 1000.0 for i in (0, 1))


def radial_holes(centre, arms, per_arm, radii, start_deg=45.0):
    """A hole pattern laid out by hand: `arms` arms, `per_arm` holes each, on equal angles."""
    cx, cy = centre
    out = []
    for a in range(arms):
        theta = math.radians(start_deg + a * 360.0 / arms)
        for r in radii[:per_arm]:
            out.append([cx + r * math.cos(theta), cy + r * math.sin(theta)])
    return out


# ---------------------------------------------------------------- the arm pattern

def test_every_pocket_centre_comes_from_its_own_holes(work, pass_raw):
    """A pocket is its arm's screw holes, walked out along the arm — nothing else.

    Checked as geometry rather than by repeating the generator's sum: the pocket centre lies on
    the line through the arm's own holes, exactly `arm_l/2 - hole_inset` from their centroid,
    on the outboard side.
    """
    _, run, report = work
    raw = pass_raw
    assert run() == 0
    d = report()["derived"]
    arm = raw["parts"]["arm"]
    offset = arm["dims"][0] * 1000.0 / 2.0 - arm["hole_inset"] * 1000.0
    centre = plate_centre(raw)

    spec_holes = {tuple(round(c * 1000.0, 6) for c in h)
                  for h in raw["parts"]["bottom_plate"]["arm_holes"]["positions"]}
    grouped = [tuple(h) for a in d["arms"] for h in a["holes_mm"]]
    assert len(grouped) == len(spec_holes)                     # every hole used exactly once
    assert {tuple(round(c, 6) for c in h) for h in grouped} == spec_holes

    for a in d["arms"]:
        holes, g, p = a["holes_mm"], a["hole_centre_mm"], a["pocket_centre_mm"]
        for axis in (0, 1):                                    # the centroid is the spec's
            assert g[axis] == pytest.approx(sum(h[axis] for h in holes) / len(holes), abs=TOL_MM)
        assert math.dist(p, g) == pytest.approx(abs(offset), abs=TOL_MM)
        (x1, y1), (x2, y2) = holes[0], holes[-1]               # pocket is on the holes' own line
        cross = (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1)
        assert abs(cross) / math.dist(holes[0], holes[-1]) <= TOL_MM
        assert math.dist(p, centre) > math.dist(g, centre)     # and it runs outboard


def test_moving_a_plate_hole_moves_only_its_own_pocket(work, tmp_path, pass_raw):
    """The pockets track the plate: change one screw position, one pocket follows it."""
    write, run, report = work
    assert run() == 0
    before = {a["name"]: a["pocket_centre_mm"] for a in report()["derived"]["arms"]}

    moved = copy.deepcopy(pass_raw)
    moved["parts"]["bottom_plate"]["arm_holes"]["positions"][0][1] += 0.004   # 4 mm
    assert run_raw(tmp_path, moved)[0] == 0
    after = {a["name"]: a["pocket_centre_mm"]
             for a in json.loads((tmp_path / "cad_carrier_spider.json").read_text())
             ["derived"]["arms"]}

    changed = [n for n in before if math.dist(before[n], after[n]) > TOL_MM]
    assert len(changed) == 1, f"{changed} moved, expected exactly one arm"


def test_grouping_ignores_the_order_the_holes_are_written_in(tmp_path, pass_raw):
    """Which holes belong to which arm is a fact about the pattern, not about the yaml."""
    shuffled = copy.deepcopy(pass_raw)
    random.Random(0).shuffle(shuffled["parts"]["bottom_plate"]["arm_holes"]["positions"])
    codes_reports = [run_raw(tmp_path, raw) for raw in (pass_raw, shuffled)]
    assert [c for c, _ in codes_reports] == [0, 0]
    assert codes_reports[0][1]["derived"]["arms"] == codes_reports[1][1]["derived"]["arms"]


@pytest.mark.parametrize("arms, per_arm, radii", [(3, 2, [60.0, 80.0]),
                                                  (4, 1, [70.0]),
                                                  (6, 2, [55.0, 75.0])])
def test_the_pattern_decides_how_many_pockets_there_are(tmp_path, pass_raw, arms, per_arm, radii):
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["bottom_plate"]["arm_holes"] = {
        "per_arm": per_arm,
        "positions": [[x / 1000.0, y / 1000.0]
                      for x, y in radial_holes(plate_centre(raw), arms, per_arm, radii)]}
    code, doc = run_raw(tmp_path, raw)
    assert code == 0, doc["errors"]
    got = doc["derived"]["arms"]
    assert len(got) == arms
    assert [len(a["holes_mm"]) for a in got] == [per_arm] * arms
    assert sorted(round(a["axis_deg"]) for a in got) == sorted(
        round(math.degrees(math.atan2(math.sin(t), math.cos(t))))
        for t in (math.radians(45.0 + i * 360.0 / arms) for i in range(arms)))


def test_holes_on_the_plate_centre_fail_instead_of_guessing_a_direction(tmp_path, pass_raw):
    raw = copy.deepcopy(pass_raw)
    centre = [c / 1000.0 for c in plate_centre(raw)]
    raw["parts"]["bottom_plate"]["arm_holes"]["positions"][:2] = [list(centre), list(centre)]
    code, doc = run_raw(tmp_path, raw)
    assert code == 1
    assert "arm_holes" in doc["errors"][0]


# ---------------------------------------------------------------- what the carrier must clear

def test_carrier_fits_the_gantry_z_travel(work):
    write, run, report = work
    assert run() == 0
    doc = report()
    assert doc["bbox_mm"]["size"][2] <= doc["derived"]["gantry_z_travel_mm"]


@pytest.mark.parametrize("part", ["carrier_spider", "carrier_pod"])
def test_a_fixture_taller_than_the_z_travel_fails(tmp_path, pass_raw, part):
    raw = copy.deepcopy(pass_raw)
    raw["gantry"]["axes"]["z"]["max"] = 0.05        # 50 mm of travel
    code, doc = run_raw(tmp_path, raw, part)
    assert code == 1
    assert "gantry.axes.z" in doc["errors"][-1]


def test_the_arms_are_carried_on_ring_material_not_thin_air(work):
    """The pocket reaches inboard over the ring's opening; what carries the arm is the run of
    it that is actually backed by ring."""
    write, run, report = work
    assert run() == 0
    for arm in report()["derived"]["arms"]:
        assert arm["supported_run_mm"] >= spider_carrier.ARM_SUPPORT_MIN


def test_an_esc_below_the_mid_plate_fails(tmp_path, pass_raw):
    """The carrier holds the ESC above the mid-plate line so the mid plate can come in under it."""
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["esc"]["z_in_stack"] = raw["parts"]["mid_plate"]["z_in_stack"]
    code, doc = run_raw(tmp_path, raw)
    assert code == 1
    assert "parts.esc.z_in_stack" in doc["errors"][0]


def test_the_esc_nest_holds_the_esc_at_its_final_height(work, pass_raw):
    _, run, report = work
    raw = pass_raw
    assert run() == 0
    nest = report()["derived"]["esc_nest"]
    assert nest["floor_mm"] == pytest.approx(raw["parts"]["esc"]["z_in_stack"] * 1000.0,
                                             abs=TOL_MM)
    assert nest["pocket_mm"][:2] == pytest.approx(
        [raw["parts"]["esc"]["dims"][i] * 1000.0 + 2.0 * spider_carrier.NEST_CLEAR
         for i in (0, 1)], abs=TOL_MM)


# ---------------------------------------------------------------- gripper clearance

@pytest.mark.parametrize("part", ["carrier_spider", "carrier_pod"])
def test_nothing_sits_where_the_jaws_go(work, part):
    _, run, report = work
    assert run(part) == 0
    assert report(part)["derived"]["pick"]["keep_out_volume_mm3"] == 0.0


@pytest.mark.parametrize("part", ["carrier_spider", "carrier_pod"])
@pytest.mark.parametrize("dims, wanted", [([0.02, 0.02, 0.03], "jaws open"),
                                          ([0.06, 0.06, 0.20], "jaws reach")])
def test_a_tool_that_cannot_grip_the_boss_fails_naming_the_tool(tmp_path, pass_raw, part, dims,
                                                                wanted):
    raw = copy.deepcopy(pass_raw)
    raw["gantry"]["tools"]["tool_carrier"]["dims"] = dims
    code, doc = run_raw(tmp_path, raw, part)
    assert code == 1
    assert wanted in doc["errors"][0] and "gantry.tools.tool_carrier" in doc["errors"][0]


def test_the_keep_out_volume_is_measured_not_assumed():
    """gripper.keep_out is the jaw footprint over the last `reach` of the boss, less the boss."""
    d = gripper.GripDims(boss_d=20.0, boss_h=30.0, jaw_x=50.0, jaw_y=40.0, reach=10.0,
                         clearance=2.0)
    box = gripper.keep_out(d).bounding_box()
    assert (box.min.Z, box.max.Z) == pytest.approx((d.boss_h - d.reach, d.boss_h))
    assert (box.size.X, box.size.Y) == pytest.approx((d.jaw_x, d.jaw_y))
    assert gripper.keep_out(d).volume == pytest.approx(
        d.jaw_x * d.jaw_y * d.reach - math.pi * (d.boss_d / 2.0) ** 2 * d.reach, rel=1e-3)


def test_the_pick_boss_follows_a_pick_point_the_spec_states(tmp_path, pass_raw):
    raw = copy.deepcopy(pass_raw)
    cx, cy = plate_centre(raw)
    raw["parts"]["carrier_spider"]["pick"] = [(cx - 40.0) / 1000.0, cy / 1000.0, None, 0.0]
    code, doc = run_raw(tmp_path, raw)
    assert code == 0, doc["errors"]
    assert doc["derived"]["pick"]["pose_mm"][:2] == pytest.approx([cx - 40.0, cy], abs=TOL_MM)
    assert doc["derived"]["pick"]["keep_out_volume_mm3"] == 0.0     # still supported and clear


def test_a_pick_z_that_disagrees_with_the_built_boss_is_an_error(tmp_path, pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["carrier_pod"]["pick"] = [None, None, 0.5, 0.0]     # 500 mm, nowhere near
    code, doc = run_raw(tmp_path, raw, "carrier_pod")
    assert code == 1
    assert "parts.carrier_pod.pick[2]" in doc["errors"][0]


# ---------------------------------------------------------------- pod tray

def test_the_pod_pocket_is_the_pod_plus_a_fit_clearance(work, pass_raw):
    _, run, report = work
    assert run("carrier_pod") == 0
    pocket = report("carrier_pod")["derived"]["pocket"]
    pod = pass_raw["parts"]["pod"]["dims"]
    assert pocket["size_mm"][:2] == pytest.approx(
        [pod[i] * 1000.0 + 2.0 * pod_tray.POD_CLEAR for i in (0, 1)], abs=TOL_MM)
    assert pocket["size_mm"][2] == pytest.approx(pod[2] * 1000.0 + pod_tray.POD_CLEAR, abs=TOL_MM)


def test_the_tray_floor_is_the_top_of_the_esc(work, pass_raw):
    """The pod lands on the ESC, so the tray presents it there — no height of the tray's own."""
    _, run, report = work
    assert run("carrier_pod") == 0
    esc = pass_raw["parts"]["esc"]
    assert report("carrier_pod")["derived"]["pocket"]["floor_mm"] == pytest.approx(
        (esc["z_in_stack"] + esc["dims"][2]) * 1000.0, abs=TOL_MM)


# ---------------------------------------------------------------- CLI

@pytest.mark.parametrize("part, stem", [("carrier_spider", "spider_carrier"),
                                        ("carrier_pod", "pod_tray")])
def test_writes_step_stl_and_report(work, tmp_path, part, stem):
    _, run, report = work
    assert run(part) == 0
    step, stl = tmp_path / "out" / f"{stem}.step", tmp_path / "out" / f"{stem}.stl"
    assert step.is_file() and stl.is_file()
    doc = report(part)
    assert doc["passed"] is True and doc["missing"] == [] and doc["errors"] == []
    assert doc["outputs"] == {"step": str(step), "stl": str(stl)}
    assert doc["mass_kg"]["total"] > 0.0
    assert stl.stat().st_size < 5 * 1024 * 1024                  # P0: no STL over 5 MB
    assert doc["mesh"]["size_bytes"] == stl.stat().st_size
    assert doc["mesh"]["solid_bbox_drift_mm"] <= 0.2


@pytest.mark.parametrize("part, stem, material", [
    ("carrier_spider", "spider_carrier", spider_carrier.MATERIAL),
    ("carrier_pod", "pod_tray", pod_tray.MATERIAL)])
def test_the_material_note_travels_in_the_step_header(work, tmp_path, part, stem, material):
    """P4 B3: the fixture's material is a decision, so it has to reach whoever opens the STEP."""
    _, run, _ = work
    assert run(part) == 0
    header = (tmp_path / "out" / f"{stem}.step").read_text()[:2000]
    assert material.split(":")[0] in header.replace("\n", " ")


@pytest.mark.parametrize("part, key, blank", [
    ("carrier_spider", "parts.arm.hole_inset",
     lambda raw: raw["parts"]["arm"].__setitem__("hole_inset", None)),
    ("carrier_spider", "parts.bottom_plate.arm_holes.positions[3][1]",
     lambda raw: raw["parts"]["bottom_plate"]["arm_holes"]["positions"][3].__setitem__(1, None)),
    ("carrier_pod", "parts.esc.dims[2]",
     lambda raw: raw["parts"]["esc"]["dims"].__setitem__(2, None))])
def test_null_spec_value_fails_and_names_the_key(tmp_path, pass_raw, part, key, blank, capsys):
    raw = copy.deepcopy(pass_raw)
    blank(raw)
    code, doc = run_raw(tmp_path, raw, part)
    assert code == 1
    assert key in doc["missing"]
    assert key in capsys.readouterr().err
    assert not (tmp_path / "out").exists()


@pytest.mark.parametrize("part", ["carrier_spider", "carrier_pod"])
def test_a_sequence_that_places_the_fixture_with_no_tool_fails(tmp_path, pass_raw, part):
    """The tool that lifts a fixture is the spec's to name, not the generator's to assume."""
    raw = copy.deepcopy(pass_raw)
    for step in raw["sequence"]:
        if step.get("part") == part:
            step.pop("tool", None)
    code, doc = run_raw(tmp_path, raw, part)
    assert code == 1
    assert any(part in m for m in doc["missing"])


@pytest.mark.parametrize("part, stem", [("carrier_spider", "spider_carrier"),
                                        ("carrier_pod", "pod_tray")])
def test_dev_run_is_quarantined_in_its_own_folder(tmp_path, pass_raw, part, stem):
    raw = copy.deepcopy(pass_raw)
    raw["parts"]["esc"]["z_in_stack"] = None
    (tmp_path / "line_spec.dev.yaml").write_text(
        yaml.safe_dump({"parts": {"esc": {"z_in_stack": 0.045}}}))
    code, doc = run_raw(tmp_path, raw, part, "--dev")
    assert code == 0, doc["errors"]
    assert (tmp_path / "out" / "dev" / f"{stem}.step").is_file()
    assert not (tmp_path / "out" / f"{stem}.step").exists()
    assert doc["dev"] is True and doc["placeholders"] == ["parts.esc.z_in_stack"]
    assert not (tmp_path / f"cad_{part}.json").exists()
