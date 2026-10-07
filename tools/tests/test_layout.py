"""tools/cad/layout.py: one solid per spec entity, provenance per dimension, and an interference
check that fails on the overlaps it was not told to allow. The pass spec carries its own layout.
"""

import copy
import json
import sys
from pathlib import Path

import pytest
import yaml

import check_line
import robot_geometry as rg
import spec as spec_mod
from conftest import FIXTURES, TOOLS

sys.path.insert(0, str(TOOLS / "cad"))
import layout  # noqa: E402
from build123d import import_step  # noqa: E402

def layout_raw(pass_raw):
    return copy.deepcopy(pass_raw)


def run(tmp_path, raw, *extra, overlay=None):
    spec = tmp_path / "line_spec.yaml"
    spec.write_text(yaml.safe_dump(raw))
    if overlay is not None:
        (tmp_path / "line_spec.dev.yaml").write_text(yaml.safe_dump(overlay))
    report = tmp_path / "layout.json"
    cad = FIXTURES / "cad"
    if "--dev" in extra:                    # a dev run reads cad_<part>.dev.json: the same parts
        cad = tmp_path / "cad"
        cad.mkdir(exist_ok=True)
        for f in (FIXTURES / "cad").glob("cad_*.json"):
            (cad / f"{f.stem}.dev.json").write_text(f.read_text())
    code = layout.main(["--spec", str(spec), "--out", str(tmp_path / "out" / "line_layout.step"),
                        "--report", str(report), "--cad-reports", str(cad), "--no-render", *extra])
    return code, json.loads(report.read_text())


@pytest.fixture(scope="module")
def clean(tmp_path_factory, request):
    import conftest
    pass_raw = yaml.safe_load((conftest.FIXTURES / "pass_spec.yaml").read_text())
    raw = layout_raw(pass_raw)
    code, doc = run(tmp_path_factory.mktemp("layout"), raw)
    return raw, code, doc


def names(doc):
    return [s["name"] for s in doc["solids"]]


def find(node, label):
    """A labelled leaf of a re-imported assembly (`.solids()` flattens and drops labels)."""
    if node.label == label:
        return node
    for child in getattr(node, "children", ()) or ():
        hit = find(child, label)
        if hit is not None:
            return hit
    return None


# ---------------------------------------------------------------- one solid per entity

def test_the_clean_layout_passes(clean):
    _, code, doc = clean
    assert code == 0, doc["errors"]
    assert doc["interference"]["result"] == "PASS"
    assert doc["interference"]["allowed_overlaps"] > 0      # pallets on the belt, at least


def scara_name(sid):
    return "Scara_" + layout.pascal(sid.removeprefix("scara_"))


def test_every_station_feeder_buffer_and_robot_has_exactly_one_solid(clean):
    raw, _, doc = clean
    got = names(doc)
    for sid, st in raw["stations"].items():
        name = "Station_" + layout.pascal(sid.removeprefix("st_"))
        assert got.count(f"Pallet_{st['index']}") == 1, sid
        if st["count"] > 1:
            assert name not in got
            assert [n for n in got if n.startswith(f"{name}_Cell")] == \
                [f"{name}_Cell{k}" for k in range(1, st["count"] + 1)]
        else:
            assert got.count(name) == 1, sid
    for fid, f in raw["feeders"].items():
        feeder = "Feeder_" + layout.pascal(fid.removeprefix("fd_"))
        assert got.count(feeder) == (1 if f.get("offset") else 0), fid   # blow feeder: no pose
    for bid in raw["buffers"]:
        assert got.count("Buffer_" + layout.pascal(bid.removeprefix("buf_"))) == 1, bid
    for sid in raw["scaras"]:
        assert got.count(f"{scara_name(sid)}_Base") == 1
        assert got.count(f"{scara_name(sid)}_Envelope") == 1
    for rid in raw["return_paths"]:
        name = layout.pascal(rid.removeprefix("rt_"))
        assert got.count(f"ReturnPath_{name}") == got.count(f"ReturnSink_{name}") == 1
    assert got.count("GangHead_Top") == 1
    assert "GangHead_Top_Slide" not in got                     # no SCARA at st_drive: no slide
    assert sorted(n for n in got if n.startswith("Guard_")) == ["Guard_Fence",
                                                                "Guard_Partition_Front_Back"]
    assert len(got) == len(set(got))


def test_the_step_keeps_every_name_and_the_hierarchy(clean):
    raw, _, doc = clean
    assert doc["verify"]["hierarchy_preserved"] is True
    assert doc["verify"]["reimported_solids"] == doc["counts"]["solids"] == len(doc["solids"])
    back = import_step(doc["outputs"]["step"])
    assert back.label == "Line"
    assert [c.label for c in back.children] == list(layout.GROUPS)
    robots = next(c for c in back.children if c.label == "Robots")
    assert sorted(c.label for c in robots.children) == sorted(
        [f"{scara_name(s)}_Base" for s in raw["scaras"]] + ["GangHead_Top"])


def test_every_dimension_names_its_spec_key(clean):
    _, _, doc = clean
    base = next(s for s in doc["solids"] if s["name"] == "Scara_Place_Base")
    assert base["dims"]["size_x"]["spec_keys"] == ["layout.scara_base.diameter"]
    # The base is its resolved pose (D-013): the offset in its station's frame.
    assert base["dims"]["centre_y"]["spec_keys"] == ["scaras.scara_place.pose[1]"]
    env = next(s for s in doc["solids"] if s["name"] == "Scara_Place_Envelope")
    assert env["dims"]["size_x"]["spec_keys"] == ["robot_models.test_scara.reach"]
    assert set(env["dims"]["inner_x"]["spec_keys"]) == {
        "robot_models.test_scara.link1", "robot_models.test_scara.link2",
        "robot_models.test_scara.joint_limits.j2"}
    head = next(s for s in doc["solids"] if s["name"] == "GangHead_Top")
    assert "gang_heads.gh_top.stroke" in head["dims"]["centre_z"]["spec_keys"]
    guard = next(s for s in doc["solids"] if s["name"] == "Guard_Fence")
    assert "GUARD_H" in guard["dims"]["size_z"]["constants"]
    assert all(not s["placeholder"] for s in doc["solids"])     # no overlay in this run


def test_the_base_and_envelope_are_where_the_clearance_check_sees_them(clean):
    """layout.py draws the base with its own Dim arithmetic; spec.scara_base is what
    scara_clearance and scara_reach judge. The two must agree."""
    raw, _, doc = clean
    spec = spec_mod.parse_spec(raw)
    r = raw["layout"]["scara_base"]["diameter"] / 2.0
    for sid in raw["scaras"]:
        bx, by, _ = spec_mod.scara_base(spec, sid)
        box = next(s for s in doc["solids"] if s["name"] == f"{scara_name(sid)}_Base")["bbox_mm"]
        assert [box["min"][0], box["max"][0], box["min"][1], box["max"][1]] == pytest.approx(
            [(bx - r) * 1000, (bx + r) * 1000, (by - r) * 1000, (by + r) * 1000], abs=1e-2)
        env = next(s for s in doc["solids"]
                   if s["name"] == f"{scara_name(sid)}_Envelope")["bbox_mm"]
        model = raw["robot_models"][raw["scaras"][sid]["model"]]
        assert env["size"][0] == pytest.approx(2 * model["reach"] * 1000, abs=1e-2)
        assert (env["min"][2], env["max"][2]) == pytest.approx(
            (0.0, model["z_stroke"] * 1000), abs=1e-2)


def test_the_gang_head_box_is_robot_geometrys(clean):
    raw, _, doc = clean
    hb = rg.head_box(spec_mod.parse_spec(raw), "gh_top", rg.CadHeights(FIXTURES / "cad"))
    box = next(s for s in doc["solids"] if s["name"] == "GangHead_Top")["bbox_mm"]
    assert (box["min"][2], box["max"][2]) == pytest.approx((hb.z0 * 1000, hb.z1 * 1000), abs=1e-2)
    assert [box["min"][0], box["max"][0]] == pytest.approx([hb.rect.x0 * 1000, hb.rect.x1 * 1000],
                                                           abs=1e-2)


def test_a_head_on_a_slide_waits_opposite_its_scara_with_a_slide_envelope(tmp_path, pass_raw):
    from conftest import _slide_at_place
    raw = layout_raw(pass_raw)
    _slide_at_place(0.5, 0.5)(raw)
    code, doc = run(tmp_path, raw)
    head = next(s for s in doc["solids"] if s["name"] == "GangHead_Top")["bbox_mm"]
    slide = next(s for s in doc["solids"] if s["name"] == "GangHead_Top_Slide")
    assert slide["group"] == "Reference" and slide["category"] == "envelope"
    # scara_place is on +y, so the head waits 0.30 m out on -y: y -0.40 .. -0.20.
    assert (head["min"][1], head["max"][1]) == pytest.approx((-400.0, -200.0), abs=1e-2)
    assert (slide["bbox_mm"]["min"][1], slide["bbox_mm"]["max"][1]) == pytest.approx(
        (-400.0, 100.0), abs=1e-2)
    assert code == 0, doc["errors"]


def test_a_head_on_a_slide_has_a_rail_and_a_post_in_the_robots_group(tmp_path, pass_raw):
    """D-014: the rail runs from the working head's outer face (y -0.1) past the waiting head to
    the far face of the post (y -0.5); the post stands on the floor. The head riding in its rail
    and the rail on its post are allowed; both are in the interference check otherwise."""
    from conftest import _slide_at_place
    raw = layout_raw(pass_raw)
    _slide_at_place(0.5, 0.5)(raw)
    code, doc = run(tmp_path, raw)
    assert code == 0, doc["errors"]
    rail = next(s for s in doc["solids"] if s["name"] == "GangHead_Top_Rail")
    post = next(s for s in doc["solids"] if s["name"] == "GangHead_Top_Post")
    assert rail["group"] == post["group"] == "Robots"
    assert rail["category"] == post["category"] == "slide_frame"
    assert (rail["bbox_mm"]["min"][1], rail["bbox_mm"]["max"][1]) == pytest.approx(
        (-500.0, -100.0), abs=1e-2)
    assert post["bbox_mm"]["min"][2] == pytest.approx(-raw["line"]["work_height"] * 1000.0)
    assert "gang_heads.gh_top.slide.frame.post[0]" in post["dims"]["size_x"]["spec_keys"]
    assert "gang_head ∩ slide_frame (same gang head)" in doc["interference"]["allowed_rules"]
    assert doc["park"]["st_place"]["capsule_clearance_m"] == pytest.approx(0.110, abs=1e-4)
    assert doc["park"]["st_place"]["capsule_clearance_3d_m"] == pytest.approx(0.3432, abs=1e-4)


def test_a_slide_frame_on_something_else_is_an_interference(tmp_path, pass_raw):
    """Same-owner only: fd_mid_plate moved onto gh_top's post is an interference."""
    from conftest import _slide_at_place
    raw = layout_raw(pass_raw)
    _slide_at_place(0.5, 0.5)(raw)
    raw["feeders"]["fd_mid_plate"]["offset"] = [0.0, -0.45, 0.15, 0.0]
    code, doc = run(tmp_path, raw)
    assert code == 1
    pairs = [{f["a"], f["b"]} for f in doc["interference"]["failures"]]
    assert {"GangHead_Top_Post", "Feeder_MidPlate"} in pairs


def test_queue_slots_are_drawn_at_slot_pitch(clean):
    """One slot after each station (0.5 m gaps, 0.25 m pitch), 0.25 m downstream of it."""
    raw, _, doc = clean
    slots = {s["name"]: s["bbox_mm"] for s in doc["solids"] if s["name"].startswith("Pallet_Slot")}
    assert sorted(slots) == [f"Pallet_Slot_{n}_1" for n in range(1, 6)]
    centre = (slots["Pallet_Slot_1_1"]["min"][0] + slots["Pallet_Slot_1_1"]["max"][0]) / 2.0
    assert centre == pytest.approx(250.0, abs=1e-3)


def test_one_fence_round_the_line_and_a_partition_per_milestone_boundary(clean):
    """D-013: the fence runs guard_offset outside everything worked at — from buf_plate's
    footprint (x -0.275) to rt_fixture's put-down end (2.7 + 0.075) — and a partition stands
    half-way between st_drive (1.0) and st_test (1.5), the front | back boundary."""
    _, _, doc = clean
    wall = layout.GUARD_T * 1000.0
    fence = next(s for s in doc["solids"] if s["name"] == "Guard_Fence")["bbox_mm"]
    part = next(s for s in doc["solids"] if s["name"] == "Guard_Partition_Front_Back")["bbox_mm"]
    assert (fence["min"][0], fence["max"][0]) == pytest.approx(
        (-275.0 - 300.0 - wall, 2775.0 + 300.0 + wall), abs=1e-3)
    assert (part["min"][0] + part["max"][0]) / 2.0 == pytest.approx(1250.0, abs=1e-3)
    assert (part["min"][1], part["max"][1]) == pytest.approx(
        (fence["min"][1] + wall, fence["max"][1] - wall), abs=1e-3)


def test_a_return_chute_leaves_through_a_tunnel_and_its_sink_stands_outside(clean):
    _, code, doc = clean
    assert code == 0, doc["errors"]
    fence = next(s for s in doc["solids"] if s["name"] == "Guard_Fence")["bbox_mm"]
    sink = next(s for s in doc["solids"] if s["name"] == "ReturnSink_Fixture")["bbox_mm"]
    assert sink["min"][0] > fence["max"][0]
    assert not any("ends inside the fence" in w for w in doc["warnings"])


def test_a_milestone_is_drawn_on_its_own(tmp_path, pass_raw):
    code, doc = run(tmp_path, layout_raw(pass_raw), "--milestone", "back")
    assert code == 0, doc["errors"]
    got = names(doc)
    assert doc["milestone"] == "back" and "/out/back/" in doc["outputs"]["step"]
    assert {"Scara_Spider_Base", "Scara_Pod_Base", "Buffer_Fixture", "Buffer_Pod",
            "Guard_Fence", "ReturnPath_Fixture"} <= set(got)
    assert not {"Scara_Place_Base", "GangHead_Top", "Feeder_MidPlate", "Station_Drive"} \
        & set(got)
    assert not [n for n in got if n.startswith("Guard_Partition")]      # one milestone
    conveyor = next(s for s in doc["solids"] if s["name"] == "Conveyor")["bbox_mm"]
    # st_test (1.5) to st_pod (2.5), half a pitch beyond each end.
    assert (conveyor["min"][0], conveyor["max"][0]) == pytest.approx((1250.0, 2750.0), abs=1e-3)


def test_a_station_run_across_a_milestone_boundary_keeps_its_partition(tmp_path, pass_raw):
    """D-015 --stations: st_drive and st_test, one from each milestone, drawn into out/batch/
    with the front | back partition between them; nothing from the other stations."""
    code, doc = run(tmp_path, layout_raw(pass_raw), "--stations", "st_drive,st_test", "--name",
                    "b1", "--no-render")
    assert code == 0, doc["errors"]
    got = names(doc)
    assert doc["stations"] == ["st_drive", "st_test"] and "/out/batch/" in doc["outputs"]["step"]
    assert "Guard_Partition_Front_Back" in got and "GangHead_Top" in got
    assert not {"Scara_Place_Base", "Scara_Spider_Base", "Feeder_MidPlate"} & set(got)


# ---------------------------------------------------------------- what it refuses

def test_a_feeder_standing_on_the_conveyor_fails_naming_both(tmp_path, pass_raw):
    raw = layout_raw(pass_raw)
    raw["feeders"]["fd_mid_plate"]["offset"] = [-0.25, 0.0, 0.15, 0.0]
    code, doc = run(tmp_path, raw)
    assert code == 1
    assert doc["interference"]["result"] == "FAIL"
    pairs = [{f["a"], f["b"]} for f in doc["interference"]["failures"]]
    assert {"Conveyor", "Feeder_MidPlate"} in pairs
    assert all(f["volume_mm3"] > layout.VOLUME_TOL_MM3 for f in doc["interference"]["failures"])
    assert doc["passed"] is False


def test_a_partition_inside_an_envelopes_offset_is_warned(tmp_path, pass_raw):
    """The partition stands half-way between the boundary stations; a guard_offset that makes
    it reach into either one's reach envelope is said, not drawn away."""
    raw = layout_raw(pass_raw)
    raw["layout"]["guard_offset"] = 0.35
    code, doc = run(tmp_path, raw)
    assert code == 0, doc["errors"]
    assert any(w.startswith("partition front|back") for w in doc["warnings"])


def test_a_placeholder_solid_is_suffixed_but_keeps_its_category_colour(tmp_path, pass_raw):
    raw = layout_raw(pass_raw)
    raw["layout"]["guard_offset"] = None
    code, doc = run(tmp_path, raw, "--dev", overlay={"layout": {"guard_offset": 0.3}})
    assert code == 0, doc["errors"]
    labels = {s["name"]: s["label"] for s in doc["solids"]}
    assert labels["Guard_Fence"] == "Guard_Fence_PH"
    assert labels["Conveyor"] == "Conveyor"
    assert doc["placeholders"] == ["layout.guard_offset"]
    assert "/dev/" in doc["outputs"]["step"]
    guard = find(import_step(doc["outputs"]["step"]), "Guard_Fence_PH")
    assert tuple(round(c, 2) for c in tuple(guard.color)[:3]) == \
        tuple(round(c, 2) for c in layout.colour_of("guard"))
    assert tuple(round(c, 2) for c in tuple(guard.color)[:3]) != \
        tuple(round(c, 2) for c in layout.PLACEHOLDER_COLOUR)


def test_a_null_layout_value_fails_naming_the_key(tmp_path, pass_raw):
    raw = layout_raw(pass_raw)
    raw["layout"]["conveyor"]["width"] = None
    code, doc = run(tmp_path, raw)
    assert code == 1
    assert "layout.conveyor.width" in doc["missing"]
    assert not (tmp_path / "out").exists()


def test_no_layout_section_fails(tmp_path, pass_raw):
    del pass_raw["layout"]
    code, doc = run(tmp_path, pass_raw)
    assert code == 1
    assert "no `layout:` section" in doc["errors"][0]


def test_a_footprint_for_no_feeder_or_buffer_is_a_schema_failure(pass_raw):
    raw = layout_raw(pass_raw)
    raw["layout"]["feeder_footprints"]["fd_nowhere"] = [0.1, 0.1, 1.0]
    schema = {r.name: r for r in check_line.run_checks(raw)[0]}["schema"]
    assert schema.status == "FAIL"
    assert any("fd_nowhere" in m for _, m in schema.findings)


# ---------------------------------------------------------------- opacity

def leaf_alphas(step):
    return {s.label: tuple(s.color)[3] for s in layout.leaves(import_step(str(step)))}


def test_every_re_imported_solid_is_fully_opaque(clean):
    _, _, doc = clean
    alphas = leaf_alphas(doc["outputs"]["step"])
    assert len(alphas) == doc["counts"]["solids"]
    assert set(alphas.values()) == {1.0}
    assert any(n.endswith("_Envelope") for n in alphas)          # envelopes too, not just bodies
    assert doc["verify"]["opacity"]["opaque"] is True


def test_placeholder_solids_are_opaque_too(tmp_path, pass_raw):
    raw = layout_raw(pass_raw)
    raw["layout"]["guard_offset"] = None
    code, doc = run(tmp_path, raw, "--dev", overlay={"layout": {"guard_offset": 0.3}})
    assert code == 0, doc["errors"]
    alphas = leaf_alphas(doc["outputs"]["step"])
    assert alphas["Guard_Fence_PH"] == 1.0
    assert set(alphas.values()) == {1.0}


def test_the_file_states_opacity_for_every_solid(clean):
    """OCCT writes no opacity at alpha 1.0; the file must carry it, all zero (opaque)."""
    _, _, doc = clean
    text = open(doc["outputs"]["step"]).read()
    values = layout.TRANSPARENT_ENTITY.findall(text)
    assert len(values) >= doc["counts"]["solids"]
    assert {float(v) for v in values} == {0.0}


def test_a_translucent_solid_is_caught_by_the_file_not_by_the_re_import(tmp_path):
    """import_step does not read transparency back: Glass re-imports at alpha 1.0. Only the
    file's own SURFACE_STYLE_TRANSPARENT entities give it away, which is why they are checked."""
    from build123d import Box, Color, Compound, export_step
    a, b = Box(10, 10, 10), Box(5, 5, 5)
    a.label, b.label = "Opaque", "Glass"
    a.color, b.color = Color(1, 0, 0, 1.0), Color(0, 0, 1, 0.5)
    step = tmp_path / "t.step"
    export_step(Compound(children=[a, b], label="Line"), str(step))
    op = layout.opacity(step, import_step(str(step)))
    assert op["all_alpha_1"] is True                     # the blind spot, pinned
    assert op["transparent_entities_nonzero"] > 0 and op["opaque"] is False


def test_an_alpha_1_export_states_no_opacity_so_it_is_not_accepted(tmp_path):
    """The file this task started from: colours, but no opacity entity anywhere."""
    from build123d import Box, Color, Compound, export_step
    a = Box(10, 10, 10)
    a.label, a.color = "A", Color(1, 0, 0, 1.0)
    step = tmp_path / "t.step"
    export_step(Compound(children=[a], label="Line"), str(step))
    op = layout.opacity(step, import_step(str(step)))
    assert op["all_alpha_1"] is True and op["transparent_entities"] == 0
    assert op["file_states_opacity"] is False and op["opaque"] is False


def test_write_step_makes_a_translucent_export_opaque_and_restores_the_colours(tmp_path):
    from build123d import Box, Color, Compound
    a = Box(10, 10, 10)
    a.label, a.color = "A", Color(0.2, 0.4, 0.6, 1.0)
    line = Compound(children=[a], label="Line")
    step = tmp_path / "t.step"
    layout.write_step(line, step)
    assert leaf_alphas(step) == {"A": 1.0}
    assert tuple(a.color) == pytest.approx((0.2, 0.4, 0.6, 1.0))    # the probe alpha did not leak


# ---------------------------------------------------------------- axes, Reference, two files

def test_the_export_rotates_spec_xyz_to_step_x_z_negy(clean):
    """SolidWorks is Y-up: a station at spec (x, y, z) appears at (x, z, -y) in the STEP."""
    _, _, doc = clean
    floor = next(s for s in doc["solids"] if s["name"] == "Floor")["bbox_mm"]
    cx = (floor["min"][0] + floor["max"][0]) / 2.0
    cy = (floor["min"][1] + floor["max"][1]) / 2.0
    cz = (floor["min"][2] + floor["max"][2]) / 2.0
    leaf = find(import_step(doc["outputs"]["step"]), "Floor")
    b = leaf.bounding_box()
    got = ((b.min.X + b.max.X) / 2.0, (b.min.Y + b.max.Y) / 2.0, (b.min.Z + b.max.Z) / 2.0)
    assert got == pytest.approx((cx, cz, -cy), abs=1e-2)


def test_both_step_files_are_always_written(clean):
    _, _, doc = clean
    step, physical = Path(doc["outputs"]["step"]), Path(doc["outputs"]["step_physical"])
    assert step.exists() and physical.exists()
    assert physical.name == "line_layout_physical.step"
    assert doc["verify_physical"]["hierarchy_preserved"] is True
    assert doc["verify_physical"]["opacity"]["opaque"] is True


def test_the_physical_step_has_no_reference_group(clean):
    _, _, doc = clean
    back = import_step(doc["outputs"]["step_physical"])
    assert [c.label for c in back.children] == list(layout.PHYSICAL_GROUPS)
    assert "Reference" not in [c.label for c in back.children]


def test_the_reference_group_holds_exactly_the_non_physical_solids(clean):
    raw, _, doc = clean
    back = import_step(doc["outputs"]["step"])
    reference = next(c for c in back.children if c.label == "Reference")
    got = {c.label for c in reference.children}
    expected = {"Floor", "OperatorAisle", "Guard_Fence", "Guard_Partition_Front_Back"}
    expected |= {f"{scara_name(sid)}_Envelope" for sid in raw["scaras"]}
    assert got == expected
    robots = next(c for c in back.children if c.label == "Robots")
    assert all(not c.label.endswith(("_Envelope", "_Slide")) for c in robots.children)


def test_robots_are_teal_and_gang_heads_orange_brown(clean):
    _, _, doc = clean
    colours = {s.label: tuple(round(c, 2) for c in tuple(s.color)[:3])
               for s in layout.leaves(import_step(doc["outputs"]["step"]))}
    assert colours["Scara_Place_Base"] == tuple(round(c, 2) for c in layout.colour_of("robot"))
    assert colours["GangHead_Top"] == tuple(round(c, 2) for c in layout.colour_of("gang_head"))
    assert colours["Scara_Place_Base"] != colours["GangHead_Top"]


def test_no_step_solid_is_coloured_orange(clean):
    _, _, doc = clean
    orange = tuple(round(c, 2) for c in layout.PLACEHOLDER_COLOUR)
    for step_path in (doc["outputs"]["step"], doc["outputs"]["step_physical"]):
        colours = {s.label: tuple(round(c, 2) for c in tuple(s.color)[:3])
                  for s in layout.leaves(import_step(str(step_path)))}
        assert orange not in colours.values(), (step_path, colours)
