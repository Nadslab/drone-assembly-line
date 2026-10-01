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
import gantry_geometry as gg
import spec as spec_mod
from conftest import TOOLS

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
    code = layout.main(["--spec", str(spec), "--out", str(tmp_path / "out" / "line_layout.step"),
                        "--report", str(report), *extra])
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


def test_every_station_feeder_buffer_and_gantry_has_exactly_one_solid(clean):
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
        assert got.count(feeder) == (1 if f.get("pose") else 0), fid   # blow feeder: no pose
    for bid in raw["buffers"]:
        assert got.count("Buffer_" + layout.pascal(bid.removeprefix("buf_"))) == 1, bid
    for gid in raw["gantries"]:
        n = gid.removeprefix("gantry_")
        assert got.count(f"Gantry_{n}_Envelope") == 1
        assert got.count(f"Guard_Gantry_{n}") == 1
        assert sorted(x for x in got if x.startswith(f"Gantry_{n}_Post_")) == \
            [f"Gantry_{n}_Post_{k}" for k in range(1, 5)]
        assert sorted(x for x in got if x.startswith(f"Gantry_{n}_Beam_")) == \
            [f"Gantry_{n}_Beam_1", f"Gantry_{n}_Beam_2"]
    assert len(got) == len(set(got))


def test_the_step_keeps_every_name_and_the_hierarchy(clean):
    _, _, doc = clean
    assert doc["verify"]["hierarchy_preserved"] is True
    assert doc["verify"]["reimported_solids"] == doc["counts"]["solids"] == len(doc["solids"])
    back = import_step(doc["outputs"]["step"])
    assert back.label == "Line"
    assert [c.label for c in back.children] == list(layout.GROUPS)
    gantries = next(c for c in back.children if c.label == "Gantries")
    assert sorted(c.label for c in gantries.children) == ["Gantry_1", "Gantry_2"]


def test_every_dimension_names_its_spec_key(clean):
    _, _, doc = clean
    post = next(s for s in doc["solids"] if s["name"] == "Gantry_1_Post_1")
    assert post["dims"]["size_x"]["spec_keys"] == ["layout.gantry_frame.post_size"]
    assert "gantries.gantry_1.mount[0]" in post["dims"]["centre_x"]["spec_keys"]
    guard = next(s for s in doc["solids"] if s["name"] == "Guard_Gantry_1")
    assert "GUARD_H" in guard["dims"]["size_z"]["constants"]
    assert all(not s["placeholder"] for s in doc["solids"])     # no overlay in this run


# ---------------------------------------------------------------- what it refuses

def test_a_feeder_standing_on_the_conveyor_fails_naming_both(tmp_path, pass_raw):
    raw = layout_raw(pass_raw)
    raw["feeders"]["fd_mid_plate"]["pose"] = [0.25, 0.0, 0.15, 0.0]
    code, doc = run(tmp_path, raw)
    assert code == 1
    assert doc["interference"]["result"] == "FAIL"
    pairs = [{f["a"], f["b"]} for f in doc["interference"]["failures"]]
    assert {"Conveyor", "Feeder_MidPlate"} in pairs
    assert all(f["volume_mm3"] > layout.VOLUME_TOL_MM3 for f in doc["interference"]["failures"])
    assert doc["passed"] is False


def test_a_guard_closer_than_the_posts_cuts_through_them(tmp_path, pass_raw):
    raw = layout_raw(pass_raw)
    raw["layout"]["guard_offset"] = 0.05                       # posts are 0.10 wide
    code, doc = run(tmp_path, raw)
    assert code == 1
    assert any("Guard_Gantry_1" in (f["a"], f["b"]) and "Gantry_1_Post_1" in (f["a"], f["b"])
               for f in doc["interference"]["failures"])


def test_two_adjacent_gantries_share_one_guard_and_nothing_interferes(tmp_path, pass_raw):
    """D-011: gantry_2 moved to x 1.0-1.7, 0.4 m after gantry_1's 0.0-0.6. Each guard stands
    guard_offset 0.3 outside its envelope, so the two walls would cross each other's posts; they
    become one guard around both envelopes, and the layout is clean."""
    raw = layout_raw(pass_raw)
    raw["gantries"]["gantry_2"]["mount"][0] = 1.0
    code, doc = run(tmp_path, raw)
    guards = [n for n in names(doc) if n.startswith("Guard_")]
    assert guards == ["Guard_Gantry_1_Gantry_2"]
    assert doc["interference"]["result"] == "PASS", doc["interference"]["failures"]
    assert doc["interference"]["failures"] == []
    assert code == 0
    g = next(s for s in doc["solids"] if s["name"] == "Guard_Gantry_1_Gantry_2")
    wall = layout.GUARD_T * 1000.0
    # x: gantry_1's lower envelope edge to gantry_2's upper one, plus offset and wall each side.
    assert g["bbox_mm"]["min"][0] == pytest.approx(0.0 - 300.0 - wall, abs=1e-6)
    assert g["bbox_mm"]["max"][0] == pytest.approx(1700.0 + 300.0 + wall, abs=1e-6)


def test_guards_that_do_not_touch_stay_separate(clean):
    _, _, doc = clean
    assert sorted(n for n in names(doc) if n.startswith("Guard_")) == ["Guard_Gantry_1",
                                                                      "Guard_Gantry_2"]


def test_a_chain_of_touching_guards_is_one_group():
    d = layout.Dim
    env = {g: [(d(x0), d(x0 + 0.5)), (d(0.0), d(0.5))]
           for g, x0 in (("gantry_1", 0.0), ("gantry_2", 1.0), ("gantry_3", 2.0),
                         ("gantry_4", 9.0))}
    assert layout.guard_groups(env, d(0.3), d(0.01)) == [["gantry_1", "gantry_2", "gantry_3"],
                                                          ["gantry_4"]]


def test_a_placeholder_solid_is_suffixed_but_keeps_its_category_colour(tmp_path, pass_raw):
    raw = layout_raw(pass_raw)
    raw["layout"]["guard_offset"] = None
    code, doc = run(tmp_path, raw, "--dev", overlay={"layout": {"guard_offset": 0.3}})
    assert code == 0, doc["errors"]
    labels = {s["name"]: s["label"] for s in doc["solids"]}
    assert labels["Guard_Gantry_1"] == "Guard_Gantry_1_PH"
    assert labels["Conveyor"] == "Conveyor"
    assert doc["placeholders"] == ["layout.guard_offset"]
    assert "/dev/" in doc["outputs"]["step"]
    guard = find(import_step(doc["outputs"]["step"]), "Guard_Gantry_1_PH")
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


def test_the_layout_posts_are_the_posts_the_straddle_check_sees(clean):
    """layout.py draws the posts with its own arithmetic; gantry_geometry is what check_line's
    gantry_straddle judges. The two must be the same rectangles."""
    raw, _, doc = clean
    spec = spec_mod.parse_spec(raw)
    for gid in raw["gantries"]:
        n = gid.removeprefix("gantry_")
        for k, rect in enumerate(gg.post_rects(spec, gid), start=1):
            box = next(s for s in doc["solids"] if s["name"] == f"Gantry_{n}_Post_{k}")["bbox_mm"]
            assert [box["min"][0], box["max"][0], box["min"][1], box["max"][1]] == \
                pytest.approx([v * 1000.0 for v in rect.doc()], abs=1e-3)


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
    assert alphas["Guard_Gantry_1_PH"] == 1.0
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
    expected = {"Floor", "OperatorAisle"}
    for gid in raw["gantries"]:
        n = gid.removeprefix("gantry_")
        expected.add(f"Guard_Gantry_{n}")
        expected.add(f"Gantry_{n}_Envelope")
    assert got == expected
    gantries = next(c for c in back.children if c.label == "Gantries")
    for sub in gantries.children:
        assert all(not c.label.endswith("_Envelope") for c in sub.children)  # posts/beams only


def test_no_step_solid_is_coloured_orange(clean):
    _, _, doc = clean
    orange = tuple(round(c, 2) for c in layout.PLACEHOLDER_COLOUR)
    for step_path in (doc["outputs"]["step"], doc["outputs"]["step_physical"]):
        colours = {s.label: tuple(round(c, 2) for c in tuple(s.color)[:3])
                  for s in layout.leaves(import_step(str(step_path)))}
        assert orange not in colours.values(), (step_path, colours)
