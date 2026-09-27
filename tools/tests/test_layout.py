"""tools/cad/layout.py: one solid per spec entity, provenance per dimension, and an interference
check that fails on the overlaps it was not told to allow.

The shared pass_spec's two gantries overlap in travel (both draw from buf_plate), so their frames
would stand in each other. `layout_raw` moves gantry_2 onto its own stations and gives its steps a
buffer of their own; everything else is the pass spec.
"""

import copy
import json
import sys

import pytest
import yaml

import check_line
from conftest import TOOLS

sys.path.insert(0, str(TOOLS / "cad"))
import layout  # noqa: E402
from build123d import import_step  # noqa: E402

LAYOUT = {
    "conveyor": {"width": 0.30, "top_height": 0.88, "belt_speed": 0.3},
    "pallet_block": {"length": 0.20, "width": 0.20, "thickness": 0.02},
    "stop_unit_length": 0.20,
    "buffer_slots_between_stations": 1,
    "gantry_frame": {"post_size": 0.10, "beam_height": 0.12, "beam_depth": 0.10},
    "feeder_footprints": {"fd_mid_plate": [0.15, 0.15, 1.05], "buf_plate": [0.15, 0.15, 1.0],
                          "buf_fixture": [0.15, 0.15, 1.0]},
    "test_branch": {"offset_y": -1.0, "cell_w": 0.30, "cell_d": 0.30},
    "operator_aisle": {"side": "+y", "width": 1.0},
    "guard_offset": 0.3,
}


def layout_raw(pass_raw):
    raw = copy.deepcopy(pass_raw)
    raw["layout"] = copy.deepcopy(LAYOUT)
    raw["buffers"]["buf_fixture"] = {"pose": [2.25, 0.3, 0.1, 0.0]}
    for step in raw["sequence"]:
        if step["id"] in ("place_spider", "place_pod"):
            step["from"] = "buf_fixture"
    raw["gantries"]["gantry_2"]["mount"] = [1.9, 0.0, 0.0]
    raw["gantries"]["gantry_2"]["axes"]["x"].update(min=0.0, max=0.7)
    return raw


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


def test_a_placeholder_solid_is_orange_and_suffixed(tmp_path, pass_raw):
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
    assert tuple(round(c, 2) for c in tuple(guard.color)[:3]) == layout.PLACEHOLDER_COLOUR


def test_a_null_layout_value_fails_naming_the_key(tmp_path, pass_raw):
    raw = layout_raw(pass_raw)
    raw["layout"]["conveyor"]["width"] = None
    code, doc = run(tmp_path, raw)
    assert code == 1
    assert "layout.conveyor.width" in doc["missing"]
    assert not (tmp_path / "out").exists()


def test_no_layout_section_fails(tmp_path, pass_raw):
    code, doc = run(tmp_path, pass_raw)
    assert code == 1
    assert "no `layout:` section" in doc["errors"][0]


def test_a_footprint_for_no_feeder_or_buffer_is_a_schema_failure(pass_raw):
    raw = layout_raw(pass_raw)
    raw["layout"]["feeder_footprints"]["fd_nowhere"] = [0.1, 0.1, 1.0]
    schema = {r.name: r for r in check_line.run_checks(raw)[0]}["schema"]
    assert schema.status == "FAIL"
    assert any("fd_nowhere" in m for _, m in schema.findings)
