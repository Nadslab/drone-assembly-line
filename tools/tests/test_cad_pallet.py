"""P4 B1-B2: the pallet and its locating pins are built from the spec, not from typed numbers.

The load-bearing assertion is `test_pin_axes_equal_the_spec_datums`: the pins are what the whole
main line is datumed to, so their XY must equal `parts.bottom_plate.datums` to 0.01 mm — measured
off the built solids, not read back from the inputs.
"""

import json

import pytest
import yaml
from build123d import Align, Box, Pos, Rot

import spec as spec_mod
from cad import pallet, pins

TOL_MM = 0.01
BIG = 500.0


@pytest.fixture(autouse=True)
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(spec_mod, "REPO", tmp_path)
    monkeypatch.setattr(pallet, "report_path",
                        lambda n, dev=False: tmp_path / f"{n}{'.dev' if dev else ''}.json")


@pytest.fixture
def work(tmp_path, pass_raw):
    spec = tmp_path / "line_spec.yaml"

    def write(raw=None):
        spec.write_text(yaml.safe_dump(raw if raw is not None else pass_raw))
        return spec
    write()
    return spec, tmp_path / "out", write


def run(work, *extra):
    spec, out, _ = work
    return pallet.main(["--spec", str(spec), "--out", str(out), *extra])


def report(tmp_path, dev=False):
    return json.loads((tmp_path / f"cad_pallet{'.dev' if dev else ''}.json").read_text())


def layout_for(raw):
    lo = pallet.layout(pallet.Values(raw))
    assert lo is not None
    return lo


def probe(part, x, y, size=1.0):
    """The material of `part` inside a thin vertical column at (x, y)."""
    return part & Pos(x, y) * Box(size, size, BIG)


# ---------------------------------------------------------------- datums

@pytest.mark.parametrize("round_xy, diamond_xy", [
    ((0.0, 0.0), (0.1, 0.0)),                 # the spec's own datum pair
    ((0.0012, -0.0034), (0.0712, 0.0466)),    # off-origin and off-axis: a 45° datum line
    ((0.05, 0.05), (-0.05, 0.05)),            # datum line pointing -x
])
def test_pin_axes_equal_the_spec_datums(pass_raw, round_xy, diamond_xy):
    pass_raw["parts"]["bottom_plate"]["datums"] = {"round": list(round_xy),
                                                   "diamond": list(diamond_xy)}
    lo = layout_for(pass_raw)
    got = pallet.derived(pallet.build(lo), lo)
    for name, want in (("pin_round_mm", round_xy), ("pin_diamond_mm", diamond_xy)):
        for axis, (g, w) in enumerate(zip(got[name], want)):
            assert abs(g - w * 1000.0) <= TOL_MM, f"{name}[{axis}]"


def test_report_pin_positions_come_from_the_built_solids(work, tmp_path, pass_raw):
    spec, _, write = work
    pass_raw["parts"]["bottom_plate"]["datums"] = {"round": [0.001, 0.002],
                                                   "diamond": [0.101, 0.002]}
    write(pass_raw)
    assert run(work) == 0
    d = report(tmp_path)["derived"]
    assert d["pin_round_mm"] == pytest.approx([1.0, 2.0], abs=TOL_MM)
    assert d["pin_diamond_mm"] == pytest.approx([101.0, 2.0], abs=TOL_MM)
    assert d["datum_pitch_mm"] == pytest.approx(100.0, abs=TOL_MM)


# ---------------------------------------------------------------- pin geometry

@pytest.mark.parametrize("diamond_xy, angle", [((0.1, 0.0), 0.0), ((0.0, 0.1), 90.0),
                                               ((0.07071, 0.07071), 45.0)])
def test_diamond_pin_is_relieved_along_the_datum_line(pass_raw, diamond_xy, angle):
    """The flats face the round pin, so the pin is narrow along the datum line and full
    diameter across it — that is what stops hole-spacing tolerance from jamming the stack."""
    pass_raw["parts"]["bottom_plate"]["datums"] = {"round": [0.0, 0.0],
                                                   "diamond": list(diamond_xy)}
    lo = layout_for(pass_raw)
    assert lo.relief_deg == pytest.approx(angle, abs=0.01)

    pin = Rot(Z=-lo.relief_deg) * pins.diamond_pin(lo.pin, lo.relief_deg)  # datum line -> +x
    above_rest_plane = pin & Pos(0, 0, 1.0) * Box(BIG, BIG, 2.0,
                                                  align=(Align.CENTER, Align.CENTER, Align.MIN))
    size = above_rest_plane.bounding_box().size
    assert size.X == pytest.approx(lo.pin.crown_w, abs=TOL_MM)      # relieved along the line
    assert size.Y == pytest.approx(lo.pin.locating_d, abs=TOL_MM)   # bearing across it


def test_round_pin_is_full_diameter(pass_raw):
    lo = layout_for(pass_raw)
    pin = pins.round_pin(lo.pin)
    above = pin & Pos(0, 0, 1.0) * Box(BIG, BIG, 2.0, align=(Align.CENTER, Align.CENTER,
                                                             Align.MIN))
    size = above.bounding_box().size
    assert size.X == pytest.approx(lo.pin.locating_d, abs=TOL_MM)
    assert size.Y == pytest.approx(lo.pin.locating_d, abs=TOL_MM)
    assert lo.pin.locating_d < pass_raw["cad"]["datum_d"] * 1000.0   # clears the reamed hole


def test_pins_clear_the_whole_stack_plus_a_lead_in(pass_raw):
    lo = layout_for(pass_raw)
    top = pass_raw["parts"]["top_plate"]
    stack_mm = (top["z_in_stack"] + top["envelope"][2]) * 1000.0
    assert lo.stack_h == pytest.approx(stack_mm, abs=TOL_MM)
    for pin in (pins.round_pin(lo.pin), pins.diamond_pin(lo.pin, lo.relief_deg)):
        assert pin.bounding_box().max.Z >= stack_mm + pallet.PIN_LEAD_IN - TOL_MM
        assert pin.bounding_box().min.Z == pytest.approx(-pallet.THICKNESS, abs=TOL_MM)


# ---------------------------------------------------------------- pallet body

def test_nest_is_relieved_under_the_footprint_but_the_plate_lands_on_the_rest_pads(pass_raw):
    """Clearance under every screw site comes from relieving the whole footprint, so the
    generator needs no hole pattern; the plate is carried by the four corner pads."""
    lo = layout_for(pass_raw)
    body = pallet.solid(pallet.build(lo), "Pallet_Body")
    cx, cy = lo.centre
    assert probe(body, cx, cy).bounding_box().max.Z == pytest.approx(-pallet.RELIEF_DEPTH,
                                                                     abs=TOL_MM)
    for px, py in pallet.derived(pallet.build(lo), lo)["nest"]["rest_pad_centres_mm"]:
        assert probe(body, px, py).bounding_box().max.Z == pytest.approx(0.0, abs=TOL_MM)
    outside = probe(body, cx, cy + lo.plate[1] / 2.0 + pallet.WALL / 2.0)
    assert outside.bounding_box().max.Z == pytest.approx(0.0, abs=TOL_MM)   # wall, not relieved


def test_stop_notch_is_cut_into_the_leading_face(pass_raw):
    lo = layout_for(pass_raw)
    body = pallet.solid(pallet.build(lo), "Pallet_Body")
    face_x = lo.centre[0] + lo.size[0] / 2.0
    assert probe(body, face_x - 1.0, lo.centre[1]).volume == pytest.approx(0.0)
    assert probe(body, face_x - 1.0, lo.centre[1] + 3.0 * pallet.STOP_NOTCH_DEPTH).volume > 0.0


# ---------------------------------------------------------------- CLI

def test_writes_step_stl_and_report(work, tmp_path):
    _, out, _ = work
    assert run(work) == 0
    step, stl = out / "pallet.step", out / "pallet.stl"
    assert step.is_file() and stl.is_file()
    doc = report(tmp_path)
    assert doc["passed"] is True and doc["missing"] == [] and doc["errors"] == []
    assert doc["outputs"] == {"step": str(step), "stl": str(stl)}
    assert doc["bbox_mm"]["size"][:2] == [220.0, 220.0]          # 200 mm plate + 2 x wall
    assert doc["mass_kg"]["total"] > doc["mass_kg"]["Pallet_Body"] > 0.0
    assert stl.stat().st_size < 5 * 1024 * 1024                  # P0: no STL over 5 MB
    assert doc["mesh"]["size_bytes"] == stl.stat().st_size
    assert doc["mesh"]["solid_bbox_drift_mm"] <= 0.2


def test_null_spec_value_fails_and_names_every_key(work, tmp_path, pass_raw, capsys):
    _, out, write = work
    pass_raw["conveyor"]["pitch"] = None
    pass_raw["parts"]["top_plate"]["z_in_stack"] = None
    write(pass_raw)
    assert run(work) == 1
    assert report(tmp_path)["missing"] == ["parts.top_plate.z_in_stack", "conveyor.pitch"]
    err = capsys.readouterr().err
    assert "parts.top_plate.z_in_stack" in err and "conveyor.pitch" in err
    assert not (out / "pallet.step").exists()


def test_pallet_longer_than_the_conveyor_pitch_fails(work, tmp_path, pass_raw):
    _, _, write = work
    pass_raw["conveyor"]["pitch"] = 0.2                 # 200 mm pitch, 220 mm pallet
    write(pass_raw)
    assert run(work) == 1
    assert "conveyor.pitch" in report(tmp_path)["errors"][0]


def test_datums_at_the_same_point_fail(work, tmp_path, pass_raw):
    _, _, write = work
    pass_raw["parts"]["bottom_plate"]["datums"]["diamond"] = [0.0, 0.0]
    write(pass_raw)
    assert run(work) == 1
    assert "parts.bottom_plate.datums" in report(tmp_path)["errors"][0]


def test_dev_run_is_quarantined_in_its_own_folder(work, tmp_path, pass_raw):
    spec, out, write = work
    pass_raw["conveyor"]["pitch"] = None
    write(pass_raw)
    (tmp_path / "line_spec.dev.yaml").write_text(yaml.safe_dump({"conveyor": {"pitch": 0.5}}))
    assert run(work, "--dev") == 0
    assert (out / "dev" / "pallet.step").is_file()
    assert not (out / "pallet.step").exists()
    doc = report(tmp_path, dev=True)
    assert doc["dev"] is True and doc["placeholders"] == ["conveyor.pitch"]
    assert not (tmp_path / "cad_pallet.json").exists()
