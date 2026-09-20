import json
import subprocess
import sys

import pytest
import yaml

import gen_sw_params as gsp
import spec as spec_mod
from conftest import TOOLS

ALL = ["datum_d", "datum_dx", "datum_dy", "press_nut_hole_d", "lead_slot_w", "lead_slot_l",
       "z_mid", "z_top"]


def vars_for(raw):
    return gsp.collect(spec_mod.parse_spec(raw))


def test_round_trip_all_variables(pass_raw):
    text, written, skipped = gsp.render(vars_for(pass_raw))
    assert skipped == []
    parsed = gsp.parse_params(text)
    assert list(parsed) == ALL
    assert parsed == {"datum_d": 3.0, "datum_dx": 100.0, "datum_dy": 0.0,
                      "press_nut_hole_d": 4.0, "lead_slot_w": 6.0, "lead_slot_l": 12.0,
                      "z_mid": 30.0, "z_top": 60.0}


def test_format_is_one_quoted_variable_per_line_mm_4_decimals(pass_raw):
    text, _, _ = gsp.render(vars_for(pass_raw))
    assert text.splitlines()[1] == '"datum_dx" = 100.0000'
    assert text.endswith("\n")


def test_datum_offset_is_diamond_minus_round(pass_raw):
    pass_raw["parts"]["bottom_plate"]["datums"] = {"round": [0.001, 0.002],
                                                   "diamond": [0.0635, 0.0]}
    parsed = gsp.parse_params(gsp.render(vars_for(pass_raw))[0])
    assert parsed["datum_dx"] == pytest.approx(62.5)
    assert parsed["datum_dy"] == pytest.approx(-2.0)


def test_no_negative_zero(pass_raw):
    pass_raw["parts"]["bottom_plate"]["datums"]["diamond"] = [0.1, -1e-9]
    assert '"datum_dy" = 0.0000' in gsp.render(vars_for(pass_raw))[0]


def test_nulls_are_skipped_and_named(pass_raw):
    pass_raw["cad"]["press_nut_hole_d"] = None
    pass_raw["parts"]["mid_plate"]["lead_slot"] = [0.006, None]
    text, written, skipped = gsp.render(vars_for(pass_raw))
    assert {v.name: v.missing for v in skipped} == {
        "press_nut_hole_d": ["cad.press_nut_hole_d"],
        "lead_slot_l": ["parts.mid_plate.lead_slot[1]"]}
    assert set(gsp.parse_params(text)) == set(ALL) - {"press_nut_hole_d", "lead_slot_l"}


def test_parse_rejects_malformed_and_duplicate_lines():
    with pytest.raises(ValueError, match="not a global-variable"):
        gsp.parse_params("datum_d = 3\n")
    with pytest.raises(ValueError, match="duplicate"):
        gsp.parse_params('"a" = 1.0000\n"a" = 2.0000\n')


def run(tmp_path, raw, *extra):
    spec_path = tmp_path / "spec.yaml"
    spec_path.write_text(yaml.safe_dump(raw))
    out = tmp_path / "sub" / "cad_params.txt"
    proc = subprocess.run(
        [sys.executable, str(TOOLS / "gen_sw_params.py"), "--spec", str(spec_path),
         "--out", str(out), *extra], capture_output=True, text=True)
    return proc, out


def test_cli_writes_file_that_round_trips(tmp_path, pass_raw):
    proc, out = run(tmp_path, pass_raw)
    assert proc.returncode == 0, proc.stderr
    assert gsp.parse_params(out.read_text())["datum_dx"] == 100.0


def test_cli_warns_naming_the_key_and_still_succeeds(tmp_path, pass_raw):
    pass_raw["parts"]["top_plate"]["z_in_stack"] = None
    proc, out = run(tmp_path, pass_raw)
    assert proc.returncode == 0
    assert "WARN skipped z_top" in proc.stderr and "parts.top_plate.z_in_stack" in proc.stderr
    assert "z_top" not in gsp.parse_params(out.read_text())


def test_cli_invalid_spec_exits_nonzero_and_writes_no_file(tmp_path, pass_raw):
    pass_raw["line"]["takt_s"] = -1
    proc, out = run(tmp_path, pass_raw)
    assert proc.returncode == 1
    assert not out.exists()


def test_dev_run_never_writes_the_cad_folder():
    proc = subprocess.run([sys.executable, str(TOOLS / "gen_sw_params.py"), "--dev", "--out",
                           "/tmp/should_not_exist.txt"], capture_output=True, text=True)
    assert proc.returncode == 2
    assert "refused" in proc.stderr


def test_dev_run_reports_placeholders_and_separate_files():
    proc = subprocess.run([sys.executable, str(TOOLS / "gen_sw_params.py"), "--dev"],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    doc = json.loads(spec_mod.report_path("sw_params", dev=True).read_text())
    assert doc["dev"] is True and doc["out"].endswith("cad_params.dev.txt")
    assert "cad.press_nut_hole_d" in doc["placeholders"]
    assert set(gsp.parse_params(gsp.DEV_OUT.read_text())) == set(ALL)
