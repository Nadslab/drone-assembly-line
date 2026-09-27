"""Fixtures: one passing spec and one failing (or warning) spec per check.

Failing specs are the passing spec with a single deliberate defect, so each test isolates the
check it names.
"""

import copy
import sys
from pathlib import Path

import pytest
import yaml

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))

FIXTURES = Path(__file__).resolve().parent / "fixtures"


def _seq(spec, step_id):
    return next(s for s in spec["sequence"] if s["id"] == step_id)


def _cycle(s):
    _seq(s, "load")["requires"] = ["test"]


def _requires_later(s):
    _seq(s, "load")["requires"] = ["place"]


def _four_lengths(s):
    for i, mm in enumerate((0.010, 0.012, 0.016)):
        s["fasteners"][f"m3_l{i}"] = {"length": mm, "feeder": "fd_screw"}


def _null_durations(s):
    _seq(s, "place")["duration_s"] = None


def _rename_gantry(old, new):
    def mutate(s):
        s["gantries"][new] = s["gantries"].pop(old)
    return mutate


def _set(path, value):
    """Mutation that sets spec[k1][k2]... = value."""
    def mutate(s):
        node = s
        for k in path[:-1]:
            node = node[k]
        node[path[-1]] = value
    return mutate


# (check, case id, mutation, expected status of that check)
FAILING = [
    ("schema", "dangling_station", _set(["sequence", 1, "station"], "st_nowhere"), "FAIL"),
    ("schema", "dangling_feeder", _set(["fasteners", "m3_x8", "feeder"], "fd_nope"), "FAIL"),
    ("unresolved", "null_mass", _set(["parts", "mid_plate", "mass"], None), "WARN"),
    ("precedence", "cycle", _cycle, "FAIL"),
    ("precedence", "requires_later_step", _requires_later, "FAIL"),
    ("takt", "test_bank_too_slow", _set(["sequence", 3, "duration_s"], 150.0), "FAIL"),
    ("takt", "unknown_duration", _null_durations, "WARN"),
    ("line_balance", "partial_durations", _null_durations, "WARN"),
    ("reach", "feeder_outside_x", _set(["feeders", "fd_mid_plate", "pose"], [9.0, 0.3, 0.1, 0.0]),
     "FAIL"),
    ("reach", "null_axis_limit", _set(["gantries", "gantry_1", "axes", "x", "max"], None), "WARN"),
    # Outside gantry_1's travel, and st_place (which draws from it) is gantry_1's.
    ("reach", "feeder_outside_its_zone_gantry",
     _set(["feeders", "fd_mid_plate", "pose"], [1.0, 0.3, 0.15, 0.0]), "FAIL"),
    # Zone rules (pydantic): a failure there fails the parse, so it surfaces as `schema`.
    ("schema", "zone_names_unknown_station",
     _set(["gantries", "gantry_1", "zone"], ["st_load", "st_place", "st_nowhere"]), "FAIL"),
    ("schema", "station_in_two_zones",
     _set(["gantries", "gantry_2", "zone"], ["st_spider", "st_pod", "st_place"]), "FAIL"),
    ("schema", "gantry_step_in_no_zone", _set(["gantries", "gantry_1", "zone"], ["st_load"]),
     "FAIL"),
    ("schema", "gantry_id_not_numbered", _rename_gantry("gantry_2", "gantry_b"), "FAIL"),
    ("schema", "one_tool_id_two_designs",
     _set(["gantries", "gantry_2", "tools", "tool_plate"],
          {"grip": "vacuum", "dims": [0.10, 0.10, 0.02]}), "FAIL"),
    ("schema", "tool_not_on_the_zone_gantry", _set(["sequence", 1, "tool"], "tool_carrier"),
     "FAIL"),
    ("gantry_load", "gantry_2_over_takt",
     _set(["gantries", "gantry_2", "axes", "x", "v_max"], 0.1), "FAIL"),
    ("gantry_load", "null_grip_time", _set(["gantry_defaults", "grip_s"], None), "WARN"),
    # D-009: the pallet is moving for pitch/belt_speed + stop_unit_cycle_s of every takt.
    ("gantry_load", "transfer_eats_the_budget", _set(["layout", "stop_unit_cycle_s"], 15.0),
     "FAIL"),
    ("schema", "place_and_drive_in_one_step", _set(["sequence", 1, "fastener"], "m3_x8"), "FAIL"),
    # D-009 frame rule: the conveyor is +-0.15; gantry_1's -y posts stand at y.min - 0.10.
    ("gantry_straddle", "post_in_the_conveyor",
     _set(["gantries", "gantry_1", "axes", "y", "min"], -0.1), "FAIL"),
    ("gantry_straddle", "post_inside_the_clearance",
     _set(["gantries", "gantry_1", "axes", "y", "min"], -0.2), "FAIL"),
    ("gantry_straddle", "null_clearance",
     _set(["layout", "gantry_frame", "conveyor_clearance"], None), "WARN"),
    # gantry_2's safe_z is 0.372 (pod tray 0.2365 + spider carrier hanging 0.1054 + 0.03).
    ("safe_z", "safe_z_above_the_z_travel",
     _set(["gantries", "gantry_2", "axes", "z", "max"], 0.3), "FAIL"),
    ("safe_z", "null_margin", _set(["gantry_defaults", "z_margin"], None), "WARN"),
    ("feeder_rate", "nest_set_outruns_its_refill",
     _set(["feeders", "fd_mid_plate", "per_pick"], 10), "FAIL"),
    ("feeder_rate", "screws_too_slow", _set(["feeders", "fd_screw", "feed_s"], 5.0), "FAIL"),
    ("feeder_rate", "unknown_refill", _set(["feeders", "fd_mid_plate", "refill_s"], None), "WARN"),
    ("buffer", "source_slower_than_takt", _set(["sources", "src_plate", "interval_s"], 25.0),
     "FAIL"),
    ("buffer", "capacity_zero", _set(["sources", "src_plate", "buffer_capacity"], 0), "FAIL"),
    ("fastener_lengths", "four_lengths", _four_lengths, "WARN"),
    ("no_flip", "fasten_from_below", _set(["gang_heads", "gh_top", "direction"], "up"), "FAIL"),
    ("datums", "mid_plate_round_moved", _set(["parts", "mid_plate", "datums", "round"], [0.01, 0.0]),
     "FAIL"),
    ("datums", "missing_datums", lambda s: s["parts"]["top_plate"].pop("datums"), "WARN"),
]


@pytest.fixture(autouse=True)
def cad_fixture_reports(monkeypatch):
    """safe_z reads build123d part heights from cad_<part>.json: use the fixture copies, so a
    test never depends on what tools/cad last built in reports/. A test that wants the real ones
    passes the directory explicitly."""
    import gantry_geometry
    monkeypatch.setattr(gantry_geometry, "CAD_REPORTS", FIXTURES / "cad")


@pytest.fixture
def pass_raw():
    return yaml.safe_load((FIXTURES / "pass_spec.yaml").read_text())


@pytest.fixture(params=FAILING, ids=[f"{c}-{i}" for c, i, _, _ in FAILING])
def failing_case(request, pass_raw):
    check, _, mutate, expected = request.param
    raw = copy.deepcopy(pass_raw)
    mutate(raw)
    return check, raw, expected
