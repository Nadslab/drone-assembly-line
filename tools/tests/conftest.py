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
    ("reach", "null_axis_limit", _set(["gantry", "axes", "x", "max"], None), "WARN"),
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


@pytest.fixture
def pass_raw():
    return yaml.safe_load((FIXTURES / "pass_spec.yaml").read_text())


@pytest.fixture(params=FAILING, ids=[f"{c}-{i}" for c, i, _, _ in FAILING])
def failing_case(request, pass_raw):
    check, _, mutate, expected = request.param
    raw = copy.deepcopy(pass_raw)
    mutate(raw)
    return check, raw, expected
