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


def _drive_at_place(s):
    """gh_top screws down the mid plate scara_place just placed, at st_place: a SCARA plus the gang
    head that fastens what it placed. Allowed (D-011, D-012). st_drive is left empty, so it goes
    (from its milestone too)."""
    _seq(s, "drive")["station"] = "st_place"
    del s["stations"]["st_drive"]
    s["milestones"]["front"].remove("st_drive")
    s["stations"]["st_test"]["anchor"] = {"after": "st_place", "gap": 1.0}


def _slide_at_place(t_in, t_out, stroke=0.30):
    """gh_top at st_place, on a slide with its frame (it shares the station with scara_place,
    D-012, D-014). 0.30 m out the mid plate's band (0.141 m) clears the waiting head by 58.6 mm."""
    def mutate(s):
        _drive_at_place(s)
        _seq(s, "drive")["duration_s"] = 5.0          # place 6 + drive 5: inside the 20 s takt
        s["gang_heads"]["gh_top"]["slide"] = {"stroke": stroke, "t_in": t_in, "t_out": t_out,
                                              "frame": {"post": [0.1, 0.1], "rail": [0.08, 0.06]}}
    return mutate


def _parked_over_the_head(s):
    """scara_place parks stretched out over the pallet (j1 = j2 = 0) with its stroke lowered to
    0.25 m, so its tools (z 0.23..0.25) sit in gh_top's working box (z 0.118..0.268)."""
    _slide_at_place(0.5, 0.5)(s)
    s["robot_models"]["test_scara"]["z_stroke"] = 0.25
    s["scaras"]["scara_place"]["park_pose"] = {"j1": 0.0, "j2": 0.0}


def _park_pose_unknown(s):
    _slide_at_place(0.5, 0.5)(s)
    s["scaras"]["scara_place"]["park_pose"] = {"j1": None, "j2": None}


def _rail_in_the_place_path(s):
    """gh_top's rail made 0.5 m tall: it now stands beside the pallet from z -0.02 to 0.48, and
    the plate tool (radius 0.085) placing at the datum comes within 15 mm of it in plan."""
    _slide_at_place(0.5, 0.5)(s)
    s["gang_heads"]["gh_top"]["slide"]["frame"]["rail"] = [0.08, 0.5]


def _refilled_buffer_under_the_park(s):
    """The mid plate comes from buf_mid, a buffer src_mid refills, standing right under
    scara_place's park pose (TCP 0.452, 0.328): the parked arm is over its refill envelope."""
    _slide_at_place(0.5, 0.5)(s)
    del s["feeders"]["fd_mid_plate"]
    del s["layout"]["feeder_footprints"]["fd_mid_plate"]
    s["buffers"]["buf_mid"] = {"station": "st_place", "offset": [-0.05, 0.33, 0.1, 0.0]}
    s["layout"]["feeder_footprints"]["buf_mid"] = [0.06, 0.06, 1.0]
    s["sources"]["src_mid"] = {"output": "mid_plate", "buffer": "buf_mid", "interval_s": 20.0,
                               "buffer_capacity": 3}
    _seq(s, "place")["from"] = "buf_mid"


def _second_head(station, requires):
    """A second gang head, gh_two, driving at `station`."""
    def mutate(s):
        s["gang_heads"]["gh_two"] = dict(s["gang_heads"]["gh_top"], pattern_from=None, spindles=2)
        s["sequence"].append({"id": "drive_two", "station": station, "capability": "drive",
                              "head": "gh_two", "fastener": "m3_x8", "qty": 2, "duration_s": 1.0,
                              "requires": requires})
    return mutate


def _drive_at_place_after(requires):
    def mutate(s):
        _drive_at_place(s)
        _seq(s, "drive")["requires"] = requires
    return mutate


def _gh_top_at_two_stations(s):
    s["sequence"].append({"id": "drive_again", "station": "st_place", "capability": "drive",
                          "head": "gh_top", "fastener": "m3_x8", "qty": 8, "duration_s": 1.0,
                          "requires": ["place"]})


def _sub_line_loader(source):
    """st_load becomes a sub-line loader: the plate arrives on the pallet from `source`."""
    def mutate(s):
        s["stations"]["st_load"].update(actor="sub_line", source=source)
        _seq(s, "load").pop("from")
    return mutate


def _second_drive_without_fastener(s):
    s["sequence"].append({"id": "drive_more", "station": "st_drive", "capability": "drive",
                          "head": "gh_top", "qty": 2, "duration_s": 1.0, "requires": ["drive"]})


def _rename_scara(old, new):
    def mutate(s):
        s["scaras"][new] = s["scaras"].pop(old)
    return mutate


def _drop_tooling_hole(s):
    s["drone"]["arm_tooling_holes"].pop()


def _actuate_without_device(s):
    s["sequence"].append({"id": "pins", "station": "st_place", "capability": "actuate",
                          "duration_s": 0.5, "requires": ["load"]})


def _absolute_station(s):
    s["stations"]["st_drive"].pop("anchor")
    s["stations"]["st_drive"]["pose"] = [1.0, 0.0, 0.0, 0.0]
    s["stations"]["st_test"]["anchor"]["after"] = "st_drive"


def _absolute_feeder(s):
    f = s["feeders"]["fd_mid_plate"]
    f.pop("offset")
    f.pop("station")
    f["pose"] = [0.3, 0.3, 0.15, 0.0]


def _longer_links(link):
    """Every SCARA reaches 2 x link: 0.30 m overlaps neighbours 0.5 m apart by 0.1 m."""
    def mutate(s):
        m = s["robot_models"]["test_scara"]
        m.update(link1=link, link2=link, reach=2 * link)
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
    # Actor audit (D-010).
    ("actors", "station_without_actor", _set(["sequence", 3, "capability"], "inspect"), "FAIL"),
    ("actors", "undecided_actor_is_unresolved",
     lambda s: (_set(["sequence", 3, "capability"], "inspect")(s),
                _set(["stations", "st_test", "actor"], None)(s)), "WARN"),
    ("actors", "two_primary_actors", _set(["sequence", 3, "station"], "st_place"), "FAIL"),
    ("actors", "gang_head_not_fastening_what_the_scara_placed",
     _drive_at_place_after(["load"]), "FAIL"),
    ("actors", "gang_head_named_at_two_stations", _gh_top_at_two_stations, "FAIL"),
    ("actors", "two_gang_heads_at_a_scara_station",
     lambda s: (_drive_at_place(s), _second_head("st_place", ["place"])(s)), "FAIL"),
    ("actors", "two_drivers_at_a_driver_station", _second_head("st_drive", ["drive"]), "FAIL"),
    ("actors", "drive_step_without_driver", lambda s: _seq(s, "drive").pop("head"), "FAIL"),
    ("actors", "drive_step_without_fastener", _second_drive_without_fastener, "WARN"),
    ("actors", "feeder_never_drawn_from",
     _set(["feeders", "fd_spare"], {"type": "magazine", "part": "mid_plate",
                                    "pose": [0.3, 0.3, 0.15, 0.0], "capacity": 20,
                                    "refill_s": 60.0}), "FAIL"),
    ("actors", "buffer_never_drawn_from", _set(["buffers", "buf_spare"],
                                               {"pose": [0.3, -0.3, 0.1, 0.0]}), "FAIL"),
    ("actors", "feeder_holds_another_part", _set(["sequence", 1, "part"], "top_plate"), "FAIL"),
    ("actors", "buffer_filled_with_another_part", _set(["sequence", 0, "part"], "mid_plate"),
     "FAIL"),
    ("actors", "part_from_nowhere", lambda s: _seq(s, "place").pop("from"), "FAIL"),
    ("actors", "sub_line_source_does_not_exist", _sub_line_loader("buf_nope"), "FAIL"),
    ("actors", "sub_line_without_source", _sub_line_loader(None), "FAIL"),
    # remove (D-013): only what is on the pallet, onto its own return path.
    ("actors", "removes_a_part_not_on_the_pallet", _set(["sequence", 5, "part"], "carrier_pod"),
     "FAIL"),
    ("actors", "return_path_nothing_uses",
     _set(["return_paths", "rt_spare"], {"carrier": "carrier_pod", "station": "st_pod",
                                         "kind": "chute", "offset": [0.2, -0.3, 0.1, 0.0],
                                         "length": 0.2, "width": 0.1, "sink": "buf_pod"}),
     "FAIL"),
    ("schema", "remove_without_a_return_path", lambda s: _seq(s, "remove_spider").pop("to"),
     "FAIL"),
    # station_time (D-011): gh_top 1 pass x 19 s + index 0.5/0.3 s > takt 20 s.
    ("station_time", "drive_pass_too_long", _set(["gang_heads", "gh_top", "drive_s"], 19.0),
     "FAIL"),
    ("station_time", "null_drive_s", _set(["gang_heads", "gh_top", "drive_s"], None), "WARN"),
    ("unresolved", "null_mass", _set(["parts", "mid_plate", "mass"], None), "WARN"),
    ("precedence", "cycle", _cycle, "FAIL"),
    ("precedence", "requires_later_step", _requires_later, "FAIL"),
    ("takt", "test_bank_too_slow", _set(["sequence", 3, "duration_s"], 150.0), "FAIL"),
    ("takt", "unknown_duration", _null_durations, "WARN"),
    ("line_balance", "partial_durations", _null_durations, "WARN"),
    # One SCARA per pick-and-place station (D-012). Structural rules fail the parse: `schema`.
    ("schema", "scara_id_is_not_its_station", _rename_scara("scara_place", "scara_put"), "FAIL"),
    ("schema", "scara_station_does_not_exist",
     _set(["scaras", "scara_place", "station"], "st_nowhere"), "FAIL"),
    ("schema", "robot_step_at_a_station_without_scara",
     lambda s: s["scaras"].pop("scara_place"), "FAIL"),
    ("schema", "scara_tool_does_not_exist", _set(["scaras", "scara_pod", "tools"], ["tool_nope"]),
     "FAIL"),
    ("schema", "scara_model_does_not_exist", _set(["scaras", "scara_pod", "model"], "nope"),
     "FAIL"),
    ("schema", "tool_not_on_the_station_scara", _set(["sequence", 1, "tool"], "tool_carrier"),
     "FAIL"),
    ("schema", "place_and_drive_in_one_step", _set(["sequence", 1, "fastener"], "m3_x8"), "FAIL"),
    ("schema", "actuate_step_without_device", _actuate_without_device, "FAIL"),
    # Milestones (D-012).
    ("schema", "station_in_no_milestone",
     _set(["milestones", "back"], ["st_test", "st_spider"]), "FAIL"),
    ("schema", "station_in_two_milestones",
     _set(["milestones", "back"], ["st_drive", "st_test", "st_spider", "st_pod"]), "FAIL"),
    ("schema", "milestones_out_of_line_order",
     _set(["milestones", "front"], ["st_place", "st_load", "st_drive"]), "FAIL"),
    # anchors (D-013): stations on an anchor chain, everything they own at an offset.
    ("anchors", "station_with_an_absolute_pose", _absolute_station, "FAIL"),
    ("anchors", "anchored_to_the_wrong_station",
     _set(["stations", "st_drive", "anchor"], {"after": "st_load", "gap": 1.0}), "FAIL"),
    ("anchors", "feeder_with_an_absolute_pose", _absolute_feeder, "FAIL"),
    # scara_reach (D-013): IK inside the model's links and j1/j2 limits, z inside the stroke.
    ("scara_reach", "pick_beyond_reach",
     _set(["feeders", "fd_mid_plate", "offset"], [-0.2, 0.6, 0.15, 0.0]), "FAIL"),
    ("scara_reach", "pick_inside_r_min",
     _set(["robot_models", "test_scara", "joint_limits", "j2"], 1.0), "FAIL"),
    ("scara_reach", "pick_outside_j1",
     _set(["robot_models", "test_scara", "joint_limits", "j1"], 0.3), "FAIL"),
    ("scara_reach", "pick_above_the_stroke",
     _set(["feeders", "fd_mid_plate", "offset"], [-0.2, 0.3, 0.7, 0.0]), "FAIL"),
    ("scara_reach", "null_links", _set(["robot_models", "test_scara", "link1"], None), "WARN"),
    # scara_clearance: a base footprint stands clear; shared reach is a WARN.
    ("scara_clearance", "base_in_the_conveyor",
     _set(["scaras", "scara_place", "offset", 1], 0.15), "FAIL"),
    ("scara_clearance", "base_on_a_buffer",
     _set(["buffers", "buf_plate", "offset"], [0.45, 0.3, 0.1, 0.0]), "FAIL"),
    ("scara_clearance", "two_bases_overlap", _set(["layout", "scara_base", "diameter"], 0.55),
     "FAIL"),
    ("scara_clearance", "envelopes_overlap", _longer_links(0.15), "WARN"),
    ("scara_clearance", "null_base_diameter", _set(["layout", "scara_base", "diameter"], None),
     "WARN"),
    # safe_z (path based, D-013): scara_spider's is 0.235 (buf_fixture 0.1 under its path +
    # carrier_spider 0.105 + 0.03), scara_pod's 0.240 (lifting that carrier off): 0.22 fails both.
    ("safe_z", "safe_z_above_the_z_stroke",
     _set(["robot_models", "test_scara", "z_stroke"], 0.22), "FAIL"),
    ("safe_z", "null_margin", _set(["scara_defaults", "z_margin"], None), "WARN"),
    # payload / wrist_inertia (D-013): scara_spider lifts 0.3 kg of tool + 0.6 kg of carrier.
    ("payload", "over_payload_rated",
     _set(["robot_models", "test_scara", "payload_rated"], 0.5), "FAIL"),
    ("payload", "null_tool_mass", _set(["tools", "tool_carrier", "mass"], None), "WARN"),
    ("wrist_inertia", "over_inertia_rated",
     _set(["robot_models", "test_scara", "inertia_rated"], 0.001), "WARN"),
    ("wrist_inertia", "null_part_mass", _set(["parts", "carrier_pod", "mass"], None), "WARN"),
    # D-015: above inertia_max the robot cannot carry it.
    ("wrist_inertia", "over_inertia_max",
     _set(["robot_models", "test_scara", "inertia_max"], 0.001), "FAIL"),
    ("robot_load", "scara_over_budget",
     _set(["robot_models", "test_scara", "motion", "v_max"], 0.02), "FAIL"),
    ("robot_load", "null_grip_time", _set(["scara_defaults", "grip_s"], None), "WARN"),
    # D-009: the pallet is moving for pitch/belt_speed + stop_unit_cycle_s of every takt.
    ("robot_load", "transfer_eats_the_budget", _set(["layout", "stop_unit_cycle_s"], 16.0),
     "FAIL"),
    # park_clearance (D-014): the arm parked while the head works stays out of its envelope.
    ("park_clearance", "parked_arm_in_the_working_head", _parked_over_the_head, "FAIL"),
    ("park_clearance", "null_park_pose", _park_pose_unknown, "WARN"),
    # slide_clearance (D-014): 0.05 m out the head waits under the place.
    ("slide_clearance", "part_lands_on_the_retracted_head", _slide_at_place(0.5, 0.5, 0.05),
     "FAIL"),
    ("slide_clearance", "null_slide_stroke", _slide_at_place(0.5, 0.5, None), "WARN"),
    # path_clearance (D-015): every arm path 20 mm clear of the waiting head and its frame.
    ("path_clearance", "arm_path_past_a_tall_rail", _rail_in_the_place_path, "FAIL"),
    # refill_clearance (D-015): the parked arm keeps out of a refilled buffer's drop-in column.
    ("refill_clearance", "parked_over_a_refilled_buffer", _refilled_buffer_under_the_park,
     "FAIL"),
    # station_time with a slide (D-012): place + 1 pass x 3 s + 9 + 9 s of slide + index > 20 s.
    ("station_time", "slide_too_slow", _slide_at_place(9.0, 9.0), "FAIL"),
    ("station_time", "null_slide_time", _slide_at_place(None, 0.5), "WARN"),
    # arm_pin_clearance (D-012): a tooling hole 2 x M3 clear of every arm screw, one per arm.
    ("arm_pin_clearance", "tooling_hole_next_to_a_screw",
     _set(["drone", "arm_tooling_holes", 0], [0.090, 0.040]), "FAIL"),
    ("arm_pin_clearance", "not_one_pin_per_arm", _drop_tooling_hole, "FAIL"),
    ("arm_pin_clearance", "screw_diameter_unknown", _set(["fasteners", "m3_x8", "d"], None),
     "WARN"),
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
    import robot_geometry
    monkeypatch.setattr(robot_geometry, "CAD_REPORTS", FIXTURES / "cad")


@pytest.fixture
def pass_raw():
    return yaml.safe_load((FIXTURES / "pass_spec.yaml").read_text())


@pytest.fixture(params=FAILING, ids=[f"{c}-{i}" for c, i, _, _ in FAILING])
def failing_case(request, pass_raw):
    check, _, mutate, expected = request.param
    raw = copy.deepcopy(pass_raw)
    mutate(raw)
    return check, raw, expected
