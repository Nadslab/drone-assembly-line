"""The ros2_control controller config, from line_spec.yaml (plan P5.3).

    tools/.venv/bin/python tools/gen_controllers.py [--dev] [--spec PATH]

Writes `drone_line_sim/config/controllers.yaml`: one `joint_trajectory_controller` per SCARA
(`scara_<station>_controller` over j1..j4, D-012), one per gang head, and a
`joint_state_broadcaster`. The joints are named exactly as `urdf/line.urdf.xacro` names them —
`scara_<station>_j1..j4` and `<head_id>_stroke` (`_1`/`_2` for the two banks of an opposed head) —
and `tools/checks/descriptions.sh` fails if the description and this file ever disagree, which is
the only thing keeping the two derivations honest.

What comes from the spec: which SCARAs exist, which heads exist and how many banks each has
(`gang_heads.<id>.opposed`), and every limit: a SCARA's from its robot model,
`robot_models.<scaras.<id>.model>` (`scara_limits`, the same arithmetic as
urdf/scara.urdf.xacro, D-013), a head's from
`gang_heads.<id>.{stroke,v_max,a_max}`. What is typed here: the controller update rate and the
trajectory/goal tolerances, all four from the P1 spike (docs/decisions.md D-002), collected at the
top.

`a_max` is not a `joint_trajectory_controller` parameter in Jazzy — the controller follows the
trajectory it is handed — so it is not written to the yaml. It goes to `reports/gen_controllers
.json` as part of `joint_limits`, which is where the sequencer (P8) reads it to time a move, and
where the descriptions check reads what to compare the URDF against.

Output is a generated file (CLAUDE.md P0): edit the spec, never controllers.yaml. `--dev` merges
the placeholder overlay and writes `config/controllers.dev.yaml` with the report at
`reports/gen_controllers.dev.json`.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cad.common import Values, lookup  # noqa: E402
import math  # noqa: E402

from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, head_station,  # noqa: E402
                  load_merged, parse_spec, report_path, scara_ids)

PKG = REPO / "drone_line_sim"
HEADER = "# GENERATED from line_spec.yaml by gen_controllers.py — do not edit\n"

# ---- from the P1 spike, docs/decisions.md D-002. The only numbers typed in this file. ----
UPDATE_RATE_HZ = 100
STATE_PUBLISH_RATE = 50.0
ACTION_MONITOR_RATE = 20.0
STOPPED_VELOCITY_TOLERANCE = 0.01
GOAL_TIME_S = 1.0
# trajectory = allowed deviation while moving, goal = allowed error at the end. D-002 measured
# both on a prismatic axis; a revolute axis takes the same tolerances in radians.
TOLERANCE = {"prismatic": {"trajectory": 0.05, "goal": 0.005},
             "revolute": {"trajectory": 0.10, "goal": 0.010}}

JTC_TYPE = "joint_trajectory_controller/JointTrajectoryController"
JSB_TYPE = "joint_state_broadcaster/JointStateBroadcaster"
SCARA_JOINTS = ("j1", "j2", "j3", "j4")      # shoulder, elbow, Z (quill), wrist


# ---------------------------------------------------------------- the joints the spec implies
# Shared with tools/checks/descriptions_check.py: one derivation of "what joints exist", used by
# the generator that configures them and by the check that compares them with the description.

def scara_joints(raw: dict, sid: str) -> list[str]:
    return [f"{sid}_{j}" for j in SCARA_JOINTS]


def head_banks(raw: dict, hid: str) -> list[str]:
    """Joint-name suffixes for one head: one ram, or two when it closes from both sides."""
    head = lookup(raw, f"gang_heads.{hid}") or {}
    opposed = bool(head.get("opposed")) and head.get("direction") == "horizontal"
    return ["_stroke_1", "_stroke_2"] if opposed else ["_stroke"]


def head_joints(raw: dict, hid: str) -> list[str]:
    return [f"{hid}{s}" for s in head_banks(raw, hid)]


def pattern_positions(raw: dict, hid: str) -> list | None:
    """`pattern_from: bottom_plate.arm_holes` -> parts.bottom_plate.arm_holes.positions."""
    pattern = lookup(raw, f"gang_heads.{hid}.pattern_from")
    if not pattern:
        return None
    return lookup(raw, f"parts.{pattern}.positions")


def validate_heads(raw: dict) -> list[str]:
    """Spec coherence the description cannot express, so it has to be caught before expansion.

    A xacro that fails on one of these fails with a stack trace naming an expression; these
    messages name the spec key instead.
    """
    errors = []
    for hid, head in (raw.get("gang_heads") or {}).items():
        spindles = head.get("spindles")
        if head.get("opposed") and head.get("direction") != "horizontal":
            errors.append(f"gang_heads.{hid}: opposed is only a horizontal head — two banks "
                          f"closing on the work from both sides, not direction '"
                          f"{head.get('direction')}'")
        if head.get("opposed") and spindles is not None and spindles % 2:
            errors.append(f"gang_heads.{hid}: {spindles} spindles cannot be split between two "
                          "opposed banks")
        station = head_station(raw, hid)
        if station is None:
            errors.append(f"gang_heads.{hid}: no single station drives it — exactly one sequence "
                          f"step must name head: {hid} (that step's station is where it mounts)")
        elif station not in (raw.get("stations") or {}):
            errors.append(f"gang_heads.{hid}: the step that names it stands at '{station}', "
                          "which is not a station")
        pattern = head.get("pattern_from")
        if pattern:
            positions = pattern_positions(raw, hid)
            if positions is None:
                errors.append(f"gang_heads.{hid}.pattern_from: '{pattern}' has no positions "
                              "(expected parts.<part>.<feature>.positions)")
            elif spindles is not None and len(positions) != spindles:
                errors.append(f"gang_heads.{hid}: {spindles} spindles but "
                              f"parts.{pattern}.positions has {len(positions)} holes")
    return errors


# ---------------------------------------------------------------- limits, read through Values

def scara_limits(v: Values, sid: str) -> dict[str, dict]:
    """j1..j4 of one SCARA from its robot model, the arithmetic urdf/scara.urdf.xacro does
    (D-013): j1, j2, j4 within +/- joint_limits.<j>; j3 in [0, z_stroke]; velocity v_max on j3,
    v_max / link1 on j1, v_max / link2 on j2 (the rate that moves the link's tip at the TCP's
    Cartesian v_max: a stand-in, not a spec) and j4_speed on j4; a_max likewise (j4: a_max /
    link2)."""
    model = v.plain(f"scaras.{sid}.model")
    path = f"robot_models.{model}"
    l1, l2 = v.plain(f"{path}.link1"), v.plain(f"{path}.link2")
    lim = {j: v.plain(f"{path}.joint_limits.{j}") for j in ("j1", "j2", "j4")}
    stroke = v.plain(f"{path}.z_stroke")
    v_max, a_max = v.plain(f"{path}.motion.v_max"), v.plain(f"{path}.motion.a_max")
    w4 = v.plain(f"{path}.motion.j4_speed")
    if None in (l1, l2, stroke, v_max, a_max, w4, *lim.values()):
        return {}

    def rev(j, vel, acc):
        return {"type": "revolute", "spec_path": path, "min": -lim[j], "max": lim[j],
                "v_max": vel, "a_max": acc}
    return {f"{sid}_j1": rev("j1", v_max / l1, a_max / l1),
            f"{sid}_j2": rev("j2", v_max / l2, a_max / l2),
            f"{sid}_j3": {"type": "prismatic", "spec_path": path, "min": 0.0, "max": stroke,
                          "v_max": v_max, "a_max": a_max},
            f"{sid}_j4": rev("j4", w4, a_max / l2)}


def joint_limits(v: Values) -> dict:
    """Every movable joint the description has, with the spec values and paths behind it.

    Read through `Values`, so a null limit is a FAIL naming the key and an overlay value is
    reported as a placeholder — the same contract as every other generator.
    """
    limits: dict[str, dict] = {}
    for sid in scara_ids(v.raw):
        limits.update(scara_limits(v, sid))
    for hid in sorted(v.raw.get("gang_heads") or {}):
        path = f"gang_heads.{hid}"
        stroke = v.plain(f"{path}.stroke")
        v_max = v.plain(f"{path}.v_max")
        a_max = v.plain(f"{path}.a_max")
        for joint in head_joints(v.raw, hid):
            limits[joint] = {"type": "prismatic", "spec_path": path, "min": 0.0, "max": stroke,
                             "v_max": v_max, "a_max": a_max,
                             "damping": "", "friction": "", "position_gain": ""}
    # `plain` reports an absent optional field as "", which is not a number: drop those keys so a
    # consumer sees "no value in the spec" rather than an empty string.
    for joint in limits.values():
        for key in ("damping", "friction", "position_gain"):
            if joint.get(key) == "":
                del joint[key]
    return limits


# ---------------------------------------------------------------- the controllers

def controllers_of(raw: dict) -> dict[str, dict]:
    """`{controller_name: {"type", "joints"}}`: the broadcaster, one per SCARA, one per head."""
    out = {"joint_state_broadcaster": {"type": JSB_TYPE, "joints": []}}
    for sid in scara_ids(raw):
        out[f"{sid}_controller"] = {"type": JTC_TYPE, "joints": scara_joints(raw, sid)}
    for hid in sorted(raw.get("gang_heads") or {}):
        out[f"{hid}_controller"] = {"type": JTC_TYPE, "joints": head_joints(raw, hid)}
    return out


def render(controllers: dict[str, dict], limits: dict) -> str:
    """The yaml a controller_manager reads: the manager's own block, then one per controller."""
    manager = {"update_rate": UPDATE_RATE_HZ}
    for name, ctl in controllers.items():
        manager[name] = {"type": ctl["type"]}
    doc: dict = {"controller_manager": {"ros__parameters": manager}}

    for name, ctl in controllers.items():
        if not ctl["joints"]:
            continue                        # the broadcaster takes every joint it is given
        constraints = {"stopped_velocity_tolerance": STOPPED_VELOCITY_TOLERANCE,
                       "goal_time": GOAL_TIME_S}
        for joint in ctl["joints"]:
            constraints[joint] = dict(TOLERANCE[limits[joint]["type"]])
        doc[name] = {"ros__parameters": {
            "joints": list(ctl["joints"]),
            "command_interfaces": ["position"],
            "state_interfaces": ["position", "velocity"],
            "state_publish_rate": STATE_PUBLISH_RATE,
            "action_monitor_rate": ACTION_MONITOR_RATE,
            "allow_partial_joints_goal": False,
            "constraints": constraints,
        }}
    return HEADER + yaml.safe_dump(doc, sort_keys=False, default_flow_style=False)


def write_report(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate controllers.yaml from the spec")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--out", type=Path, default=None,
                    help=f"controller config (default {PKG / 'config' / 'controllers.yaml'})")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    out = args.out or (PKG / "config" / f"controllers{'.dev' if args.dev else ''}.yaml")
    doc: dict = {"passed": False, "dev": args.dev, "out": str(out),
                 "update_rate_hz": UPDATE_RATE_HZ, "controllers": {}, "joint_limits": {},
                 "placeholders": [], "missing": [], "errors": []}
    report = report_path("gen_controllers", args.dev)

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        write_report(report, doc)
        return code

    try:
        merged = load_merged(args.spec, args.dev)
        parse_spec(merged.raw)                      # schema first: a typo is not a missing value
    except SpecError as exc:
        doc["errors"] = exc.errors
        print(f"gen_controllers: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)

    doc["errors"] = validate_heads(merged.raw)
    if doc["errors"]:
        for e in doc["errors"]:
            print(f"gen_controllers: FAIL — {e}", file=sys.stderr)
        return finish(1)

    v = Values(merged.raw, {p["path"] for p in merged.placeholders})
    limits = joint_limits(v)
    controllers = controllers_of(merged.raw)
    doc["joint_limits"] = limits
    doc["controllers"] = controllers
    doc["placeholders"] = v.placeholders
    if v.missing:
        doc["missing"] = v.missing
        for m in v.missing:
            print(f"gen_controllers: FAIL — no value for {m}", file=sys.stderr)
        print("gen_controllers: fill the key(s) in, or re-run with --dev to use the placeholder "
              "overlay", file=sys.stderr)
        return finish(1)

    try:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(render(controllers, limits))
    except OSError as exc:
        doc["errors"].append(f"cannot write {out}: {exc}")
        print(f"gen_controllers: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)

    if args.dev:
        print(f"gen_controllers: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{out} is not a deliverable")
    print(f"gen_controllers: OK — {len(controllers)} controller(s) over {len(limits)} joint(s) "
          f"-> {out}")
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
