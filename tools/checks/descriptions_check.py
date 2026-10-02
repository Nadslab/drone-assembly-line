"""P5.3 check: the robot descriptions expand, parse, and say what the spec says.

    tools/checks/descriptions.sh [--real] [--keep]        # the wrapper that sources ROS
    tools/.venv/bin/python tools/checks/descriptions_check.py [--dev]

Four questions, in order — a later one is worthless if an earlier one fails:

  1. **does the spec hold up?** schema, then `gen_controllers.validate_heads` (a head with no
     station, an opposed head that is not horizontal, a pattern that does not match the spindle
     count). A xacro that trips over one of these fails with a python expression in the message;
     these fail with a spec key.
  2. **does every description expand and parse?** `xacro` on every top-level file in `urdf/`
     (a `<robot>` with a `name`; the macro libraries are expanded by including them), then
     `check_urdf` on the result.
  3. **does it say what the spec says?** every movable joint in `urdf/line.urdf.xacro`, compared
     with the spec: type, `lower`, `upper`, `velocity`, and `damping`/`friction` where the spec
     has them. Plus the spindle frames: one per spindle, at the pattern's own positions.
  4. **do the description and the controller config agree?** the movable joints of the URDF and
     the joints `config/controllers.yaml` configures have to be the same set, because nothing else
     ties the two derivations together (tools/gen_controllers.py builds the names in python, the
     xacro builds them in xacro).

Both generated files are rebuilt first, so the check can never pass on a stale one — the
tools/checks/world_smoke.sh rule. Writes reports/descriptions.json (`--dev`:
reports/descriptions.dev.json, and the expansion reads the merged overlay spec).
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))

import gen_controllers as gc  # noqa: E402
from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, dump_merged,  # noqa: E402
                  load_merged, parse_spec, report_path, resolve_anchors)

PKG = REPO / "drone_line_sim"
URDF_DIR = PKG / "urdf"
LINE_XACRO = "line.urdf.xacro"
TOL = 1e-9          # xacro writes the spec's own floats; anything above this is a real difference
POSE_TOL = 1e-6     # a position in metres: a micron, the tolerance the world check uses


def sh(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


# ---------------------------------------------------------------- expanding

def declared_args(path: Path) -> set[str]:
    return set(re.findall(r'<xacro:arg\s+name="([^"]+)"', path.read_text()))


def top_level_xacros(urdf_dir: Path) -> tuple[list[Path], list[str]]:
    """Every xacro that is a robot, and the macro libraries that are not.

    A macro library has no `name` on its `<robot>`: it is not a description, and expanding it
    alone would only prove that a file of macros is a file of macros.
    """
    robots, libraries = [], []
    for path in sorted(urdf_dir.glob("*.urdf.xacro")):
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            robots.append(path)            # let the expansion report it properly
            print(f"descriptions: {path.name} does not parse as XML: {exc}", file=sys.stderr)
            continue
        (robots if root.get("name") else libraries).append(path if root.get("name") else path.name)
    return robots, libraries


def expand(path: Path, out: Path, args: dict[str, Path]) -> tuple[bool, str]:
    """xacro, then check_urdf. Only the args the file declares are passed."""
    declared = declared_args(path)
    cmd = ["xacro", str(path)] + [f"{k}:={v}" for k, v in args.items() if k in declared]
    out.parent.mkdir(parents=True, exist_ok=True)
    result = sh(cmd + ["-o", str(out)])
    if result.returncode:
        # xacro says what failed first, then the macro stack under it; the head is the useful part.
        lines = result.stderr.strip().splitlines() or [str(result.returncode)]
        return False, "xacro failed: " + " | ".join(line.strip() for line in lines[:3])
    parsed = sh(["check_urdf", str(out)])
    if parsed.returncode:
        return False, f"check_urdf rejected it: {parsed.stderr.strip() or parsed.stdout.strip()}"
    return True, "ok"


# ---------------------------------------------------------------- comparing

def joints_of(urdf: Path) -> dict[str, ET.Element]:
    return {j.get("name"): j for j in ET.parse(urdf).getroot().findall("joint")}


def attr(element: ET.Element | None, name: str) -> float | None:
    if element is None or element.get(name) is None:
        return None
    return float(element.get(name))


def close(a: float | None, b: float | None, tol: float = TOL) -> bool:
    return a is not None and b is not None and abs(a - b) <= tol


def compare_joint(name: str, want: dict, joint: ET.Element | None) -> dict:
    """One row of the report: what the spec says, what the URDF says, and whether they agree."""
    row: dict = {"joint": name, "spec_path": want["spec_path"], "expected": {
        "type": want["type"], "lower": want["min"], "upper": want["max"],
        "velocity": want["v_max"]}, "errors": []}
    for key in ("damping", "friction"):
        if key in want:
            row["expected"][key] = want[key]
    if joint is None:
        row["errors"].append(f"{name}: the description has no such joint")
        row["ok"] = False
        return row

    limit, dynamics = joint.find("limit"), joint.find("dynamics")
    row["actual"] = {"type": joint.get("type"), "lower": attr(limit, "lower"),
                     "upper": attr(limit, "upper"), "velocity": attr(limit, "velocity"),
                     "effort": attr(limit, "effort")}
    if dynamics is not None:
        row["actual"]["damping"] = attr(dynamics, "damping")
        row["actual"]["friction"] = attr(dynamics, "friction")

    if joint.get("type") != want["type"]:
        row["errors"].append(f"{name}: {joint.get('type')} joint, spec says {want['type']} "
                             f"({want['spec_path']})")
    for key, spec_value in row["expected"].items():
        if key == "type":
            continue
        if not close(row["actual"].get(key), float(spec_value)):
            row["errors"].append(f"{name}: {key}={row['actual'].get(key)}, spec says "
                                 f"{spec_value} ({want['spec_path']})")
    # A URDF joint needs an effort, and a zero one is a joint that cannot hold itself up.
    if not (row["actual"]["effort"] and row["actual"]["effort"] > 0.0):
        row["errors"].append(f"{name}: effort={row['actual']['effort']}, expected a positive force")
    row["ok"] = not row["errors"]
    return row


def compare_spindles(raw: dict, hid: str, joints: dict[str, ET.Element]) -> dict:
    """One frame per spindle, and at the pattern's own positions when the spec names a pattern."""
    name = "GangHead_" + "".join(w.capitalize() for w in hid.removeprefix("gh_").split("_"))
    want = gc.lookup(raw, f"gang_heads.{hid}.spindles")
    pattern = gc.lookup(raw, f"gang_heads.{hid}.pattern_from")
    positions = gc.pattern_positions(raw, hid)
    frames = {n: j for n, j in joints.items() if re.fullmatch(rf"{name}_spindle_\d+_frame", n)}
    row: dict = {"head": hid, "link_prefix": name, "expected": want, "actual": len(frames),
                 "pattern_from": pattern, "errors": []}
    if want != len(frames):
        row["errors"].append(f"{hid}: {len(frames)} spindle frame(s), spec says {want} "
                             f"(gang_heads.{hid}.spindles)")
    if pattern and positions:
        # The head mounts over the station, so a spindle frame's x,y in the ram frame is the hole's
        # own x,y in the part frame — the same list the spider carrier's pockets come from (P4).
        placed = []
        for i in range(1, len(positions) + 1):
            origin = frames.get(f"{name}_spindle_{i}_frame", ET.Element("joint")).find("origin")
            xyz = (origin.get("xyz") if origin is not None else None) or ""
            placed.append([float(c) for c in xyz.split()[:2]] if xyz else None)
        row["positions"] = {"spec": [list(p) for p in positions], "urdf": placed}
        for i, (hole, put) in enumerate(zip(positions, placed), start=1):
            if put is None or not all(close(a, float(b), POSE_TOL) for a, b in zip(put, hole)):
                row["errors"].append(f"{hid}: spindle {i} at {put}, "
                                     f"parts.{pattern}.positions[{i - 1}] is {list(hole)}")
    row["ok"] = not row["errors"]
    return row


def compare_mounts(raw: dict, joints: dict[str, ET.Element]) -> list[dict]:
    """Where the machines are bolted: each SCARA at its resolved base pose (scaras.<id>.pose), a
    head at its resolved working pose (gang_heads.<id>.pose), both from offsets in their
    station's anchor frame (D-013).

    A head stands off along z by its own stroke (urdf/gang_head.urdf.xacro), so only x and y are
    the pose's to give; z is recorded, not asserted.
    """
    rows = []
    raw, _ = resolve_anchors(raw)           # poses come from anchors and offsets (D-013)

    def xyz_of(joint: str) -> list[float] | None:
        origin = joints.get(joint, ET.Element("joint")).find("origin")
        if origin is None:
            return None
        return [float(c) for c in (origin.get("xyz") or "").split()[:3]]

    for sid in gc.scara_ids(raw):
        pose = gc.lookup(raw, f"scaras.{sid}.pose")
        base = list(pose[:3]) if pose and None not in pose[:3] else None
        put = xyz_of(f"{sid}_mount")
        path = f"scaras.{sid}.pose (its offset in its station's frame)"
        rows.append({"what": sid, "joint": f"{sid}_mount", "spec_path": path,
                     "expected": base, "actual": put, "axes": "xyz",
                     "errors": [] if (put and base and all(close(a, float(b), POSE_TOL)
                                                           for a, b in zip(put, base)))
                     else [f"{sid}_mount at {put}, the spec puts the base at {base}"]})
    for hid in sorted(raw.get("gang_heads") or {}):
        pose = gc.lookup(raw, f"gang_heads.{hid}.pose") or []
        put = xyz_of(f"{hid}_mount")
        ok = put and len(pose) >= 2 and all(close(put[i], float(pose[i]), POSE_TOL)
                                            for i in range(2))
        rows.append({"what": hid, "joint": f"{hid}_mount",
                     "spec_path": f"gang_heads.{hid}.pose", "expected": list(pose[:3]),
                     "actual": put, "axes": "xy",
                     "errors": [] if ok else [f"{hid}_mount at {put}, its pose is "
                                              f"{list(pose[:3])} (x and y must match)"]})
    for row in rows:
        row["ok"] = not row["errors"]
    return rows


def controller_joints(path: Path) -> list[str]:
    """Every joint config/controllers.yaml hands to a controller."""
    doc = yaml.safe_load(path.read_text()) or {}
    joints = []
    for name, block in doc.items():
        if name == "controller_manager":
            continue
        joints += list((block or {}).get("ros__parameters", {}).get("joints") or [])
    return joints


# ---------------------------------------------------------------- the check

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Check the robot descriptions against the spec")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--urdf-dir", type=Path, default=URDF_DIR)
    ap.add_argument("--out-dir", type=Path, default=REPO / "reports" / "urdf",
                    help="where the expanded descriptions go (default %(default)s)")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    suffix = ".dev" if args.dev else ""
    merged_spec = REPO / "reports" / f"line_spec{suffix}.yaml"
    controllers = PKG / "config" / f"controllers{suffix}.yaml"
    doc: dict = {"passed": False, "dev": args.dev, "stage": "spec", "spec": str(args.spec),
                 "merged_spec": str(merged_spec), "controllers": str(controllers),
                 "expanded": {}, "macro_libraries": [], "joints": [], "spindles": [],
                 "mounts": [], "controller_joints": [], "placeholders": [], "errors": []}
    report = report_path("descriptions", args.dev)

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(doc, indent=2) + "\n")
        if code:
            for e in doc["errors"]:
                print(f"descriptions: FAIL [{doc['stage']}] {e}", file=sys.stderr)
            print(f"descriptions: FAIL (see {report})", file=sys.stderr)
        return code

    # 1. the spec
    try:
        merged = load_merged(args.spec, args.dev)
        parse_spec(merged.raw)
    except SpecError as exc:
        doc["errors"] = exc.errors
        return finish(1)
    raw = merged.raw
    doc["placeholders"] = [p["path"] for p in merged.placeholders]
    doc["errors"] = gc.validate_heads(raw)
    if doc["errors"]:
        return finish(1)

    # 2. the generated inputs: the spec as one file (xacro's load_yaml takes one path) and the
    #    controller config, both rebuilt so the check cannot pass on a stale file.
    doc["stage"] = "generate"
    dump_merged(merged, merged_spec, args.dev)
    code = gc.main(["--spec", str(args.spec), "--out", str(controllers)]
                   + (["--dev"] if args.dev else []))
    if code:
        doc["errors"].append(f"gen_controllers exited {code} — see "
                             f"{os.path.relpath(report_path('gen_controllers', args.dev), REPO)}")
        return finish(1)

    # 3. expand every description
    doc["stage"] = "expand"
    for tool in ("xacro", "check_urdf"):
        if not shutil.which(tool):
            doc["errors"].append(f"{tool} is not on PATH — source /opt/ros/jazzy/setup.bash and "
                                 "the workspace (tools/checks/descriptions.sh does both)")
            return finish(1)
    robots, libraries = top_level_xacros(args.urdf_dir)
    doc["macro_libraries"] = libraries
    if not any(r.name == LINE_XACRO for r in robots):
        doc["errors"].append(f"{args.urdf_dir / LINE_XACRO} is not a top-level description")
        return finish(1)
    for path in robots:
        out = args.out_dir / f"{path.name.removesuffix('.urdf.xacro')}{suffix}.urdf"
        ok, message = expand(path, out, {"spec_file": merged_spec,
                                         "controllers_file": controllers})
        doc["expanded"][path.name] = {"urdf": os.path.relpath(out, REPO), "result": message}
        if not ok:
            doc["errors"].append(f"{path.name}: {message}")
    if doc["errors"]:
        return finish(1)

    # 4. the line description against the spec
    doc["stage"] = "compare"
    line_urdf = args.out_dir / f"line{suffix}.urdf"
    joints = joints_of(line_urdf)
    limits = gc.joint_limits(gc.Values(raw, {p["path"] for p in merged.placeholders}))
    for name, want in limits.items():
        row = compare_joint(name, want, joints.get(name))
        doc["joints"].append(row)
        doc["errors"] += row["errors"]
    for hid in sorted(raw.get("gang_heads") or {}):
        row = compare_spindles(raw, hid, joints)
        doc["spindles"].append(row)
        doc["errors"] += row["errors"]
    for row in compare_mounts(raw, joints):
        doc["mounts"].append(row)
        doc["errors"] += row["errors"]
    if doc["errors"]:
        return finish(1)

    # 5. description vs controller config
    doc["stage"] = "controllers"
    configured = controller_joints(controllers)
    described = [n for n, j in joints.items() if j.get("type") != "fixed"]
    doc["controller_joints"] = sorted(configured)
    if sorted(configured) != sorted(described):
        only_config = sorted(set(configured) - set(described))
        only_urdf = sorted(set(described) - set(configured))
        doc["errors"].append(
            f"{controllers.name} and the description disagree about the joints: "
            f"configured but not described {only_config}, described but not configured {only_urdf}")
    if len(configured) != len(set(configured)):
        doc["errors"].append(f"{controllers.name} gives a joint to more than one controller: "
                             f"{sorted(j for j in configured if configured.count(j) > 1)}")
    if doc["errors"]:
        return finish(1)

    doc["stage"] = "done"
    if args.dev:
        print(f"descriptions: DEV MODE — {len(doc['placeholders'])} placeholder value(s) in the "
              "spec; these descriptions are not a deliverable")
    print(f"descriptions: OK — {len(doc['expanded'])} description(s) expanded and parsed, "
          f"{len(doc['joints'])} joint limit(s) and "
          f"{sum(s['expected'] or 0 for s in doc['spindles'])} spindle frame(s) match the spec "
          f"-> {report}")
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
