"""Query a running gz world and check every model is at its spec pose (plan P5.4 check).

    tools/.venv/bin/python tools/checks/world_measure.py --report reports/world.json [--dev]

Started by tools/checks/world_smoke.sh, which owns the gz server; this half only measures. For
every model in `reports/gen_world.json` — the manifest gen_world.py wrote — it asks the running
world where that model actually is and compares:

  * a model whose manifest entry names a **spec key** (`stations.<id>.pose`, `feeders.<id>.pose`,
    …) is compared against **the spec**, not against the manifest. That is the whole point: the
    chain spec -> gen_world -> sdf -> gz is checked end to end, and the manifest is trusted only
    for *which* spec key a model came from;
  * a test cell is compared against its station's spec pose plus `k x cell_pitch`, the rule the
    world is supposed to have laid them out by;
  * a model with no spec key of its own — the frame, the conveyor — is compared against the pose
    the manifest claims, which at least catches a world that did not load what it said it did.

A model in the manifest that the running world does not list is a FAIL: gz resolves includes at
parse time, so a model that is not there means the world did not load it.

Writes reports/world.json and exits non-zero on any failure.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cad.common import lookup  # noqa: E402
from gen_world import CELL_AXIS, WORLD_NAME  # noqa: E402
from spec import DEFAULT_SPEC, REPO, SpecError, add_dev_argument, load_merged  # noqa: E402

POSE_RE = re.compile(r"\[\s*(-?\d+\.?\d*(?:e-?\d+)?)\s+(-?\d+\.?\d*(?:e-?\d+)?)\s+"
                     r"(-?\d+\.?\d*(?:e-?\d+)?)\s*\]")
CELL_RE = re.compile(r"^(?P<station>.+)_Cell(?P<k>\d+)$")
DEFAULT_TOL_MM = 1.0


def gz(args: list[str], timeout: float) -> str:
    """One `gz` call. An empty string means it failed — the caller decides what that means."""
    try:
        out = subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return out.stdout


def model_list(timeout: float) -> list[str]:
    text = gz(["model", "--list"], timeout)
    names = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("- "):
            names.append(line[2:].strip())
    return names


def model_pose(name: str, timeout: float) -> list[float] | None:
    """(x, y, z) of one model, asked for by name. The fallback when the bulk read misses one."""
    found = POSE_RE.findall(gz(["model", "-m", name, "-p"], timeout))
    if not found:
        return None
    return [float(c) for c in found[0]]        # first triple is XYZ, the second is RPY


def parse_pose_v(text: str) -> dict[str, list[float]]:
    """{entity name: [x, y, z]} out of a Pose_V text message.

    One query answers for every model in the world, which is the difference between this check
    costing a second and costing half its budget: a `gz model -p` per model does not scale, and
    past a handful of them in parallel the service starts refusing.

    Protobuf text omits zero-valued fields, so a missing `x:` line means x = 0. Only the first
    entry for a name is kept: models come before the links inside them.
    """
    poses: dict[str, list[float]] = {}
    name, section, xyz = None, None, [0.0, 0.0, 0.0]
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("pose {"):
            name, section, xyz = None, None, [0.0, 0.0, 0.0]
        elif line.startswith("name:") and '"' in line:
            name = line.split('"')[1]
        elif line.startswith(("position {", "orientation {")):
            section = line.split()[0]
        elif line == "}":
            if section is not None:
                section = None
            elif name:
                poses.setdefault(name, xyz)
                name = None
        elif section == "position" and ":" in line:
            axis, _, value = line.partition(":")
            if axis.strip() in ("x", "y", "z"):
                try:
                    xyz["xyz".index(axis.strip())] = float(value)
                except ValueError:
                    pass
    return poses


def world_poses(world: str, names: list[str], timeout: float) -> dict[str, list[float] | None]:
    """Every model's live pose: one bulk read, then a direct query for anything it missed."""
    bulk = parse_pose_v(gz(["topic", "-e", "-t", f"/world/{world}/pose/info", "-n", "1"], timeout))
    return {n: bulk.get(n) if n in bulk else model_pose(n, timeout) for n in names}


def expected_pose(raw: dict, name: str, entry: dict) -> tuple[list[float], str] | None:
    """Where the spec says this model belongs, and the key that says so.

    None when the spec has nothing to say about it, in which case the caller falls back to the
    manifest's own pose.
    """
    path = entry.get("spec_path")
    if not path:
        return None
    stated = lookup(raw, path)
    if not stated or any(c is None for c in stated[:3]):
        return None
    pose = [float(c) for c in stated[:3]]
    cell = CELL_RE.match(name)
    if cell:
        # The cells of a station are parallel copies laid out at cell_pitch (P5.1).
        pitch = lookup(raw, f"{path.rsplit('.', 1)[0]}.cell_pitch")
        if pitch is None:
            return None
        pose[CELL_AXIS] += (int(cell.group("k")) - 1) * float(pitch)
        return pose, f"{path} + {int(cell.group('k')) - 1} x cell_pitch"
    return pose, path


def check(raw: dict, manifest: dict, timeout: float, tol_mm: float) -> dict:
    """Compare every model in the manifest with where the running world actually has it."""
    live = model_list(timeout)
    wanted = [n for n in (manifest.get("models") or {}) if n in live]
    actual_poses = world_poses(WORLD_NAME, wanted, timeout)
    doc = {"world": WORLD_NAME, "tol_mm": tol_mm, "models": {}, "live_models": live,
           "missing_models": [], "extra_models": [], "errors": []}

    for name, entry in (manifest.get("models") or {}).items():
        result = {"kind": entry.get("kind"), "spec_path": entry.get("spec_path"),
                  "expected_m": None, "actual_m": None, "error_mm": None, "status": "FAIL"}
        doc["models"][name] = result
        if name not in live:
            result["note"] = "not in the running world"
            doc["missing_models"].append(name)
            doc["errors"].append(f"{name} is in the world file but the running world does not "
                                 "list it: the model did not load")
            continue
        actual = actual_poses.get(name)
        if actual is None:
            result["note"] = "pose could not be read"
            doc["errors"].append(f"{name}: could not read a pose from `gz model -m {name} -p`")
            continue
        wanted = expected_pose(raw, name, entry)
        if wanted is None:
            expected, source = [float(c) for c in entry["pose"]], "the world manifest (derived)"
        else:
            expected, source = wanted
        error_mm = max(abs(a - e) for a, e in zip(actual, expected)) * 1000.0
        result.update(expected_m=[round(c, 6) for c in expected],
                      actual_m=[round(c, 6) for c in actual],
                      error_mm=round(error_mm, 4), from_=source,
                      status="PASS" if error_mm <= tol_mm else "FAIL")
        if error_mm > tol_mm:
            doc["errors"].append(f"{name} is {error_mm:.2f} mm from {source} "
                                 f"(expected {expected}, world has {actual})")

    known = set(doc["models"]) | {"Ground"}
    doc["extra_models"] = [n for n in live if n not in known]
    return doc


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Check a running world against the spec")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--manifest", type=Path, required=True,
                    help="reports/gen_world[.dev].json, written by gen_world.py")
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--tol-mm", type=float, default=DEFAULT_TOL_MM)
    ap.add_argument("--gz-timeout", type=float, default=10.0,
                    help="seconds to wait for one gz query")
    ap.add_argument("--elapsed-s", type=float, default=0.0)
    ap.add_argument("--iterations", type=int, default=0, help="what the server was run for")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    started = time.perf_counter()
    doc: dict = {"passed": False, "dev": args.dev, "iterations": args.iterations,
                 "manifest": str(args.manifest), "errors": []}

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        doc["elapsed_s"] = round(args.elapsed_s + time.perf_counter() - started, 3)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(doc, indent=2) + "\n")
        return code

    try:
        raw = load_merged(args.spec, args.dev).raw
    except SpecError as exc:
        doc["errors"] = exc.errors
        print(f"world_smoke: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)
    try:
        manifest = json.loads(args.manifest.read_text())
    except (OSError, ValueError) as exc:
        doc["errors"].append(f"cannot read {args.manifest}: {exc} (run tools/gen_world.py first)")
        print(f"world_smoke: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)
    doc["world_file"] = manifest.get("out")

    doc.update(check(raw, manifest, args.gz_timeout, args.tol_mm))
    passed = sum(1 for m in doc["models"].values() if m["status"] == "PASS")
    for e in doc["errors"]:
        print(f"world_smoke: FAIL — {e}", file=sys.stderr)
    for name in doc["extra_models"]:
        print(f"world_smoke: WARN the world has a model the manifest does not: {name}",
              file=sys.stderr)
    print(f"world_smoke: {'OK' if not doc['errors'] else 'FAIL'} — {passed}/{len(doc['models'])} "
          f"model(s) within {args.tol_mm:g} mm of the spec "
          f"(relative to {Path(REPO).name}/{args.report.name})")
    return finish(1 if doc["errors"] else 0)


if __name__ == "__main__":
    sys.exit(main())
