"""Regenerate every build123d part, then the SolidWorks meshes (plan P4, *Check*).

    tools/.venv/bin/python tools/cad/build_all.py [--out DIR] [--spec PATH] [--dev]

One command for "make the CAD again": it runs each generator in build order — pallet, the two
carriers, the magazines, the screw presenters, the gantry tools — and then mesh_pipeline.py, which
is what turns the raw SolidWorks exports into committed meshes. Everything runs **in this process**,
so build123d and OCP are imported once for the whole build instead of once per generator, which is
most of the plan's 30 s budget.

Nothing is decided here. Each generator owns its geometry, its checks and its own
reports/cad_<part>.json; this file only says what to run, in what order, and gathers the results
into reports/cad_all.json: per step the exit code, how long it took, and what it wrote, plus every
output STL measured against the 5 MB rule (CLAUDE.md P0).

Exit code is non-zero when any generator fails, when the whole build misses `--budget-s`, or when
an output breaks the 5 MB rule. A missing `--src` folder of raw SolidWorks STL is a skip, not a
failure — the build123d half of the CAD has to stay runnable before any SolidWorks export exists;
pass `--require-meshes` to insist on it.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))         # tools/cad: the generators
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: spec, mesh_pipeline

import magazine  # noqa: E402
import mesh_pipeline  # noqa: E402
import pallet  # noqa: E402
import pod_tray  # noqa: E402
import screw_presenter  # noqa: E402
import spider_carrier  # noqa: E402
import tools as gantry_tools  # noqa: E402
from common import write_report  # noqa: E402
from spec import DEFAULT_SPEC, REPO, add_dev_argument, report_path  # noqa: E402

DEFAULT_OUT = REPO / "cad_out"
MAX_BYTES = 5 * 1024 * 1024          # P0: no STL over 5 MB is ever committed

# Build order (plan P4 B1-B7): each generator, the report it writes, and the module that owns it.
# pins.py has no CLI of its own — it is the pallet's, and the pallet builds it.
GENERATORS = [
    ("pallet", "cad_pallet", pallet),
    ("spider_carrier", "cad_carrier_spider", spider_carrier),
    ("pod_tray", "cad_carrier_pod", pod_tray),
    ("magazine", "cad_magazines", magazine),
    ("screw_presenter", "cad_presenters", screw_presenter),
    ("tools", "cad_tools", gantry_tools),
]


def read_report(name: str, dev: bool) -> dict:
    """Whatever the generator just wrote, or an empty doc if it could not write one."""
    path = report_path(name, dev)
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def outputs_of(doc: dict) -> list[str]:
    """Every file a report names, whether it built one part or one per spec entry."""
    files = list(doc.get("outputs", {}).values())
    for item in (doc.get("items") or {}).values():
        files += list(item.get("outputs", {}).values())
    for mesh in (doc.get("meshes") or {}).values():
        if mesh.get("out"):
            files.append(mesh["out"])
    return files


def run(step: dict, call, *, label: str) -> dict:
    """Time one generator, record what it wrote, and keep going."""
    start = time.perf_counter()
    step["code"] = call()
    step["seconds"] = round(time.perf_counter() - start, 3)
    step["status"] = "PASS" if step["code"] == 0 else "FAIL"
    print(f"build_all: {label} {step['status']} in {step['seconds']:.1f} s", file=sys.stderr)
    return step


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Run every cad generator, then mesh_pipeline")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help=f"default {DEFAULT_OUT}")
    ap.add_argument("--src", type=Path, default=mesh_pipeline.DEFAULT_SRC,
                    help="folder of raw SolidWorks STL (mm) for mesh_pipeline")
    ap.add_argument("--budget-s", type=float, default=30.0,
                    help="the whole build has to fit in this (plan P4 check); default %(default)s")
    ap.add_argument("--require-meshes", action="store_true",
                    help="fail instead of skipping when --src does not exist")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    out_dir = args.out / "dev" if args.dev else args.out
    doc = {"passed": False, "dev": args.dev, "out": str(out_dir), "spec": str(args.spec),
           "budget_s": args.budget_s, "elapsed_s": 0.0, "steps": [], "outputs": [],
           "placeholders": [], "warnings": [], "errors": []}
    report = report_path("cad_all", args.dev)
    started = time.perf_counter()

    def finish(code: int) -> int:
        doc["elapsed_s"] = round(time.perf_counter() - started, 3)
        doc["passed"] = code == 0
        write_report(report, doc)
        status = "OK" if code == 0 else "FAIL"
        print(f"build_all: {status} — {sum(s['status'] == 'PASS' for s in doc['steps'])}/"
              f"{len(doc['steps'])} step(s) passed, {len(doc['outputs'])} file(s) in "
              f"{doc['elapsed_s']:.1f} s (budget {args.budget_s:g} s)")
        return code

    common = ["--spec", str(args.spec), "--out", str(args.out)] + (["--dev"] if args.dev else [])
    for label, name, module in GENERATORS:
        step = {"name": label, "generator": f"tools/cad/{label}.py", "report": str(
            report_path(name, args.dev))}
        run(step, lambda m=module: m.main(common), label=label)
        sub = read_report(name, args.dev)
        step["outputs"] = outputs_of(sub)
        step["warnings"] = sub.get("warnings", [])
        doc["steps"].append(step)
        doc["outputs"] += step["outputs"]
        doc["warnings"] += [f"{label}: {w}" for w in step["warnings"]]
        for path in sub.get("placeholders", []):
            if path not in doc["placeholders"]:
                doc["placeholders"].append(path)
        if step["code"] != 0:
            doc["errors"].append(f"{label} failed; see {step['report']}")

    # mesh_pipeline is not a generator: it processes raw SolidWorks exports, which may not exist
    # yet. Its absence must not stop the build123d half of the CAD from being regenerated.
    mesh_step = {"name": "mesh_pipeline", "generator": "tools/mesh_pipeline.py",
                 "report": str(report_path("meshes", args.dev)), "outputs": [], "warnings": []}
    if not args.src.is_dir() and not args.require_meshes:
        mesh_step.update(status="SKIP", code=0, seconds=0.0,
                         note=f"no raw STL folder at {args.src}")
        doc["warnings"].append(f"mesh_pipeline: skipped, no raw STL folder at {args.src}")
        print(f"build_all: mesh_pipeline SKIP — no raw STL folder at {args.src}", file=sys.stderr)
    else:
        run(mesh_step, lambda: mesh_pipeline.main(
            ["--spec", str(args.spec), "--src", str(args.src)] + (["--dev"] if args.dev else [])),
            label="mesh_pipeline")
        sub = read_report("meshes", args.dev)
        mesh_step["outputs"] = outputs_of(sub)
        doc["outputs"] += mesh_step["outputs"]
        if mesh_step["code"] != 0:
            doc["errors"].append(f"mesh_pipeline failed; see {mesh_step['report']}")
    doc["steps"].append(mesh_step)

    doc["sizes"] = {}
    for path in doc["outputs"]:
        if not path.endswith(".stl"):
            continue
        size = Path(path).stat().st_size if Path(path).is_file() else 0
        doc["sizes"][path] = size
        if size >= MAX_BYTES:
            doc["errors"].append(f"{path} is {size / 1024 / 1024:.1f} MB: over the 5 MB rule")

    elapsed = round(time.perf_counter() - started, 3)
    if elapsed > args.budget_s:
        doc["errors"].append(f"the build took {elapsed:.1f} s, over the {args.budget_s:g} s budget "
                             "the plan gives it (P4 check)")

    for w in doc["warnings"]:
        print(f"build_all: WARN {w}", file=sys.stderr)
    for e in doc["errors"]:
        print(f"build_all: FAIL — {e}", file=sys.stderr)
    if args.dev:
        print(f"build_all: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{out_dir} is not a deliverable")
    return finish(1 if doc["errors"] else 0)


if __name__ == "__main__":
    sys.exit(main())
