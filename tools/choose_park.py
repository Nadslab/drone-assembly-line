"""Choose each SCARA's dev park pose (D-014): where it waits while its station's gang head works.

    tools/.venv/bin/python tools/choose_park.py --dev [--write] [--stations IDS] [--cad-reports DIR]

For every SCARA whose station drives a gang head (`--stations`: only the SCARAs at those
stations): j2 folded to its limit (on the elbow side of the place pose), j1 searched over its
joint limits for the value closest to the place pose's j1 whose arm (vendor STEP capsules,
tools/robot_geometry.py) clears the head's working envelope — the head at its working position
plus its slide frame — by at least robot_geometry.PARK_CLEAR_M in plan (D-015), keeps clear of
its station's refill envelopes (refilled buffers and, D-016, operator-refilled magazines; j2 is
never eased for a magazine: the SCARA is reported instead), and leaves every path robot_geometry.PATH_CLEAR_M clear of the
waiting heads and their slide frames (robot_geometry.choose_park).

`--write` puts the result into line_spec.dev.yaml, on each SCARA's
`park_pose: {...}  # PLACEHOLDER (auto-chosen)` line; only that line changes. The base spec's
park_pose stays null: a park pose found from placeholder geometry is a placeholder. Writes
reports/park_pose.dev.json; exits non-zero when a SCARA has no clearing pose.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import robot_geometry as rg  # noqa: E402
from spec import (DEFAULT_SPEC, SpecError, add_dev_argument, load_merged, overlay_path,  # noqa: E402
                  parse_spec, parse_station_list, report_path)

TAG = "# PLACEHOLDER (auto-chosen)"


def write_overlay(path: Path, chosen: dict[str, dict]) -> list[str]:
    """Rewrite the `park_pose:` line under each chosen SCARA in the overlay; returns the ids
    whose line was not found."""
    lines = path.read_text().splitlines(keepends=True)
    missing = []
    for sid, c in chosen.items():
        head = next((i for i, ln in enumerate(lines) if ln.rstrip() == f"  {sid}:"), None)
        at = None
        if head is not None:
            for i in range(head + 1, len(lines)):
                if re.match(r"^  \S", lines[i]):
                    break
                if lines[i].lstrip().startswith("park_pose:"):
                    at = i
                    break
        if at is None:
            missing.append(sid)
            continue
        body = f"    park_pose: {{j1: {c['j1']}, j2: {c['j2']}}}"
        lines[at] = (f"{body:<46}{TAG} j1 {c['j1_deg']:g}°, j2 {c['j2_deg']:g}°, clears "
                     f"{c['clearance_m'] * 1000:.0f} mm in plan\n")
    path.write_text("".join(lines))
    return missing


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--cad-reports", type=Path, default=None,
                    help=f"where vendor_<model>.json and cad_<part>.json live "
                         f"(default {rg.CAD_REPORTS})")
    ap.add_argument("--write", action="store_true",
                    help="write the poses into line_spec.dev.yaml (needs --dev)")
    ap.add_argument("--stations", nargs="+", default=None,
                    help="choose only for the SCARAs at these stations (comma or space separated)")
    add_dev_argument(ap)
    args = ap.parse_args(argv)
    only = parse_station_list(args.stations)
    report = report_path("park_pose", args.dev)
    doc: dict = {"dev": args.dev, "need_m": rg.PARK_CLEAR_M, "step_deg": rg.PARK_STEP_DEG,
                 "scaras": {}, "errors": [], "passed": False}

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(doc, indent=2) + "\n")
        return code

    if args.write and not args.dev:
        doc["errors"].append("--write only fills the dev overlay; pass --dev")
        print(f"choose_park: FAIL — {doc['errors'][0]}", file=sys.stderr)
        return finish(2)
    try:
        spec = parse_spec(load_merged(args.spec, args.dev).raw)
    except SpecError as exc:
        doc["errors"] = exc.errors
        print(f"choose_park: FAIL — {exc}", file=sys.stderr)
        return finish(1)
    cad = rg.CadHeights(args.cad_reports, args.dev)
    chosen = {}
    unknown = [st for st in only or [] if st not in spec.stations]
    if unknown:
        doc["errors"].append(f"--stations: {', '.join(unknown)} not in the spec")
        print(f"choose_park: FAIL — {doc['errors'][0]}", file=sys.stderr)
        return finish(2)
    for sid in spec.scaras:
        ends, starts = rg.park_steps(spec, sid)
        if not ends and not starts or (only and spec.scaras[sid].station not in only):
            continue
        c = rg.choose_park(spec, sid, cad)
        doc["scaras"][sid] = c
        if "error" in c:
            doc["errors"].append(c["error"])
            print(f"choose_park: {sid}: FAIL — {c['error']}")
            continue
        chosen[sid] = c
        near = c["nearest"]
        print(f"choose_park: {sid}: j1 {c['j1_deg']:g}° (place j1 "
              f"{math.degrees(c['place_j'][0]):.1f}°), j2 {c['j2_deg']:g}°, clears "
              f"{c['clearance_m'] * 1000:.0f} mm in plan (nearest: {near['arm']} to {near['head']} "
              f"{near['part']}); refill {c['refill_clearance_m']}, path {c['path_clearance_m']}; "
              f"fold eased {c['fold_eased_deg']:g}°; "
              f"nearer poses rejected: {c['rejected_nearer'] or 'none'}"
              + (f"; UNRESOLVED {c['unresolved']}" if c["unresolved"] else ""))
    if args.write and chosen:
        missing = write_overlay(overlay_path(args.spec), chosen)
        for sid in missing:
            doc["errors"].append(f"{sid}: no `park_pose:` line under it in the overlay")
        print(f"choose_park: wrote {len(chosen) - len(missing)} park pose(s) -> "
              f"{overlay_path(args.spec)}")
    print(f"-> {report}")
    return finish(1 if doc["errors"] else 0)


if __name__ == "__main__":
    sys.exit(main())
