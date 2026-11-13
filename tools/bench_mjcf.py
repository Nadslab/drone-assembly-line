"""Headless MuJoCo speed benchmark for a generated MJCF (M1): physics only, no rendering.

    tools/.venv/bin/python tools/bench_mjcf.py [XML ...] [--secs 10] [--min-rt 1.0]

For each file (default drone_line_sim/mjcf/line.dev.xml), from its first keyframe:
  * rest:   mj_step for --secs of wall time, controls held at the keyframe;
  * moving: the same with every actuator sweeping its whole ctrlrange at 0.2 Hz — what the
    viewer's sliders do, and what exposes an unstable joint.
Reports steps/s, real-time factor (steps/s x timestep), contacts and solver iterations, and
MuJoCo's own instability warnings (bad QACC etc.). FAIL when a file is below --min-rt real time
in either run or MuJoCo warned. Writes reports/bench_mjcf.json; last line `M1.bench PASS|FAIL`.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parent.parent
DEFAULT = REPO / "drone_line_sim" / "mjcf" / "line.dev.xml"
SWEEP_HZ = 0.2


def run(m, d, secs: float, moving: bool) -> dict:
    mujoco.mj_resetDataKeyframe(m, d, 0) if m.nkey else mujoco.mj_resetData(m, d)
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    n, ncon, iters = 0, [], []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < secs:
        if moving:
            d.ctrl[:] = lo + (hi - lo) * (0.5 + 0.5 * math.sin(2.0 * math.pi * SWEEP_HZ * d.time))
        mujoco.mj_step(m, d)
        n += 1
        if n % 50 == 0:
            ncon.append(d.ncon)
            iters.append(int(d.solver_niter[0]))
    wall = time.perf_counter() - t0
    warnings = {mujoco.mjtWarning(i).name: int(d.warning[i].number)
                for i in range(len(d.warning)) if d.warning[i].number}
    return {"steps": n, "wall_s": round(wall, 2), "steps_per_s": round(n / wall),
            "realtime": round(n * m.opt.timestep / wall, 2),
            "ncon_mean": round(float(np.mean(ncon)), 1) if ncon else 0.0,
            "ncon_max": max(ncon, default=0),
            "solver_iters_mean": round(float(np.mean(iters)), 1) if iters else 0.0,
            "solver_iters_max": max(iters, default=0), "mujoco_warnings": warnings}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Headless MJCF physics speed benchmark")
    ap.add_argument("xml", nargs="*", type=Path, default=[DEFAULT])
    ap.add_argument("--secs", type=float, default=10.0, help="wall seconds per run")
    ap.add_argument("--min-rt", type=float, default=1.0, help="minimum real-time factor")
    args = ap.parse_args(argv)
    doc = {"mujoco": mujoco.__version__, "secs": args.secs, "min_realtime": args.min_rt,
           "files": {}}
    ok = True
    parts = []
    for xml in args.xml:
        m = mujoco.MjModel.from_xml_path(str(xml))
        d = mujoco.MjData(m)
        o = m.opt
        row = {"nbody": m.nbody, "ngeom": m.ngeom, "nv": m.nv, "nu": m.nu,
               "timestep": o.timestep, "integrator": mujoco.mjtIntegrator(o.integrator).name,
               "solver": mujoco.mjtSolver(o.solver).name, "iterations": o.iterations,
               "ls_iterations": o.ls_iterations, "tolerance": o.tolerance,
               "contact": not (o.disableflags & mujoco.mjtDisableBit.mjDSBL_CONTACT)}
        row["rest"] = run(m, d, args.secs, False)
        row["moving"] = run(m, d, args.secs, True)
        good = all(row[k]["realtime"] >= args.min_rt and not row[k]["mujoco_warnings"]
                   for k in ("rest", "moving"))
        ok &= good
        doc["files"][str(xml)] = row
        for k in ("rest", "moving"):
            r = row[k]
            print(f"bench_mjcf: {xml.name} {k}: {r['steps_per_s']} steps/s = "
                  f"{r['realtime'] * 100:.0f}% real time (timestep {o.timestep}, "
                  f"{row['integrator']}, ncon max {r['ncon_max']}, solver iters max "
                  f"{r['solver_iters_max']}, warnings {r['mujoco_warnings'] or 'none'})")
        parts.append(f"{xml.name} {row['rest']['realtime'] * 100:.0f}%/"
                     f"{row['moving']['realtime'] * 100:.0f}% (rest/moving)")
    doc["passed"] = ok
    out = REPO / "reports" / "bench_mjcf.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, indent=2) + "\n")
    print(f"M1.bench {'PASS' if ok else 'FAIL'} real time, no rendering: " + "; ".join(parts))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
