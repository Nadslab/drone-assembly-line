"""Open a generated line MJCF in MuJoCo's viewer (M1): look and move only, no editing (M2).

    tools/.venv/bin/python tools/view_mjcf.py [XML] [--key folded] [--fast] [--speed 1.0]
    tools/.venv/bin/python tools/view_mjcf.py --list      # print joints/keyframes, no window

Default XML: drone_line_sim/mjcf/line.dev.xml (regenerate with tools/gen_mjcf.py --dev).
The equivalent of `python -m mujoco.viewer`, through launch_passive so this loop owns the clock:
physics is stepped to the wall clock — as many mj_step as the elapsed time needs between two
frames, frames capped at FRAME_HZ — not one step per frame. On this WSL2 box a frame costs
~40-250 ms through Mesa's D3D12 layer, so one step per frame ran at ~5 % real time
(verify/m1_realtime.log). Prints the realised real-time factor every PRINT_S seconds.

--fast: cheaper frames, the MJCF untouched — envelope, carrier-proxy and guard geom groups
(3, 4, 5) hidden, shadows and reflections off (viewer scene flags via user_scn). Every toggle
stays available in the viewer's Rendering panel. Full line, D3D12: ~430 ms -> ~140 ms per frame
(verify/m1_viewer_fast.log).

Moving things: the right panel's Control sliders drive the position actuators — one per joint,
named like it (scara_<station>_j1..j4, gh_<head>_slide). Keyframes (`folded`, `parked_<scara>`)
load from the left panel's Simulation > Key. Gravity is off and the gains are PLACEHOLDERs: this
is for looking at the layout and moving joints, not for physics.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

REPO = Path(__file__).resolve().parent.parent
DEFAULT = REPO / "drone_line_sim" / "mjcf" / "line.dev.xml"
FRAME_HZ = 60.0
PRINT_S = 5.0
FAST_HIDDEN_GROUPS = (3, 4, 5)   # envelopes, carrier collision proxies, guards (gen_mjcf.G_*)
MAX_LAG_S = 0.25        # behind by more than this (a stalled frame): drop the backlog, resync


def describe(m: mujoco.MjModel) -> None:
    kinds = {int(mujoco.mjtJoint.mjJNT_HINGE): "hinge (rad)", int(mujoco.mjtJoint.mjJNT_SLIDE): "slide (m)"}
    print(f"{m.njnt} joints / {m.nu} slider actuators:")
    for j in range(m.njnt):
        lo, hi = m.jnt_range[j]
        print(f"  {m.joint(j).name:34s} {kinds.get(int(m.jnt_type[j]), '?'):12s} [{lo:+.4f}, {hi:+.4f}]")
    print("keyframes:", ", ".join(m.key(k).name for k in range(m.nkey)) or "none")


def apply_fast(opt: mujoco.MjvOption, scn: mujoco.MjvScene) -> None:
    """--fast: what costs draw calls on WSL's D3D12 layer and is not needed to look at the line."""
    for group in FAST_HIDDEN_GROUPS:
        opt.geomgroup[group] = 0
    scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
    scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="View a generated line MJCF (no editing)")
    ap.add_argument("xml", nargs="?", type=Path, default=DEFAULT)
    ap.add_argument("--key", default="folded", help="keyframe to start from (default folded)")
    ap.add_argument("--fast", action="store_true",
                    help="hide envelope/proxy/guard groups, shadows and reflections off")
    ap.add_argument("--speed", type=float, default=1.0, help="sim seconds per wall second")
    ap.add_argument("--list", action="store_true", help="print joints and keyframes, no window")
    args = ap.parse_args(argv)
    if not args.xml.is_file():
        print(f"view_mjcf: {args.xml} not found — run tools/gen_mjcf.py --dev", file=sys.stderr)
        return 1
    m = mujoco.MjModel.from_xml_path(str(args.xml))
    d = mujoco.MjData(m)
    describe(m)
    try:
        mujoco.mj_resetDataKeyframe(m, d, m.key(args.key).id)
    except KeyError:
        print(f"view_mjcf: no keyframe {args.key!r}; starting from qpos0", file=sys.stderr)
    mujoco.mj_forward(m, d)
    if args.list:
        return 0

    with mujoco.viewer.launch_passive(m, d) as v:
        if args.fast:
            apply_fast(v.opt, v.user_scn)
            v.sync()
        wall0, sim0 = time.perf_counter(), d.time
        last_print, steps = wall0, 0
        while v.is_running():
            frame_start = time.perf_counter()
            target = sim0 + (frame_start - wall0) * args.speed
            if target - d.time > MAX_LAG_S * args.speed:      # a stall: do not try to catch up
                wall0, sim0 = frame_start, d.time
                target = d.time
            with v.lock():
                while d.time < target:
                    mujoco.mj_step(m, d)
                    steps += 1
            v.sync()
            now = time.perf_counter()
            if now - last_print >= PRINT_S:
                rt = steps * m.opt.timestep / (now - last_print)
                print(f"view_mjcf: {rt * 100:.0f}% real time ({steps / (now - last_print):.0f} "
                      "steps/s)", flush=True)
                last_print, steps = now, 0
            time.sleep(max(0.0, 1.0 / FRAME_HZ - (time.perf_counter() - frame_start)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
