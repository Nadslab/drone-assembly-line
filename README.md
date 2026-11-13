# drone-line

Spec-driven drone assembly line: `drone_line_sim/config/line_spec.yaml` is the single source of
truth; everything else is generated from it (see CLAUDE.md and docs/decisions.md).

## MuJoCo (M1, D-017)

Run everything from the repo root with the tools venv (`tools/.venv`, mujoco 4.0.1). Headless
renders need `MUJOCO_GL=egl` (the check sets it).

**Regenerate the MJCF** — never edit `drone_line_sim/mjcf/*.xml`; edit the spec and re-run:

```bash
tools/.venv/bin/python tools/gen_mjcf.py --dev                     # -> drone_line_sim/mjcf/line.dev.xml
tools/.venv/bin/python tools/gen_mjcf.py --dev --stations st_load_pallet,st_build_frame --name loader_a
                                                                   # -> line.loader_a.dev.xml
tools/.venv/bin/python tools/gen_mjcf.py                           # base spec: exits 1 until its nulls are filled
tools/checks/mjcf_gate_312.sh drone_line_sim/mjcf/line.dev.xml     # must also load in MuJoCo 3.12 (ROS vendor)
```

**Run the match check** (MJCF vs spec poses, layout.py solids and its interference check;
writes `reports/mjcf_match[.<name>].dev.json` and `verify/m1_<name>_{iso,top}.png`):

```bash
tools/.venv/bin/python tools/check_mjcf_match.py --dev             # full line
tools/.venv/bin/python tools/check_mjcf_match.py --dev --stations st_load_pallet,st_build_frame --name loader_a
tools/.venv/bin/python tools/bench_mjcf.py                         # headless physics speed, >= 1x real time
```

The last line of each run is `M1.<step> PASS|FAIL <summary>`. Don't run it alongside layout.py
or pytest (RAM).

**Open the viewer** (look and move only; a window — not for headless verification):

```bash
tools/.venv/bin/python tools/view_mjcf.py                          # line.dev.xml, folded keyframe
tools/.venv/bin/python tools/view_mjcf.py drone_line_sim/mjcf/line.loader_a.dev.xml --key parked_scara_build_frame
tools/.venv/bin/python tools/view_mjcf.py --list                   # joints and keyframes, no window
```

The right panel's Control sliders move the joints (`scara_<station>_j1..j4`,
`gh_<head>_slide`); keyframes (`folded`, `parked_<scara>`) are under Simulation > Key. Shadows
are off by default (`--shadows` keeps them; they halve the frame rate on WSL's D3D12 layer). The
console prints the real-time factor every 5 s.
