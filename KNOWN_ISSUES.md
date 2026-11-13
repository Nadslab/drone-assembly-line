# Known issues

Open problems logged instead of blocking a run. Newest milestone first.

## M1 — MuJoCo port

- **MuJoCo version split.** `tools/.venv` runs mujoco 4.0.1; `ros-jazzy-mujoco-vendor` 0.1.0
  (what `mujoco_ros2_control` builds on) bundles MuJoCo 3.12.0. The generated MJCF uses only the
  3.12-compatible subset (no 4.0-only attributes; joint stiffness/damping left unset — 4.0 takes
  three values there) and `check_mjcf_match.py` gates `line.dev.xml` on loading in 3.12.0 too.
  4.0 also rejects the same asset included twice; the exporter defines each mesh once.
- **mujoco is not pinned in a requirements file.** `tools/` has none; `tools/.venv` holds
  `mujoco==4.0.1` by hand (M1.1).
- **Offscreen GL: EGL only.** `MUJOCO_GL=egl` renders on this WSL2 box; OSMesa is not installed
  (`libOSMesa` missing). Renders set `MUJOCO_GL=egl`.
- **Layout-named bodies are not stable IDs.** Bodies with a spec ID (`st_*`, `scara_*`, `fd_*`,
  `buf_*`, `rt_*`, `gh_*`) keep it; everything else (`Conveyor`, `Pallet_Slot_<n>_<k>`,
  `Guard_Fence`, `Floor`, ...) is named after the layout solid and can renumber when the spec
  changes. In M2 only spec-ID bodies will be editable.
- **Base spec cannot be exported yet.** `line_spec.yaml` still has nulls (station gaps, offsets,
  ...); `gen_mjcf.py` without `--dev` exits non-zero and lists them. Only `line.dev.xml` exists.
- **refill_clearance FAIL at st_build_frame and st_fit_top_plate** (`check_line --dev`): the
  known D-016 item — no park pose clears fd_mid_plate / fd_top_plate. Not fixed in M1.
- **No timed motion.** No choreography YAML exists; the match check samples each SCARA in two
  static poses (folded, as layout.py draws it, and parked at `park_pose`).
