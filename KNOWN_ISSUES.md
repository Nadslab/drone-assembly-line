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
- **Overlaps inside one assembly are not compared.** One SCARA's convex proxies (pedestal,
  housing bbox, link capsules, shaft) overlap at its joints (link2 ∩ shaft ~99 mm deep), and a
  carrier's per-solid collision boxes nest (CarrierSpider_2_2 ∩ _3, 24 mm); the vendor/carrier
  STEP solids layout.py intersects do not. `check_mjcf_match.py` leaves pairs within one SCARA or
  one carrier out of the overlap comparison on both sides (listed in the report) and the MJCF
  excludes contact between one SCARA's bodies. Arm-vs-arm clashes between two SCARAs and
  arm-vs-everything-else are still compared.
- **The match check proves agreement, not a clean layout.** A real clash present in both the
  spec's layout and the MJCF passes (verified: Buffer_Spider ∩ Conveyor in a scratch spec);
  layout.py's interference check is still what fails a clashing layout.
- **mj_collision cannot see static-vs-static pairs.** Every fixed body is welded to the world, so
  MuJoCo never collides two of them. The check uses mj_geomDistance on every bbox candidate
  (same narrowphase) and only requires mj_collision's contacts to be a subset.
- **Arm housing is a box.** The vendor base housing is its layout bbox (`Scara_<S>_Housing`); the
  pedestal keeps the layout name `Scara_<S>_Base` (layout.py gives both solids that name).
- **Actuator gains are PLACEHOLDER** (kp/kv in gen_mjcf.py), gravity is off: enough for viewer
  sliders, not a motion model.
- **Coplanar faces flicker in renders.** Each station footprint shares its side faces with the
  conveyor (an allowed overlap in layout.py), so renders z-fight there.
- **Full-line match check takes 34–41 s without renders (> 30 s budget), ~52 s with
  `--render`.** No check was cut. Most of it is layout.plan (~11 s, the vendor STEP) and the
  optimal bounding boxes and booleans of five vendor arms in six samples. The vendor STEP is
  loaded once per run, each parked arm is a transformed instance, shapes are cached by solid
  name + placement, and layout.interference runs per cross-assembly pair. Renders are off by
  default (`--render`). loader_a takes ~17 s.
- **Viewer frame rate on WSL2 (rendering only, MJCF unchanged).** Mesa's D3D12 layer (RTX 3060)
  costs ~1.5 ms per draw call, independent of resolution: ~56 ms per frame for loader_a, ~250 ms
  for the full line; the 4096 shadow map doubles it (llvmpipe is faster, 73 ms). A viewer loop
  that steps once per frame runs at ~5 % real time. `tools/view_mjcf.py` steps physics to the
  wall clock and turns shadows off in memory. Headless physics: full line 44x (rest) / 30x
  (every joint moving) real time (verify/m1_realtime.log).
- **D-016 refill columns are not in the MJCF.** Operator refill zones are not layout solids, so
  the refill_clearance FAIL at A/E has no MuJoCo counterpart yet.
