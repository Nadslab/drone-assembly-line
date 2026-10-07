# CLAUDE.md

Guidance for Claude Code working in this repository.

## Environment

- **OS:** WSL2, Ubuntu 24.04
- **ROS 2:** Jazzy
- **Gazebo:** standalone gz-harmonic 8.x (not the ROS-bundled Gazebo)
- **Python tooling venv:** `tools/.venv` — used for everything under `tools/` (spec loading,
  checks, generators, build123d, mesh pipeline). **Never `pip install` into the system Python.**
  Always invoke tools as `tools/.venv/bin/python tools/<script>.py`, or activate the venv first.
- `tools/` has a `COLCON_IGNORE` file — colcon never builds it.

## Repository layout

- `drone_line_sim/` — ROS 2 `ament_cmake` package: spec, descriptions, generated worlds/models,
  meshes, launch files. Holds everything Gazebo/xacro need at runtime.
- `drone_line_control/` — ROS 2 `ament_python` package: sequencer, actions, feeders, sources,
  monitors.
- `tools/` — plain Python (not built by colcon): spec loading/validation, static checks,
  generators (world, models, controllers), mesh pipeline, build123d CAD tooling.
- `docs/` — plan, naming conventions (`naming.md`), decisions log.
- `cad_raw/` — SolidWorks exports (gitignored except a manifest).
- `reports/` — generated JSON/CSV check output (gitignored).
- `templates/` — Jinja templates for SDF generation.

## Build / test commands

```bash
# Build the two ROS packages
cd ~/ros2_ws && colcon build --packages-select drone_line_sim drone_line_control

# Confirm tools/ is not a colcon package
colcon list

# Run tools venv scripts (spec validation, checks, generators)
tools/.venv/bin/python tools/check_line.py
tools/.venv/bin/python tools/spec.py
tools/.venv/bin/python tools/check_line.py --dev   # merge line_spec.dev.yaml placeholders
tools/.venv/bin/python tools/check_line.py --dev --milestone frame   # one milestone only
tools/.venv/bin/python tools/check_line.py --dev --stations st_a,st_b --name batch1   # a batch
                                                   # (also estimate_moves.py, layout.py, choose_park.py)
tools/.venv/bin/python tools/estimate_moves.py --dev   # per-SCARA pick-and-place time vs takt
tools/.venv/bin/python tools/estimate_moves.py --dev --calibrate   # a_max from the standard cycle (v_max kept)
tools/.venv/bin/python tools/choose_park.py --dev --write [--stations ...]   # dev park poses -> line_spec.dev.yaml
tools/.venv/bin/python tools/gen_sw_params.py      # spec -> cad_params.txt (SolidWorks variables)
tools/.venv/bin/python tools/mesh_pipeline.py      # raw SolidWorks STL -> checked Gazebo meshes
tools/.venv/bin/python tools/cad/pallet.py --dev   # build123d pallet + pins -> cad_out/
tools/.venv/bin/python tools/cad/build_all.py --dev  # every generator + mesh_pipeline, < 30 s
tools/.venv/bin/python tools/cad/layout.py --dev   # line layout STEP + top SVG + interference check
                                                   # -> C:\Users\Admin\Documents\drone-line-cad\60_layout\dev\
tools/.venv/bin/python tools/cad/layout.py --dev --milestone frame   # -> 60_layout\dev\frame\
                                                   # (also reports/render/<name>_{top,iso}.png and
                                                   # <station>_park.png; --no-render skips them)
tools/.venv/bin/python tools/cad/vendor.py <STEP> --model <id>  # joints + arm capsules -> reports/vendor_<id>.json
tools/.venv/bin/python tools/gen_models.py --dev   # spec + cad reports -> models/<id>/model.sdf
tools/.venv/bin/python tools/gen_world.py --dev    # spec -> worlds/main_line.sdf
tools/.venv/bin/python tools/gen_controllers.py --dev   # spec -> config/controllers.yaml
tools/.venv/bin/python tools/spec.py --dev --dump reports/line_spec.dev.yaml  # merged spec (xacro)

# Run tools tests
tools/.venv/bin/python -m pytest tools/tests/

# Headless checks (per phase, each writes reports/<name>.json)
tools/checks/spike.sh
tools/checks/world_smoke.sh
tools/checks/descriptions.sh    # xacro + check_urdf + joint limits vs spec (no Gazebo)
tools/checks/line_smoke.sh      # launch line.launch.py headless, controllers active, SCARAs at 3 stations
```

## Naming conventions

One term per concept. Name a new thing by finding its row below, not by inventing a word.

| Concept | Term | ID form (yaml, topics) | Node/link/model form |
|---|---|---|---|
| The whole main line | **line** | `main` | `Line` |
| A work location on the main line | **station** | `st_build_frame` | `Station_BuildFrame` |
| Parallel copies of a station | **cell** | `st_test.cell_3` | `Station_Test_Cell3` |
| Carrier riding the conveyor | **pallet** | `pallet` | `Pallet_<n>` |
| Queued pallet position between two stations | **queue slot** | count derived per gap at `layout.pallet_block.length + layout.slot_gap` (`spec.queue_gaps`) | `Pallet_Slot_<n>_<k>` (after station index n, k-th) |
| Returnable fixture carrying a subassembly | **carrier** | `carrier_spider`, `carrier_pod` | `Carrier_Spider` |
| Chute or conveyor taking an empty carrier back to its sub-line | **return path** | `rt_<carrier>` (`rt_carrier_spider`) | `ReturnPath_CarrierSpider`, its end `ReturnSink_CarrierSpider` |
| Part source at a fixed pick pose | **feeder** | `fd_mid_plate` | `Feeder_MidPlate` |
| WIP between a sub-line and the main line | **buffer** | `buf_spider` | `Buffer_Spider` |
| Pick-and-place robot, one per station | **scara** | `scara_<station without st_>` (`scara_build_frame`) | `Scara_<Name>` (`Scara_BuildFrame`); layout `Scara_<Name>_Base` (pedestal), `Scara_<Name>_<Part>` (vendor arm), `Scara_<Name>_Envelope` |
| SCARA product used by a scara | **robot model** | `robot_models.<id>` (`epson_ls10b_700_z300`) | — |
| Run of consecutive stations built and checked together | **milestone** | `milestones.<name>` (`frame`) | partition `Guard_Partition_<A>_<B>` at its boundary; the line's one fence `Guard_Fence` |
| Ad-hoc run of consecutive stations picked on the command line (not in the spec) | **batch** | `--stations st_a,st_b --name batch1` | layout `60_layout/dev/batch/`, reports `<tool>.<name>[.dev].json` |
| Screwdriving unit (fixed pattern) | **gang head** | `gh_arm_sandwich` | `GangHead_ArmSandwich`; on a slide, `GangHead_<Name>_Slide` (envelope) |
| Rail and post carrying a gang head on its slide | **slide frame** | `gang_heads.<id>.slide.frame` | `GangHead_<Name>_Rail`, `GangHead_<Name>_Post` |
| Where a SCARA waits while its station's gang head works | **park pose** | `scaras.<id>.park_pose` | close-up `reports/render/<station>_park.png` |
| Vendor's published reference move the motion is calibrated to | **standard cycle** | `robot_models.<id>.standard_cycle` | — |
| Interchangeable SCARA tooling | **tool** | `tool_plate_vacuum` (`tools:`) | `Tool_PlateVacuum` |
| SCARA end-effector carrying two tools 180° apart, switched by a J4 turn | **dual-tool** | `scaras.<id>.tools: [<tool>, <tool>]` | — |
| SCARA tool taking a whole set in one pick | **gang gripper** | `<part>_gang` (`standoff_gang`) | `Tool_StandoffGang` |
| Named frame on a part used for grasp | **pick frame** | `pick` | `<Part>_pick` |
| Named datum on a plate/pallet | **datum** | `datum_round`, `datum_diamond` | `<Part>_datum_round` |
| Pallet pin locating one arm through its tooling hole | **arm pin** | `drone.arm_tooling_holes` | `Pin_Arm_<n>` |
| An operation the sequencer can request | **capability** | `place`, `drive`, `remove` (lift a carrier off onto its return path), `actuate` (a station device, e.g. arm pins), `advance`, `test` | — |

Rules:
- Station IDs use the **verb** of what happens there (`st_build_frame`, `st_seat_esc`), not the equipment.
- yaml/topic IDs are `snake_case`; Gazebo model/link names are `PascalCase` derived mechanically.
- "Arm" means the drone arm only. Robots are "scara" and "gang head". Never "robot arm" (a
  SCARA's own segments are "links").
- A SCARA's joints are `scara_<station>_j1..j4`, its links `Scara_<Name>_*` / `Scara_<Name>_Tcp`,
  its controller `scara_<station>_controller` (docs/naming.md).

## P0 rules (apply throughout the project)

- **Never hand-edit generated files.** Files under `worlds/`, `models/`, `config/controllers.yaml`
  are outputs — edit `config/line_spec.yaml` (or the relevant generator) and re-run the generator.
- **Every generator/check exits non-zero on failure** and writes its result to
  `reports/<name>.json`.
- **Dev overlay:** `config/line_spec.dev.yaml` holds `# PLACEHOLDER` values so work can proceed
  before CAD/AnyLogic numbers exist. The overlay never overrides a real value: a non-null value
  in `line_spec.yaml` always wins, and the overlay cannot add entities. **No deliverable number
  may come from it** (SolidWorks params, mesh scaling, reported durations, sizing). Every
  generator/check takes `--dev` (`spec.add_dev_argument`), loads via `load_spec(dev=True)`, and
  writes `reports/<name>.dev.json` listing its `placeholders`. Delete an overlay line when the
  real value lands in the base spec.
- **Poses are derived (D-013).** A station is placed by `anchor: {after, gap}`; everything it
  owns (SCARA base, gang head, feeders, buffers, return paths) by `offset: [dx, dy, dz, yaw]` in
  its frame. Never write a `pose` into the spec: `spec.resolve_anchors` computes it and every
  generator reads only that. The `anchors` check fails an absolute pose.
- **The layout STEP is generated.** `60_layout/line_layout.step` (and its SVG) come from
  `tools/cad/layout.py`; edits happen in the spec, never in SolidWorks. A change made in the
  SolidWorks file is lost on the next run and never reaches the checks. `layout.py` also writes
  `line_layout_physical.step` next to it — the same assembly without the `Reference` group
  (reach and slide envelopes, guard walls, the operator aisle, the floor); `--no-reference` drops
  `Reference` from `line_layout.step` too. Every check (interference, straddle, opacity) and
  `reports/layout.json` stay in the spec/line frame (Z up). Only the exported STEP is rotated:
  SolidWorks is Y-up, so the assembly is rotated -90° about X right before writing, mapping spec
  `(x, y, z)` to STEP `(x, z, -y)` so SolidWorks' Top view shows the plan view.
- **Headless only.** No GUI launches during verification.
- **Never run layout.py and pytest concurrently;** the full-line layout peaks around 1.7 GB on a
  7.8 GB machine.
- **No STL over 5 MB is ever committed.**
