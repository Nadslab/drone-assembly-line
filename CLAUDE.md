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
- `docs/` — plan, naming conventions, decisions log.
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

# Run tools tests
tools/.venv/bin/python -m pytest tools/tests/

# Headless checks (per phase, each writes reports/<name>.json)
tools/checks/spike.sh
tools/checks/world_smoke.sh
```

## Naming conventions

One term per concept. Name a new thing by finding its row below, not by inventing a word.

| Concept | Term | ID form (yaml, topics) | Node/link/model form |
|---|---|---|---|
| The whole main line | **line** | `main` | `Line` |
| A work location on the main line | **station** | `st_place_spider` | `Station_PlaceSpider` |
| Parallel copies of a station | **cell** | `st_test.cell_3` | `Station_Test_Cell3` |
| Carrier riding the conveyor | **pallet** | `pallet` | `Pallet_<n>` |
| Returnable fixture carrying a subassembly | **carrier** | `carrier_spider`, `carrier_pod` | `Carrier_Spider` |
| Part source at a fixed pick pose | **feeder** | `fd_mid_plate` | `Feeder_MidPlate` |
| WIP between a sub-line and the main line | **buffer** | `buf_spider` | `Buffer_Spider` |
| Pick-and-place robot | **gantry** | `gantry` | `Gantry` |
| Screwdriving unit (fixed pattern) | **gang head** | `gh_arm_sandwich` | `GangHead_ArmSandwich` |
| Screwdriving unit (variable position) | **scara** | `scara_1` | `Scara_1` |
| Interchangeable gantry tooling | **tool** | `tool_plate` | `Tool_Plate` |
| Named frame on a part used for grasp | **pick frame** | `pick` | `<Part>_pick` |
| Named datum on a plate/pallet | **datum** | `datum_round`, `datum_diamond` | `<Part>_datum_round` |
| An operation the sequencer can request | **capability** | `place`, `drive`, `advance`, `test` | — |

Rules:
- Station IDs use the **verb** of what happens there (`st_drive_arm_sandwich`), not the equipment.
- yaml/topic IDs are `snake_case`; Gazebo model/link names are `PascalCase` derived mechanically.
- "Arm" means the drone arm only. Robots are "gantry", "gang head", "scara". Never "robot arm".

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
- **Headless only.** No GUI launches during verification.
- **No STL over 5 MB is ever committed.**
