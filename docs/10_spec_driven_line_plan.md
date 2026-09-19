# 10 — Spec-Driven Line Plan (Gazebo + CAD + generation)

**Status:** plan, not started
**Date:** 2026-09-19
**Supersedes:** the four-step Gazebo sub-step list (world / feeders / robot descriptions / ros2_control)
and the CAD Phase 0–E list. Both are folded in below. Nothing from them is lost; the difference is
that most hand-authored files are now **generated from one spec**.
**Scope:** main line (physical target) simulated in full; sub-lines A/B/C simulated as sources.
**Takt:** 20.2 s effective (1,000/shift, 70% utilization).

---

## 0. Why this plan exists

OpenSDL (fl-sean03) builds its entire animated lab cell from one Python script: geometry from
primitives, motion from an ordered beat table, and ~107 automated checks that refuse to export a
broken scene. No CAD. The lesson for this project is not "use Blender"; it is:

1. **One source of truth.** Every coordinate, part, feeder and step lives in one file.
2. **Generate, don't hand-edit.** The Gazebo world, robot parameters, tooling geometry, reports and
   renders are *outputs* of that file.
3. **Checks are a build gate.** A generator that produces an invalid line exits non-zero.
4. **Semantic verbs, not device commands.** The sequencer asks for `place`, `drive`, `test`, never
   "move joint 3".
5. **Fixed vocabulary.** One word per concept, enforced by a naming table.

What this buys at scale: duplicating a bottleneck station is `count: 2`, re-balancing is editing a
duration, a new RF variant is a new part row, and each of those regenerates the world and re-runs
every check in under 30 s headless.

What OpenSDL does **not** give you, and why Gazebo stays: its scene is keyframed, with no
controllers, forces or real cycle times ("does not test forces"; "does not reproduce device cycle
times"). Gazebo + ros2_control remains the place where gantry motion, grasping and cycle time are
validated. AnyLogic remains the line-level DES reference.

### Is CAD still needed? Yes, but less of it

| Item | Before | Now |
|---|---|---|
| Drone part mods (plates, arms): tooling holes, lead slots, press-nut holes | SolidWorks | **SolidWorks** (stock carbon geometry, hand-modified). Unchanged. |
| Stack assembly check (A5) | SolidWorks | **SolidWorks**, now also checks against generated tooling STEP |
| Pallet, spider carrier, magazines, presenters, nests, end-effectors | SolidWorks | **build123d (Python)** → STEP (for SolidWorks/fabrication) + STL (for Gazebo) |
| `line_layout.SLDASM` (Phase D) | SolidWorks | **Deleted.** Replaced by spec + reach check + generated world |
| `station_poses.yaml` | Exported from SolidWorks | **Merged into `line_spec.yaml`** (hand-edited, source of truth) |
| Gantry / driver robot descriptions | Hand xacro | xacro that **reads the spec** for limits and dimensions |
| Static frame, conveyors, enclosures | SDF primitives | SDF primitives, **generated** |

Rule: **anything that will be fabricated and depends on line dimensions is code. Anything that is a
purchased or stock product part stays in SolidWorks.**

---

## 1. Repository layout

New repo, next to the old one in the same workspace. The old SO-101 repo stays untouched as
reference; reusable pieces are *copied*, not moved.

```
~/ros2_ws/src/
├── drone-assembly-line/            # OLD repo (SO-101 era). Do not modify. Reference only.
└── drone-line/                     # NEW repo
    ├── CLAUDE.md                   # commands, naming table, rules for Claude Code
    ├── README.md
    ├── docs/
    │   ├── 10_spec_driven_line_plan.md   # this file
    │   ├── naming.md
    │   └── decisions.md                  # D-0XX entries
    ├── drone_line_sim/             # ROS 2 ament_cmake package: data + description
    │   ├── package.xml / CMakeLists.txt
    │   ├── config/
    │   │   ├── line_spec.yaml      # ★ THE SOURCE OF TRUTH
    │   │   └── controllers.yaml    # generated
    │   ├── urdf/                   # gantry.urdf.xacro, driver.urdf.xacro (read line_spec.yaml)
    │   ├── models/                 # generated SDF models (pallet, carrier, magazines…)
    │   ├── meshes/                 # processed STLs only (decimated, < 5 MB each)
    │   ├── worlds/                 # generated main_line.sdf
    │   └── launch/
    ├── drone_line_control/         # ROS 2 ament_python package: runtime
    │   └── drone_line_control/
    │       ├── sequencer.py        # state machine over spec.sequence
    │       ├── actions/            # place, grasp, release, drive, advance, test
    │       ├── feeders.py          # unified feeder node
    │       ├── sources.py          # sub-line A/B/C sources
    │       └── monitors.py         # runtime invariants (ownership, carry, contact)
    ├── tools/                      # plain Python, NOT built by colcon
    │   ├── COLCON_IGNORE
    │   ├── spec.py                 # load + schema-validate line_spec.yaml (pydantic)
    │   ├── check_line.py           # static checks → reports/line_check.json
    │   ├── gen_world.py            # Jinja → worlds/main_line.sdf
    │   ├── gen_models.py           # SDF models with mass/inertia from spec
    │   ├── gen_controllers.py      # controllers.yaml from spec axes
    │   ├── mesh_pipeline.py        # raw STL → scaled/decimated/validated STL
    │   ├── cad/                    # build123d tooling generators
    │   └── render/                 # optional Blender replay (Phase 11)
    ├── cad_raw/                    # SolidWorks exports (gitignored except a manifest)
    ├── reports/                    # generated JSON/CSV (gitignored)
    ├── templates/                  # Jinja templates for SDF
    └── .github/workflows/ci.yml
```

Why two ROS packages: `drone_line_sim` holds everything Gazebo and xacro need at runtime (so
`$(find drone_line_sim)` resolves the spec), `drone_line_control` holds nodes. `tools/` has
`COLCON_IGNORE` so colcon never tries to build it.

**Generated files are committed.** CI regenerates them and fails if the committed copy differs
("drift check"), the same pattern OpenSDL uses for its schemas. This keeps `git diff` meaningful
and means a clone runs without first running generators.

### What to copy from the old repo

| From old `drone_assembly_cell` | To | Note |
|---|---|---|
| Processed plate/arm STLs | `drone_line_sim/meshes/drone/` | Re-run through `mesh_pipeline.py` |
| `set_pose` conveyor logic | `drone_line_control/actions/advance.py` | Becomes the `advance` capability |
| Screw spawner | `drone_line_control/feeders.py` | Becomes one feeder type |
| Launch patterns (standalone gz-harmonic + ros_gz bridge) | `drone_line_sim/launch/` | Keep the WSL2/D3D12 env settings |
| SO-101 URDF, 5-arm world, arm controllers | **nothing** | Superseded by gantry + drivers |

---

## 2. Naming conventions (put in `docs/naming.md` and `CLAUDE.md`)

One term per concept. A new thing is named by finding its row, not by inventing a word.

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

---

## 3. The spec: `drone_line_sim/config/line_spec.yaml`

Units: **metres, kilograms, seconds, radians**. Origin: pallet datum (round pin) at station 1 index
position, Z up. Every coordinate in the project is expressed in this frame.

Values marked `null` are **not yet known** and must be filled from CAD, AnyLogic or line balancing.
`check_line.py` reports every `null` as `UNRESOLVED`, so the report doubles as a to-do list.

```yaml
schema_version: 1
line:
  takt_s: 20.2
  units: {length: m, mass: kg, time: s, angle: rad}
  origin: pallet_datum_round

# ---------- parts: everything that moves or is gripped ----------
parts:
  bottom_plate:
    source: solidworks            # solidworks | build123d | primitive
    mesh: meshes/drone/bottom_plate.stl
    envelope: [null, null, null]  # x,y,z m — filled by mesh_pipeline.py, checked vs STL
    mass: null
    datums:                       # plate frame; from SolidWorks tooling-hole positions (§7.2)
      round:   [0.0, 0.0]
      diamond: [null, null]
    pick: [0.0, 0.0, null, 0.0]   # x,y,z,yaw in part frame
  mid_plate:     {source: solidworks, mesh: meshes/drone/mid_plate.stl, envelope: [null,null,null], mass: null, pick: [0,0,null,0]}
  top_plate:     {source: solidworks, mesh: meshes/drone/top_plate.stl, envelope: [null,null,null], mass: null, pick: [0,0,null,0]}
  side_plate:    {source: solidworks, mesh: meshes/drone/side_plate.stl, envelope: [null,null,null], mass: null, pick: [0,0,null,0]}
  standoff:      {source: primitive, shape: cylinder, dims: [null, null], mass: null}
  spider:        {source: primitive, shape: box, dims: [null,null,null], mass: null, carried_by: carrier_spider}
  pod:           {source: primitive, shape: box, dims: [null,null,null], mass: null, carried_by: carrier_pod}
  carrier_spider: {source: build123d, generator: tools/cad/spider_carrier.py, mass: null, pick: [0,0,null,0]}
  carrier_pod:    {source: build123d, generator: tools/cad/pod_tray.py, mass: null, pick: [0,0,null,0]}
  pallet:         {source: build123d, generator: tools/cad/pallet.py, mass: null}

# ---------- fasteners: one feeder per length (§7.9) ----------
fasteners:
  m3_x8:  {length: 0.008, feeder: fd_screw_m3x8}
  m3_x12: {length: 0.012, feeder: fd_screw_m3x12}

# ---------- feeders: fixed pick pose, the gantry never searches ----------
feeders:
  fd_mid_plate:  {type: magazine,  part: mid_plate,  pose: [null,null,null,0], capacity: 20, refill_s: null}
  fd_top_plate:  {type: magazine,  part: top_plate,  pose: [null,null,null,0], capacity: 20, refill_s: null}
  fd_side_plate: {type: magazine,  part: side_plate, pose: [null,null,null,0], capacity: 40, refill_s: null}
  fd_standoff:   {type: step,      part: standoff,   pose: [null,null,null,0], capacity: 200, refill_s: null}
  fd_screw_m3x8: {type: blow,      part: m3_x8,      target: gh_arm_sandwich, feed_s: null}

# ---------- sub-line sources (simulated) ----------
sources:
  src_A: {output: spider,       buffer: buf_spider, interval_s: 20.2, buffer_capacity: null}
  src_B: {output: pod,          buffer: buf_pod,    interval_s: 20.2, buffer_capacity: null}
  src_C: {output: bottom_plate, buffer: buf_plate,  interval_s: 20.2, buffer_capacity: null}
buffers:
  buf_spider: {pose: [null,null,null,0]}
  buf_pod:    {pose: [null,null,null,0]}
  buf_plate:  {pose: [null,null,null,0]}

# ---------- equipment ----------
gantry:
  axes:                                     # from the axis spec; limits drive xacro + reach check
    x: {min: null, max: null, v_max: null, a_max: null}
    y: {min: null, max: null, v_max: null, a_max: null}
    z: {min: null, max: null, v_max: null, a_max: null}
    c: {min: -3.1416, max: 3.1416, v_max: null, a_max: null}   # delete if no yaw needed
  tools: [tool_plate, tool_carrier]
  mount: [null, null, null]                 # gantry base in line frame
gang_heads:
  gh_arm_sandwich: {direction: down,       spindles: 8, pattern_from: bottom_plate.arm_holes, stroke: null}
  gh_side_plates:  {direction: horizontal, spindles: null, opposed: true, stroke: null}
scaras: {}                                  # add only where screw position varies

# ---------- stations ----------
conveyor: {pitch: null, index_s: null}      # pallet advance distance + time
stations:
  st_load_pallet:          {index: 1,  count: 1, pose: [null,null,null,0]}
  st_place_spider:         {index: 2,  count: 1, pose: [null,null,null,0]}
  st_place_mid_plate:      {index: 3,  count: 1, pose: [null,null,null,0]}
  st_drive_arm_sandwich:   {index: 4,  count: 1, pose: [null,null,null,0]}
  st_install_standoffs:    {index: 5,  count: 1, pose: [null,null,null,0]}
  st_seat_esc:             {index: 6,  count: 1, pose: [null,null,null,0]}
  st_place_pod:            {index: 7,  count: 1, pose: [null,null,null,0]}
  st_fit_side_plates:      {index: 8,  count: 1, pose: [null,null,null,0]}
  st_mount_servo:          {index: 9,  count: 1, pose: [null,null,null,0]}   # parked (§15.3)
  st_fit_top_plate:        {index: 10, count: 1, pose: [null,null,null,0]}
  st_audit_torque:         {index: 11, count: 1, pose: [null,null,null,0]}
  st_test:                 {index: 12, count: 5, pose: [null,null,null,0], cell_pitch: null}
  st_pack:                 {index: 13, count: 1, pose: [null,null,null,0]}

# ---------- sequence: the beat table ----------
# Order = execution order within one pallet. `requires` = precedence graph (§13 next step 1).
# duration_s = work content at that station; from AnyLogic / line balancing.
sequence:
  - {id: load_pallet,       station: st_load_pallet,        capability: load,    part: bottom_plate, from: buf_plate,   duration_s: null, requires: []}
  - {id: place_spider,      station: st_place_spider,       capability: place,   part: carrier_spider, from: buf_spider, tool: tool_carrier, duration_s: null, requires: [load_pallet]}
  - {id: place_mid_plate,   station: st_place_mid_plate,    capability: place,   part: mid_plate, from: fd_mid_plate, tool: tool_plate, duration_s: null, requires: [place_spider]}
  - {id: drive_arm_sandwich, station: st_drive_arm_sandwich, capability: drive,  head: gh_arm_sandwich, fastener: m3_x8, qty: 8, duration_s: null, requires: [place_mid_plate]}
  - {id: install_standoffs, station: st_install_standoffs,  capability: place,   part: standoff, from: fd_standoff, qty: 4, duration_s: null, requires: [drive_arm_sandwich]}
  - {id: seat_esc,          station: st_seat_esc,           capability: drive,   qty: 4, duration_s: null, requires: [install_standoffs], note: "releases carrier_spider"}
  - {id: place_pod,         station: st_place_pod,          capability: place,   part: carrier_pod, from: buf_pod, duration_s: null, requires: [seat_esc]}
  - {id: fit_side_plates,   station: st_fit_side_plates,    capability: drive,   head: gh_side_plates, qty: null, duration_s: null, requires: [place_pod]}
  - {id: mount_servo,       station: st_mount_servo,        capability: drive,   qty: 2, duration_s: null, requires: [fit_side_plates], status: parked}
  - {id: fit_top_plate,     station: st_fit_top_plate,      capability: drive,   part: top_plate, from: fd_top_plate, qty: 5, duration_s: null, requires: [fit_side_plates]}
  - {id: audit_torque,      station: st_audit_torque,       capability: inspect, duration_s: null, requires: [fit_top_plate]}
  - {id: test,              station: st_test,               capability: test,    duration_s: 90,  requires: [audit_torque]}
  - {id: pack,              station: st_pack,               capability: pack,    duration_s: null, requires: [test]}
```

Notes on the example:
- Step order and content are copied from the project summary §5 main line. Durations are `null`
  except the ~90 s test figure already in §3. Fill the rest from AnyLogic, not by guessing.
- Step 7's 8-pin cable connection is inside `place_pod`; decide whether it is automated or an
  operator task before assigning an executor.
- The spec is validated by a **pydantic model** in `tools/spec.py`, so a typo (`duraton_s`) fails
  immediately instead of silently becoming `null`.

---

## 4. Phase order and critical path

```
P0 repo+conventions ─► P1 one-axis ros2_control spike ─► P5 gantry in Gazebo ─► P7 grasp ─► P8 sequencer ─► P9 run checks
        │                                                      ▲                    ▲
        └─► P2 spec+static checks ─► P5 world gen ─────────────┘                    │
        └─► P3 SolidWorks mods ─► P3 mesh pipeline ─► P4 build123d tooling ─────────┘
                                                                 P6 feeders ────────┘
P10 CI + skills: start in P0, grow every phase.     P11 Blender replay: after P8, optional.
```

P1 is first on the ROS side because the controller_manager/oscillation bug is still open and every
robot phase depends on it. P2 and P3 run in parallel with P1 because they need no ROS.

Each phase ends with a **headless check that finishes in ≤ 30 s and writes a loggable result**.

---

## P0 — Repo and conventions (½ day)

1. Create the repo and packages (commands in §Appendix A).
2. Write `CLAUDE.md`: environment block, build/test commands, the naming table, and these rules:
   - Never hand-edit files under `worlds/`, `models/`, `config/controllers.yaml`: edit the spec and
     run the generator.
   - Every generator/check exits non-zero on failure and writes `reports/<name>.json`.
   - Headless only; no GUI launches during verification.
   - STLs over 5 MB are never committed.
3. `docs/decisions.md`: add **D-0XX — Spec-driven generation, new repo.** Context: gantry+SCARA
   architecture replaces 5×SO-101; scaling requires one source of truth. Decision: `line_spec.yaml`
   drives world, descriptions, tooling and checks; old repo frozen.
4. `.gitignore`: `cad_raw/**/*.SLD*`, `cad_raw/**/*.stl`, `reports/`, `build/`, `install/`, `log/`,
   `tools/.venv/`, `*.blend1`.

*Check:* `colcon build --packages-select drone_line_sim drone_line_control` succeeds;
`colcon list` does not list `tools`.

---

## P1 — One-axis ros2_control spike (blocking bug)

Isolate the controller_manager / oscillation problem on the simplest possible robot before
anything else depends on it.

1. `urdf/spike_axis.urdf.xacro`: world → fixed base → one prismatic link (a cart on a rail).
   Real mass and inertia on every link; `<dynamics damping="…" friction="…"/>` on the joint.
2. `<ros2_control>` block: `gz_ros2_control/GazeboSimSystem`, position command interface,
   position + velocity state interfaces.
3. Gazebo plugin in the description: `gz_ros2_control-system` /
   `gz_ros2_control::GazeboSimROS2ControlPlugin`, pointing at a controllers yaml with
   `joint_state_broadcaster` + `joint_trajectory_controller`. This plugin is what starts the
   controller manager; if it is missing or its parameters path is wrong, the controller manager
   never appears (the likely cause of the open bug).
4. Copy the structure of the gz_ros2_control cart-on-rail demo rather than the SO-101 files.
5. Try the position interface first. If it oscillates, check in this order: inertia realism →
   joint damping → controller gains → physics step size.

*Check (`tools/checks/spike.sh`, ≤ 30 s headless):* `ros2 control list_controllers` shows both
active within 30 s; a 0.5 m trajectory settles within tolerance with no overshoot oscillation;
result written to `reports/spike.json`.

Record the working damping/friction/gain values in the spec as defaults for gantry axes.

---

## P2 — Spec and static checks (no ROS, no Gazebo)

1. **`tools/spec.py`** — pydantic models for every spec section; `load_spec()` returns typed
   objects; unknown keys are errors.
2. **`tools/check_line.py`** — runs every static check, prints a table, writes
   `reports/line_check.json` with `passed: true|false`, exits non-zero on any FAIL.

| Check | Rule | Source of rule |
|---|---|---|
| `schema` | Spec parses; all references resolve (station, feeder, part, head, tool ids exist) | — |
| `unresolved` | List every `null` (WARN, not FAIL, until the phase that needs it) | — |
| `precedence` | `requires` forms a DAG; every step's requirements appear earlier in the sequence | summary §13.1 |
| `takt` | For each station: Σ durations ÷ `count` ≤ `takt_s` (the 5-cell test bank passes at 90/5 = 18 s) | summary §3 |
| `line_balance` | Report per-station utilization %, flag the bottleneck | §13.7 |
| `reach` | Every pick/place pose (feeder, buffer, station) is inside gantry axis limits; every drive pose inside its head/SCARA envelope | gantry chat §1.5 |
| `feeder_rate` | For each feeder: consumption per takt ≤ capacity ÷ refill_s | gantry chat §2.5 |
| `buffer` | Source interval ≤ takt; buffer capacity ≥ configured minimum | — |
| `fastener_lengths` | Count distinct fastener lengths (each is a feeder); WARN above a threshold | summary §7.9 |
| `no_flip` | No step requires fastening from below | summary §4 |
| `datums` | Every plate defines `round` and `diamond`; same XY across bottom/mid/top plates | summary §7.2 |

3. **Unit tests** (`tools/tests/`, pytest): each check has a passing and a deliberately failing
   fixture spec.

*Check:* `python tools/check_line.py` in < 5 s; the report lists every `UNRESOLVED` value.

---

## P3 — SolidWorks part modifications and the mesh pipeline

### P3.1 CAD conventions (unchanged from the CAD plan, now tied to the spec)

- **Datum scheme:** each plate has one round Ø3.0 H7 tooling hole (primary) and one diamond/slot
  (secondary), non-fastener, on the longest diagonal, same XY on bottom/mid/top.
- **Origin rule:** part origin at the round tooling hole centre, bottom face, Z up. This origin
  becomes the link/model frame in Gazebo.
- **Export:** mm, "do not translate STL output data to positive space", output coordinate system =
  that origin. Scale 0.001 happens in the SDF, not in SolidWorks.
- **Collision:** STL is visual only; collision is always primitives from the spec envelope.

### P3.2 Part modifications (SolidWorks, in priority order from summary §13.4)

| # | File | Change | Spec fields it fills |
|---|---|---|---|
| A1 | `bottom_plate_mod.SLDPRT` | Press-nut holes per chosen datasheet (edge distance!), tooling holes | `parts.bottom_plate.datums`, `…arm_holes` |
| A2 | `mid_plate_mod.SLDPRT` | Motor lead exit slots, tooling holes | `parts.mid_plate.datums` |
| A3 | `top_plate_mod.SLDPRT` | Tooling holes at same XY; countersinks | `parts.top_plate.datums` |
| A4 | `arm_mod.SLDPRT` | Asymmetric key, root tooling holes, pick flat if needed | `parts.arm.*` |
| A5 | `drone_stack_mod.SLDASM` | Mates to tooling holes; pins pass through all layers; **import generated pallet + carrier STEP** and run interference detection | — |

Hole coordinates that other parts depend on (tooling holes, arm-screw pattern, standoff positions)
are **typed into the spec by hand from SolidWorks measurements**, once, and from then on the
tooling generators read them from the spec. This is the one manual transfer in the pipeline; A5's
interference check against the generated tooling is what catches a transcription error.

### P3.3 `tools/mesh_pipeline.py`

Input: `cad_raw/*.stl` (mm). Output: `drone_line_sim/meshes/<group>/*.stl`.

1. Load (trimesh), confirm units by bounding box magnitude (mm-scale expected).
2. Decimate large meshes (pymeshlab `meshing_decimation_quadric_edge_collapse`) to a target face
   count; keep the file < 5 MB (lesson from the 180 MB motor STL).
3. Verify origin: the round-datum hole centre should sit at (0, 0) within tolerance.
4. Write the bounding box in metres back to `parts.<id>.envelope` in the spec **only if** the spec
   value is `null`; if it is set and differs by more than tolerance, FAIL (the CAD changed and the
   spec didn't, or vice versa).
5. Write `reports/meshes.json`: per mesh — faces before/after, size, bbox (m), origin error.

*Check:* `python tools/mesh_pipeline.py` < 30 s; no mesh > 5 MB; every bbox matches spec.

---

## P4 — Procedural tooling in build123d

Fabricated tooling is generated from spec numbers so it cannot drift from the plates, the pallet
pitch or the gantry grid. Output per part: **STEP** (open in SolidWorks for review, drawings,
fabrication) and **STL** (through `mesh_pipeline.py` to Gazebo).

Environment: separate venv (`tools/.venv`, `pip install build123d pymeshlab trimesh pydantic ruamel.yaml
pyyaml jinja2`), so CAD dependencies never touch the ROS Python.

Build order (each depends on the one before):

| # | Generator | Reads from spec | Key rule |
|---|---|---|---|
| B1 | `cad/pins.py` | datum dims, plate stack height | Round + diamond pin; length clears full stack + lead-in chamfer |
| B2 | `cad/pallet.py` | `parts.bottom_plate.datums`, envelope, conveyor pitch | Pins at datums, plate pocket, clearance under every screw site, conveyor-stop index feature |
| B3 | `cad/spider_carrier.py` ★ | `bottom_plate.arm_holes`, ESC height, gripper tool dims | Arm pockets **computed from the plate's arm-hole pattern**, never redrawn; gripper clearance; survives solder heat/IPA (material note in STEP metadata) |
| B4 | `cad/pod_tray.py` | pod dims, gantry tool | Same pattern as B3 |
| B5 | `cad/magazine.py` | part envelope, capacity, key geometry | One generator, parameterized per part (plates, side plates, arms) |
| B6 | `cad/screw_presenter.py` | fastener lengths, head pose | Position is what matters; geometry is simple |
| B7 | `cad/tools.py` | carrier/plate pick features | Gantry end-effectors (plate tool, carrier tool) |

Each generator also emits `reports/cad_<part>.json`: mass (from volume × material density, written
to the spec if `null`), bbox, and the derived feature positions (e.g. pocket centres). A pytest
asserts derived positions equal the spec source to 0.01 mm.

Then close the loop in SolidWorks: insert B2/B3 STEP into A5 and run Interference Detection.

*Check:* `python tools/cad/build_all.py` regenerates all STEP/STL in < 30 s; derived-position tests
pass.

---

## P5 — Gazebo generation

### P5.1 World: `tools/gen_world.py`

Jinja template `templates/main_line.sdf.j2` → `drone_line_sim/worlds/main_line.sdf`.

- Static geometry as primitives: frame envelope, gantry rail envelope, conveyor path, buffer
  shelves, test-cell enclosures (one per `st_test` cell, placed at `cell_pitch`).
- One `<include>` per feeder, buffer, station fixture, at spec poses.
- World plugins: Physics, SceneBroadcaster, UserCommands (for `set_pose`), Contact system (needed
  for contact sensors).
- Header comment: `GENERATED from line_spec.yaml by gen_world.py — do not edit`.

### P5.2 Models: `tools/gen_models.py`

For pallet, carriers, magazines, parts: `models/<id>/model.sdf` + `model.config`. Visual = STL
(scale 0.001), collision = primitive from envelope, **inertia computed** from mass and envelope
(box formula), pick frame and datum frames as named `<frame>` elements.

### P5.3 Robot descriptions (xacro reads the spec)

```xml
<xacro:property name="spec" value="${xacro.load_yaml('$(find drone_line_sim)/config/line_spec.yaml')}"/>
<xacro:property name="ax" value="${spec['gantry']['axes']}"/>
<!-- joint limits come from ax['x']['min'], ax['x']['max'], ax['x']['v_max'] … -->
```

- `urdf/gantry.urdf.xacro`: prismatic X, Y, Z (+ revolute C if kept), primitive links, TCP frame,
  real mass/inertia, damping/friction from P1 values. One `DetachableJoint` instance per tool/part
  type (P7).
- `urdf/gang_head.urdf.xacro` (macro): one prismatic stroke joint + N spindle frames laid out from
  `pattern_from`. Instantiated once per `gang_heads` entry.
- `urdf/scara.urdf.xacro`: only if `scaras` is non-empty.
- `tools/gen_controllers.py` → `config/controllers.yaml` with one `joint_trajectory_controller`
  over the gantry axes and one per head.

### P5.4 Launch

`launch/line.launch.py`: standalone gz-harmonic server (`-s -r`, headless for checks), ros_gz
bridge, robot_state_publisher per robot, spawn, controller spawners. Launch arg `gui:=false` default.

*Checks:*
- `check_urdf` clean on every expanded xacro.
- `tools/checks/world_smoke.sh`: world loads headless, runs N iterations, every model listed by
  `gz model --list` is at its spec pose (≤ 1 mm). Writes `reports/world.json`.

---

## P6 — Feeders and sub-line sources

1. **One feeder node** (`feeders.py`) instantiated per `feeders` entry. Interface per feeder:
   - `/<feeder_id>/available` (bool topic), `/<feeder_id>/count` (int topic)
   - `/<feeder_id>/take` (service): marks the part taken; after `refill_s` spawns the next part at
     the same pose (spawn orientation *is* the keying).
   The sequencer never knows the feeder type.
2. **Blow feeders**: screw appears at the head's bit after `feed_s`. No tube physics.
3. **Sources** (`sources.py`): spawn spider-in-carrier, pod-in-tray, bottom plate at buffer poses
   every `interval_s`; publish buffer count; same `available`/`take` interface as feeders.
4. **Starvation**: empty feeder → `available=false` → sequencer waits and logs `wait_starved`.

*Check:* drain a 20-part magazine headless; the refill delay appears as `wait_starved` time in the
event log; `reports/feeders.json`.

---

## P7 — Grasp and fastening

1. **Logical grasp with DetachableJoint** (gantry = parent, part = child):
   - One plugin instance per carried type, each with its own attach/detach/output topics.
   - Starts attached → send a setup detach at startup; use `suppress_child_warning` for parts that
     are spawned later.
   - Tree topology only; re-attach happens at the child's current pose, so `set_pose` it to the
     TCP first.
2. **Place onto pallet**: detach, then `set_pose` the part to the pallet datum pose (snap rule for
   round+diamond pins; no insertion physics).
3. **Drive**: gang head strokes down, dwell `drive_s`, each spindle's screw marked `driven`; screw
   count consumed from the blow feeder.
4. **Ownership model** (from OpenSDL's head-ownership check): every part instance has exactly one
   owner at every instant: `feeder | buffer | tool | pallet | carrier | packed`. Owner changes only
   through `take`, `grasp`, `release`, `place`. `monitors.py` publishes violations.

*Check:* attach plate → move X 0.3 m → plate follows within 1 mm → detach → plate stays; ownership
log shows `feeder → tool → pallet` with no gaps or doubles.

---

## P8 — Gantry control, capabilities and the sequencer

1. **Three-axis gantry** on the P1 pattern. Waypoint moves with trapezoidal profiles from
   `v_max`/`a_max`. Cartesian IK is trivial, so no MoveIt yet.
   *Check:* drive to every pose in the spec; TCP error < tolerance; log move time per pose pair to
   `reports/gantry_moves.csv` (this is the gantry cycle-time data).
2. **Capabilities as ROS 2 actions** — one per verb in the spec: `load`, `place`, `drive`,
   `advance`, `inspect`, `test`, `pack`. Each declares, in a small yaml next to it (OpenSDL
   pattern): inputs, timeout, whether a retry is safe, which resource it locks (`gantry`,
   `gh_arm_sandwich`, a test cell).
3. **Sequencer** (`sequencer.py`): reads `spec.sequence`, runs it as a state machine per pallet,
   respects `requires`, locks resources, runs `st_test` cells as a pool of `count` parallel
   servers. `use_sim_time` everywhere.
4. **Event log**: every capability start/end, wait and ownership change is appended to
   `reports/events_<run>.csv` (`sim_time, pallet, step, station, event, detail`). This log is the
   single input for P9 checks, AnyLogic comparison and P11 replay.
5. Keep the sequencer's boundary as actions/topics so CODESYS can take over sequencing later via
   OPC UA or Modbus without touching the capability servers.

*Check:* one drone through headless; per-step times from the event log vs spec durations and
AnyLogic; `reports/run_summary.json` with cycle time per station and takt pass/fail.

---

## P9 — Runtime checks (the OpenSDL build gate, applied to Gazebo runs)

`tools/check_run.py <events.csv>` → `reports/run_check.json`, non-zero on any FAIL.

| Check | Rule |
|---|---|
| `ownership` | Every part has exactly one owner at every event; no part disappears or duplicates |
| `carry_rigidity` | While owned by a tool, the part pose tracks the TCP within 1 mm (sampled from TF) |
| `contacts` | Every contact-sensor hit is in an allow-list (`allowed_contacts` in spec: pair + step window); anything else FAILs |
| `precedence_runtime` | No step started before its `requires` finished |
| `takt_runtime` | Measured station cycle ≤ takt; bottleneck named |
| `starvation` | Total `wait_starved` per feeder below threshold |
| `drift` | Measured step time vs spec `duration_s` within ±X%; large drift means the spec or the model is wrong |
| `throughput` | N pallets in steady state → drones/hour extrapolated vs 1,000/shift target |

*Check:* 10-pallet headless run + `check_run.py` completes in a bounded time and prints PASS/FAIL
per row.

---

## P10 — CI and Claude Code skills

### `.github/workflows/ci.yml` (on every push/PR)

1. **tools job** (ubuntu, Python only, fast): `pytest tools/tests`, `check_line.py`, run all
   generators and **fail if `git diff --exit-code` shows drift** in generated files.
2. **ros job** (container `ros:jazzy`): `colcon build`, `check_urdf` on expanded xacros.
3. **sim job** (optional, manual/weekly): Gazebo headless smoke + one-drone run + `check_run.py`.

### Claude Code skills (`.claude/skills/`)

| Skill | What it does |
|---|---|
| `add-station` | Adds a station + sequence rows to the spec, regenerates, runs `check_line.py`, reports takt impact |
| `rebalance` | Given new durations, updates spec, reruns `takt` + `line_balance`, proposes `count` changes |
| `add-variant-part` | Adds a part row (e.g. new RX/VTX in the pod envelope), checks the pod envelope, regenerates |
| `debug-run` | Reads the latest `events_*.csv` + `run_check.json`, names the first failing invariant and the step |

Each skill ends by running the checks and pasting the JSON summary.

---

## P11 — Blender replay (optional, presentation only)

For advisor reviews, C3 and outreach. Not a validation tool.

1. `tools/gen_world.py --export-json build/line_spec.json` (Blender's Python has no PyYAML).
2. `tools/render/build_scene.py` (run `blender -b -P`): builds geometry from the spec (primitives +
   the processed STLs), then **keyframes from a Gazebo event log**, not from hand-authored beats, so
   the film shows the actual simulated run.
3. Outputs: `.blend`, `.glb` (web viewer), still PNG, MP4 via ffmpeg.
4. Skip OpenSDL's camera-cut validators; a few fixed camera poses are enough.

OpenSDL is Apache-2.0; its `build_scene.py` helpers (`rounded_box`, `extrusion`, key-frame helpers)
can be reused with attribution.

---

## 12. Scaling recipes (what the setup is for)

| Change | What you edit | What happens automatically |
|---|---|---|
| Duplicate a bottleneck station | `stations.<id>.count: 2` | World regenerates with 2 cells; sequencer pools them; takt check re-runs |
| Re-balance the line | `duration_s` values, move a step to another `station` | Takt + balance report; runtime drift check against next run |
| Move a feeder | `feeders.<id>.pose` | Reach check; world regenerates; gantry move times re-measured on next run |
| New RF variant | Part row in sub-line B kit (pod envelope unchanged) | Envelope check; main line untouched (summary §9 rule) |
| Plate CAD revision | Re-export STL, update datum numbers | Mesh bbox check fails until spec matches; pallet + carrier regenerate from new datums |
| Physical sub-line later | Replace `sources.src_A` with real stations + sequence | Same checks apply |
| PLC takes over | CODESYS calls the same capability actions | Sequencer retired, checks unchanged |

---

## 13. Dropped from earlier plans, and why

- **Five SO-101 layout** — superseded by gantry + gang heads/SCARA; rebuilt from the spec, not
  patched.
- **`line_layout.SLDASM`** — its job (reach envelopes, collisions, poses) is done by `check_line.py`
  reach check + generated world + A5 interference check. Poses live in the spec, not in CAD.
- **SolidWorks-modeled tooling** — moved to build123d so it tracks the spec; SolidWorks still
  reviews it via STEP.
- **Hand-written world SDF and controller yaml** — generated.

## 14. Open items that block specific phases

| Blocks | Item | Summary ref |
|---|---|---|
| P2 takt/balance | Per-step durations from AnyLogic | §3, §13.7 |
| P2 reach, P5 gantry | Gantry axis spec (travel, v_max, a_max, yaw needed?) | — |
| P3/P4 | Press-nut selection → hole size and edge distance | §7.1 |
| P3 A1, P5 gang head | Standoff / arm-nut merge (changes pattern and screw count) | §15.1 |
| P4 B3 | Spider carrier material (solder heat + IPA) | §7.10 |
| `mount_servo` step | Servo mount geometry (parked) | §15.3 |
| `place_pod` executor | Who/what connects the 8-pin cable | §5 main line step 7 |

---

## Appendix A — Setup commands

```bash
# 1. New repo next to the old one
cd ~/ros2_ws/src
mkdir drone-line && cd drone-line
git init -b main

# 2. ROS packages
ros2 pkg create drone_line_sim --build-type ament_cmake
ros2 pkg create drone_line_control --build-type ament_python --dependencies rclpy
mkdir -p drone_line_sim/{config,urdf,models,meshes/drone,meshes/tooling,worlds,launch}
mkdir -p tools/{cad,render,checks,tests} templates docs cad_raw reports .claude/skills
touch tools/COLCON_IGNORE

# 3. Tools venv (kept separate from ROS Python)
python3 -m venv tools/.venv
tools/.venv/bin/pip install build123d pymeshlab trimesh pydantic pyyaml ruamel.yaml jinja2 pytest

# 4. Put this plan in docs/, then start Claude Code from the repo root
cp ~/Downloads/10_spec_driven_line_plan.md docs/   # adjust to where you saved it
claude

# 5. Build check
cd ~/ros2_ws && colcon build --packages-select drone_line_sim drone_line_control && colcon list
```

(In the `drone_line_sim/CMakeLists.txt`, add
`install(DIRECTORY config urdf models meshes worlds launch DESTINATION share/${PROJECT_NAME})`
so `$(find drone_line_sim)` resolves the spec.)

## Appendix B — Claude Code prompts (run from `~/ros2_ws/src/drone-line/`)

Model per prompt in brackets. Rule: **Sonnet** when the prompt fully specifies the output; **Opus**
when it has to diagnose, design state/concurrency, or reason about geometry. Run `/clear` between
prompts. Switch to Opus as soon as a failing check survives one fix attempt (see B-DEBUG).
Commit after every prompt whose check passes.

**P0 — CLAUDE.md** [Sonnet]
```
Read docs/10_spec_driven_line_plan.md §1–§2 and P0. Write CLAUDE.md: env (WSL2, Ubuntu 24.04,
ROS 2 Jazzy, standalone gz-harmonic, tools venv at tools/.venv), build/test commands, the naming
table, and the P0 rules. Add .gitignore per P0.4 and docs/decisions.md with D-0XX. No other files.
```

**P1 — spike** [Opus]
```
Implement P1 from docs/10_spec_driven_line_plan.md in drone_line_sim: spike_axis.urdf.xacro,
controllers yaml, launch/spike.launch.py (headless, gui:=false). Mirror the gz_ros2_control
cart-on-rail demo structure. Add tools/checks/spike.sh that launches, waits ≤30 s for active
controllers, sends a 0.5 m trajectory, measures overshoot/settling from /joint_states, writes
reports/spike.json, exits non-zero on failure, and kills gz on exit.
```
After it passes: write the working mass, damping, friction and gains into docs/decisions.md as a
D-entry, so P2 can use them as gantry-axis defaults.

**P2 — spec + static checks** [Sonnet]
```
Implement P2: create drone_line_sim/config/line_spec.yaml from the §3 example exactly (keep nulls).
Write tools/spec.py (pydantic, forbid extra keys) and tools/check_line.py with every check in the
P2 table; nulls are WARN "UNRESOLVED". Write reports/line_check.json with passed:true/false; exit
non-zero on FAIL. Add pytest fixtures: one passing spec, one failing spec per check. Run with
tools/.venv/bin/python. < 5 s. No ROS imports.
```

**P2b — dev overlay (so P5–P9 can run before CAD and AnyLogic numbers exist)** [Sonnet]
```
Add drone_line_sim/config/line_spec.dev.yaml: same structure as line_spec.yaml, filling only the
nulls with plausible placeholder values (poses on a straight line at 0.6 m station pitch, 20-part
magazines, gantry axes x 0–8 m, y 0–1 m, z 0–0.3 m, v_max 1 m/s, a_max 3 m/s², durations that sum
under takt per station, P1 damping/gains from docs/decisions.md). Every placeholder carries the
comment "# PLACEHOLDER". Extend tools/spec.py: load_spec(dev=True) deep-merges the overlay over the
base, base values always win. check_line.py gets --dev; its report lists every value that came
from the overlay under "placeholders". All generators take --dev. Update CLAUDE.md with the rule:
the overlay never overrides a real value, and no deliverable number may come from it.
```

**P3.2 — SolidWorks parameter bridge** [Sonnet]
```
Add tools/gen_sw_params.py: read line_spec.yaml (plate datums, press_nut_hole_d, lead slot size,
stack z heights) and write cad_params.txt in SolidWorks global-variable format, one line per
variable: "datum_dx" = 62.5 (mm, 4 decimals). Output path from --out (default
/mnt/c/Users/Admin/Documents/drone-line-cad/10_skeleton/cad_params.txt). Skip nulls with a WARN
naming the key. Write reports/sw_params.json. Test: round-trip parse of the written file.
```

**P3.3 — mesh pipeline** [Sonnet]
```
Implement tools/mesh_pipeline.py per P3.3: cad_raw/*.stl (mm) → drone_line_sim/meshes/<group>/,
trimesh + pymeshlab decimation to < 5 MB, origin check at the round datum, bbox in metres
compared with / written into line_spec.yaml envelopes (only fill nulls; FAIL on mismatch),
reports/meshes.json. Preserve yaml comments (use ruamel.yaml for the write-back).
Accept --src (default cad_raw/) so it can read /mnt/c/Users/Admin/Documents/drone-line-cad/40_exports/stl.
```

**P4a — pins + pallet** [Opus]
```
Implement tools/cad/pallet.py and tools/cad/pins.py with build123d per P4 B1–B2, reading all
dimensions from line_spec.yaml via tools/spec.py. Export STEP and STL (mm) to cad_out/, run STL
through mesh_pipeline.py, write reports/cad_pallet.json with bbox, mass and derived pin positions.
Add a pytest asserting derived pin XY equals spec datums to 0.01 mm. If a needed spec value is
null, exit with a message naming the key.
```

**P4b — spider carrier + pod tray** [Opus]
```
Implement tools/cad/spider_carrier.py and tools/cad/pod_tray.py per P4 B3–B4, following the
pattern of tools/cad/pallet.py. Arm pockets and bolt-pattern features are computed from
parts.bottom_plate.arm_holes and the arm geometry in the spec — never literal coordinates in the
generator. Leave gripper clearance around the pick features defined by the tool dims. Export
STEP+STL, reports/cad_<part>.json (bbox, mass, derived pocket centres). pytest: every pocket centre
equals its spec source to 0.01 mm; the carrier bbox fits inside gantry z travel.
```

**P4c — magazines, presenter, tools, build_all** [Sonnet]
```
Implement tools/cad/magazine.py (one parameterized generator: part envelope, capacity, key
geometry → one magazine per feeders entry of type magazine), tools/cad/screw_presenter.py and
tools/cad/tools.py (gantry plate tool + carrier tool) per P4 B5–B7, same pattern as pallet.py.
Add tools/cad/build_all.py that runs every generator, then mesh_pipeline, and writes
reports/cad_all.json. ≤ 30 s. Non-zero exit if any generator fails.
```

**P5 — world + model generators** [Sonnet]
```
Implement P5.1–P5.2: templates/main_line.sdf.j2, tools/gen_world.py, tools/gen_models.py.
Primitives for static geometry, includes at spec poses, one test enclosure per st_test cell,
Physics/SceneBroadcaster/UserCommands/Contact plugins, "GENERATED — do not edit" header. Models
get STL visual (scale 0.001), primitive collision, inertia from mass+envelope, named pick/datum
frames. Add tools/checks/world_smoke.sh (headless, ≤30 s) comparing model poses to spec ≤1 mm →
reports/world.json.
```

**P5.3 — gantry + gang-head descriptions** [Sonnet]
```
Implement P5.3 following the working spike_axis.urdf.xacro from P1. urdf/gantry.urdf.xacro loads
line_spec.yaml with xacro.load_yaml and takes every limit (min/max/v_max/a_max) from
spec['gantry']['axes']; prismatic x,y,z, revolute c only if present in the spec; primitive links
with real mass/inertia; damping/friction from the P1 values; TCP frame. urdf/gang_head.urdf.xacro
is a macro: one prismatic stroke joint + N spindle frames laid out from the pattern the spec
names, instantiated once per gang_heads entry. tools/gen_controllers.py writes
config/controllers.yaml (one joint_trajectory_controller for the gantry, one per head, plus
joint_state_broadcaster). Check: tools/checks/descriptions.sh expands every xacro (dev spec),
runs check_urdf, asserts joint limits equal the spec → reports/descriptions.json.
```

**P5.4 — unified launch** [Sonnet]
```
Write drone_line_sim/launch/line.launch.py: standalone gz-harmonic server with the generated
world (gui:=false default, headless -s -r), ros_gz bridge (clock + joint states), one
robot_state_publisher and spawn per robot, controller spawners from config/controllers.yaml,
use_sim_time everywhere, dev:=true|false arg selecting the spec. Reuse the WSL2 GPU env settings
from ~/ros2_ws/src/drone-assembly-line (read only, do not modify that repo). Check:
tools/checks/line_smoke.sh — launch, all controllers active ≤30 s, gantry moves to 3 station
poses with TCP error < 1 mm, kill gz → reports/line_smoke.json.
```

**P6 — feeders + sources** [Sonnet]
```
Implement P6 in drone_line_control: feeders.py (one node, one instance per spec feeders entry;
topics /<id>/available, /<id>/count; service /<id>/take; respawn next part at the same pose after
refill_s via gz spawn), blow feeders spawning the screw at the head after feed_s, sources.py (spawn
subassemblies at buffer poses every interval_s, same available/take interface). Port the spawner
and set_pose logic from ~/ros2_ws/src/drone-assembly-line (copy, don't modify). Append events to
reports/events_<run>.csv (sim_time, feeder, event, count). Check: tools/checks/feeders.sh drains a
20-part magazine headless and verifies refill time appears as wait time → reports/feeders.json.
```

**P7 — grasp, place, drive, ownership** [Opus]
```
Implement P7. Add DetachableJoint instances to gantry.urdf.xacro (one per carried part type from
the spec, gantry parent, own attach/detach/output topics, suppress_child_warning, setup detach at
startup). In drone_line_control/actions/: grasp (set_pose part to TCP, then attach), release
(detach), place (release + set_pose to the pallet datum pose), drive (gang head stroke, dwell,
mark screws driven, consume from the blow feeder). monitors.py keeps an ownership table: every
part instance has exactly one owner (feeder|buffer|tool|pallet|carrier|packed) at every event;
publish and log violations. Check: tools/checks/grasp.sh — take plate from feeder, grasp, move X
0.3 m, plate tracks TCP ≤1 mm, place, plate stays; ownership log shows feeder→tool→pallet with no
gaps or doubles → reports/grasp.json.
```

**P8a — gantry motion + move-time table** [Sonnet]
```
Implement P8.1: drone_line_control/motion.py — waypoint moves with trapezoidal profiles from the
spec's v_max/a_max, safe-Z travel between poses, sent to the gantry joint_trajectory_controller.
Check: tools/checks/gantry_moves.sh drives to every pose in the spec (feeders, buffers, stations),
records TCP error and move time per pose pair → reports/gantry_moves.csv + gantry_moves.json
(max error, FAIL if > 1 mm).
```

**P8b — capability action servers** [Opus]
```
Implement P8.2: one ROS 2 action per capability verb used in spec.sequence (load, place, drive,
advance, inspect, test, pack), built on the P7 actions and P8a motion. Each has a yaml beside it
declaring inputs, timeout_s, retry_safe (bool) and the resource it locks (gantry, a gang head, a
test cell). A resource manager grants locks; a capability never runs without its lock. Every start,
end, timeout and lock wait is appended to the event log. Custom action/interface types go in a new
ament_cmake package drone_line_interfaces. Check: unit test per action with a mocked resource
manager, plus a headless test running place then drive on one pallet.
```

**P8c — sequencer** [Opus]
```
Implement P8.3–P8.5: drone_line_control/sequencer.py runs spec.sequence as a per-pallet state
machine: respects requires, calls capabilities only through the P8b actions, pools st_test as
`count` parallel servers, waits (and logs wait_starved / wait_blocked) instead of failing when a
feeder or resource is unavailable. use_sim_time. Output reports/events_<run>.csv (sim_time,
pallet, step, station, event, detail) and reports/run_summary.json (cycle time per station,
bottleneck, takt pass/fail). Launch arg pallets:=N. Check: tools/checks/one_drone.sh (1 pallet,
headless) and tools/checks/steady_state.sh (10 pallets). No CODESYS-specific code; the sequencer
boundary stays actions/topics only.
```

**P9 — runtime checks** [Sonnet]
```
Implement P9: tools/check_run.py <events.csv> runs every check in the P9 table (ownership,
carry_rigidity, contacts vs spec allowed_contacts, precedence_runtime, takt_runtime, starvation,
drift vs spec duration_s, throughput extrapolated to drones/shift vs 1,000). Add allowed_contacts
to the spec schema (pair + step window). Output reports/run_check.json, PASS/FAIL table on stdout,
non-zero exit on any FAIL. pytest with a synthetic passing log and one failing log per check.
```

**P10a — CI** [Sonnet]
```
Write .github/workflows/ci.yml per P10: job tools (ubuntu, tools/.venv deps, pytest tools/tests,
check_line.py --dev, run every generator then git diff --exit-code for drift); job ros (container
ros:jazzy, colcon build, expand xacros + check_urdf); job sim (workflow_dispatch + weekly only:
Gazebo headless line_smoke.sh and one_drone.sh + check_run.py). Upload reports/ as artifacts.
Pin action versions. Keep the tools job under 3 min.
```

**P10b — Claude Code skills** [Sonnet]
```
Create .claude/skills/{add-station,rebalance,add-variant-part,debug-run}/SKILL.md per the P10
table. Each skill: when to use, exact steps (edit spec → run generators → run check_line.py
[and check_run.py if a run log exists]), and ends by pasting the JSON summary. debug-run reads the
newest reports/events_*.csv and run_check.json and names the first failing invariant, the pallet
and the step. Keep each SKILL.md under 60 lines.
```

**P11 — Blender replay (optional)** [Sonnet]
```
Implement P11: tools/gen_world.py --export-json build/line_spec.json; tools/render/build_scene.py
(run with blender -b -P) builds the line from the JSON (primitives + processed STLs), keyframes
every part and robot from a reports/events_*.csv run log (carried parts follow the TCP), and
writes build/line.blend, build/line.glb, build/still.png, and with --animation an MP4 via ffmpeg.
Three fixed camera poses, no cut logic. Reuse helper patterns from OpenSDL
examples/digital-twin-surrogate/scene/build_scene.py (Apache-2.0, credit it in a header comment).
```

**B-DEBUG — when a check fails** [Opus]
```
tools/checks/<name>.sh fails. Read reports/<name>.json and the last run's logs. State the root
cause with evidence before changing anything, then make the smallest fix, rerun the check, and
report pass/fail. Do not weaken the check or its thresholds to make it pass.
```

---

## Sources

- OpenSDL repository (Apache-2.0): https://github.com/fl-sean03/OpenSDL — see
  `examples/digital-twin-surrogate/scene/README.md`, `build_scene.py`, `check_scene.py`, `twin.yaml`
- OpenSDL docs: https://seanflorez.com/OpenSDL/
- Gazebo DetachableJoint: https://gazebosim.org/api/sim/9/detachablejoints.html
- gz_ros2_control (Jazzy): https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
- build123d: https://build123d.readthedocs.io/
- PyMeshLab: https://pymeshlab.readthedocs.io/
- xacro (`xacro.load_yaml`): https://github.com/ros/xacro/wiki
