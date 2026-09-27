# Decisions

## D-001 — Spec-driven generation, new repo

**Date:** 2026-09-19

**Context:** The gantry + SCARA line architecture replaces the earlier 5×SO-101 design. Scaling
the line (adding stations, cells, variants) requires one source of truth instead of hand-authored
Gazebo worlds, robot descriptions and tooling geometry drifting independently.

**Decision:** `drone_line_sim/config/line_spec.yaml` is the single source of truth. The Gazebo
world, robot descriptions (via xacro), tooling geometry, controller config and all checks are
generated from it, never hand-edited. The old `drone-assembly-line` repo is frozen and kept only
as reference.

## D-002 — P1 spike values: gantry-axis dynamics defaults

**Date:** 2026-09-19

**Context:** P1 (`tools/checks/spike.sh`) drives a one-axis cart on a rail through
`gz_ros2_control` with a position command interface. A 0.5 m move over 2 s ended with zero
overshoot, no oscillation and settling in ~2.0–2.5 s. The controller manager appears once the
`gz_ros2_control-system` plugin has a valid `<parameters>` path.

**Values that passed** (`drone_line_sim/urdf/spike_axis.urdf.xacro`, `config/spike_controllers.yaml`):

| Quantity | Value |
|---|---|
| Rail mass / cart mass | 5 kg / 2 kg (box inertia from dimensions) |
| Joint `damping` | 5.0 N·s/m |
| Joint `friction` | 0.5 N |
| `position_proportional_gain` (`gz_ros2_control` default, not overridden) | 0.1 |
| Controller update rate | 100 Hz |
| JTC goal tolerance / `goal_time` | 0.005 m / 1.0 s |

**Caveat:** in position mode `GazeboSimSystem` commands joint velocity from the position error,
so joint dynamics barely affect the response. These values are defaults, not a tuned result;
re-verify them when a velocity or effort interface, or a loaded gantry axis, is introduced.

**Decision:** gantry axes take `damping`, `friction` and `position_gain` (spec fields on
`gantry.axes.<axis>`) with the values above until a heavier axis proves them wrong.

## D-003 — P4 B3/B4: fixture spec surface and materials

**Date:** 2026-09-19

**Context:** `tools/cad/spider_carrier.py` and `tools/cad/pod_tray.py` must compute arm pockets
from the plate's own screw pattern and leave gripper clearance sized by the tool (plan P4 B3–B4),
but the spec had no arm-hole pattern, no arm or ESC geometry and no tool dimensions — only the
dangling reference `gang_heads.gh_arm_sandwich.pattern_from: bottom_plate.arm_holes`.

**Decision — spec additions** (all null in `line_spec.yaml`, placeholders in the dev overlay):

| Key | Meaning |
|---|---|
| `parts.bottom_plate.arm_holes` | `{per_arm, positions}` — every arm-sandwich screw in the plate frame. The holes group into `len(positions) / per_arm` arms by angle about the plate centre, so the generator never sees an arm index. |
| `parts.arm` | `dims: [length, width, thickness]` along the arm axis, plus `hole_inset` — inboard end to the centroid of the arm's own mount holes. |
| `parts.esc` | `dims` plus `z_in_stack`: the ESC's final underside height, which is where the carrier suspends it. |
| `gantry.tools` | was `list[str]`, now `{name: {grip, dims}}`; `dims = [x, y, reach]` is the envelope the tool needs at a pick feature. |
| `sequence.place_pod.tool` | `tool_carrier` — a fixture generator reads the tool that lifts it from the step that places it, rather than naming one. |

`parts.carrier_spider.pick` and `parts.carrier_pod.pick` were **removed** from the base spec: on a
`build123d` part the pick pose is an output. The generators derive it from the pick boss they
build and report it; if the spec later states `pick`, x,y are honoured as the grip point and a z
that disagrees with the built boss is a FAIL (a placeholder z is only a WARN).

**Decision — materials** (§7.10 open item, written into each STEP header via the root label):

| Fixture | Material | Why |
|---|---|---|
| Spider carrier | 6061-T6 aluminium | Sits under a soldering iron for 34 joints and is wiped with IPA between builds; no polymer survives both. |
| Pod tray | ESD-safe PA12 (SLS) | Sub-line B hands it a finished pod — no iron ever touches it — so printed is enough, and ESD-safe because it carries the avionics. |

**Consequence:** the mid plate is placed into the gap **laterally**, not by a vertical drop: the
carrier's bridge and ESC nest sit above the mid plate's final top face, so the plate is lowered
beside the stack and brought in under the suspended ESC. `derived.bridge.mid_plate_clearance_mm`
in `reports/cad_carrier_spider.json` is the corridor that motion needs.

## D-004 — P4 B5–B7: feeding tooling, and the poka-yoke key as a spec value

**Date:** 2026-09-19

**Context:** `tools/cad/magazine.py` builds one magazine per `feeders` entry of type `magazine`
(plan P4 B5), and B5's spec reads are "part envelope, capacity, **key geometry**". The envelope
and the capacity were already in the spec; the key — the poka-yoke feature that stops a side plate
going in inside-out (§7.5, §16) — was not, and it belongs to the part, not to the magazine: the
magazine's rib is the negative of the part's own corner cut.

**Decision — spec addition:** `parts.<id>.key: [x, y]`, the corner cut at the part's **+x,+y**
corner in its own frame (null in `line_spec.yaml` for `side_plate`, placeholder in the overlay).
Absent means "no key", which is a WARN, not a failure: a symmetric plate does not need one.

The generator does not take the key on trust. It seats the part in the magazine both ways round
and intersects it with the rib: the correct orientation must clash by 0 mm³ and the mirrored one
by more than 0, or the key is a FAIL (`derived.key.seated_clash_mm3` / `flipped_clash_mm3` in
`reports/cad_magazines.json`).

**Decision — where each generator's frame sits**, so the generated models drop into the world at
their spec pose with no offset:

| Generator | Frame | z = 0 |
|---|---|---|
| `magazine.py` | `feeders.<id>.pose` | the pick plane — the rim, and the top face of the presented part |
| `screw_presenter.py` | the plate frame of whatever `gang_heads.<head>.pattern_from` names | the screw tips, facing the work |
| `tools.py` | the tool centre point | the pick plane, the face that meets the part |

**Decision — reports:** a generator that builds one solid per spec entry writes **one** report
(`cad_magazines`, `cad_presenters`, `cad_tools`) with an `items` map keyed by the spec id, rather
than one file per entry. It resolves every entry before writing any file, so a run that fails
leaves no half-built `cad_out/` behind.

**Decision — a pose that cannot hold its own stack:** a magazine deeper than
`feeders.<id>.pose[2]` would stand below the floor. With a real pose that is a FAIL (the spec
contradicts itself); with a placeholder pose it is a WARN, following the same rule as
`pick_z_note` — no deliverable number comes from the overlay, so an overlay value has nothing to
be right about. Under `--dev` today, all three magazines warn: the overlay's 0.1 m feeder height
cannot hold 20 plates.

**Consequence:** `tools/cad/build_all.py` runs the six generators in build order and then
`mesh_pipeline.py`, in **one process** (build123d and OCP are imported once, which is most of the
plan's 30 s budget — the whole build is ~1.7 s under `--dev`). A missing raw-STL folder is a skip,
not a failure, so the build123d half stays runnable before any SolidWorks export exists;
`--require-meshes` insists on it.

## D-005 — P5.1/P5.2: what the generated world and models are made of

**Date:** 2026-09-19

**Context:** the world and the models are generated from `line_spec.yaml` (plan P5). Three things
had to be settled before the first world could load at all.

**Decision — a model's frame is the frame the part is already described in**, so an `<include>`
at a spec pose needs no offset anywhere:

| Model | Origin | z = 0 |
|---|---|---|
| plate (SolidWorks) | the plate frame, `CS_DATUM` | the plate's underside; the box is centred on the **datum midpoint**, not on the round datum |
| pallet / carrier (build123d) | the generator's own frame | its rest plane; the collision box is the **measured** bbox from `reports/cad_<part>.json`, which is why it is not centred on the origin |
| magazine | `feeders.<id>.pose` | the pick plane (the rim) |

**Decision — a missing mesh is never named.** gz resolves `<uri>` and `<include>` when it *parses*
a world: a model that names a mesh which is not there does not fail to render, it fails to load
the world. So `gen_models.py` falls back to the collision primitive as the visual (with a WARN)
when a part's STL has not been built yet, and `gen_world.py` refuses to write a world whose
include has no model directory. A magazine feeder with no model is a FAIL, not a box: a magazine
*is* a model, so a missing one means gen_models never ran.

**Decision — a generated mesh is copied into the package.** `cad_out/` is not installed with
`drone_line_sim`, so anything built there is copied to `meshes/tooling/<id>.stl` (where
mesh_pipeline.py already puts the SolidWorks meshes) and referenced by a path relative to the
model file. The rule is "the package is self-contained", so any mesh from outside it is copied.

**Decision — link names are `<Model>_link`.** The naming table's link form is the model's own
name, but `/world/<w>/pose/info` keys poses by entity *name*: a link named exactly like its model
makes every pose query ambiguous. The suffix is mechanical and keeps the tooling honest.

**Decision — static geometry sits below the pose it marks.** A station pose is where the
*workpiece* stands, so the station pad and the conveyor bed end at the pallet's underside — the
pallet's own measured bbox says how far below that is. A pad drawn *at* the pose is inside the
pallet standing there, and physics throws it out; that is how this was found.

**Consequence — `tools/checks/world_smoke.sh`** regenerates the models and the world, loads it
headless on a private `GZ_PARTITION`, waits for the world to reach N iterations, and compares
every model in `reports/gen_world.json` against **the spec** (a test cell against its station's
pose plus `k x cell_pitch`) to 1 mm, writing `reports/world.json`. It reads every pose from one
`gz topic -e -t /world/<w>/pose/info` message: a `gz model -p` per model took half the 30 s
budget, and past a handful in parallel the service starts refusing. The whole check runs in
~13 s. It defaults to `--dev` because the base spec's poses are still null; `--real` runs it
against `line_spec.yaml` alone.

## D-006 — P5.3: the robot descriptions, and what a limit is allowed to come from

**Date:** 2026-09-20

**Context:** the gantry and the gang heads become xacro descriptions driven by
`xacro.load_yaml(line_spec.yaml)` (plan P5.3), with `tools/gen_controllers.py` writing
`config/controllers.yaml` beside them. The P1 spike proved one axis moves; this is the shape the
rest of the machines take.

**Decision — one robot, not three.** `gz_ros2_control` starts a controller manager per model, and
one controller manager reads one parameter file. P5.3 asks for the gantry controller and one per
head in a single `controllers.yaml`, so they share a manager: `urdf/line.urdf.xacro` is the
description, `urdf/gantry.urdf.xacro` and `urdf/gang_head.urdf.xacro` are macro libraries it
instantiates. The machines are welded to the world and never touch each other, so nothing is lost
by describing them together, and P5.4 spawns one model with one `robot_description`.

**Decision — every *limit* comes from the spec; the *structure* is an envelope typed in the
xacro.** `lower`/`upper`/`velocity` are `gantry.axes.<axis>.{min,max,v_max}` and
`gang_heads.<id>.{stroke,v_max}`, and the check fails on any difference. Masses and the boxes a
machine is drawn as are local constants collected at the top of each file — the same rule
`gen_world.py` follows for the world's static geometry (D-005), for the same reason: they say how
heavy and how big, which is what an inertia and a picture need, and no deliverable number comes
from them.

**Decision — effort is derived, never typed.** A URDF joint needs one and the spec has no force
field, so each axis gets the force to accelerate what it actually carries: moved mass x `a_max`,
plus that axis's friction, plus gravity on the vertical axis. This is the one place `a_max` is
consumed in the description; `joint_trajectory_controller` has no acceleration parameter in Jazzy,
so `a_max` is reported in `reports/gen_controllers.json` (as `joint_limits`) for the sequencer to
time a move with, and is not written to the yaml.

**Decision — the gantry has no collision geometry.** In position mode `GazeboSimSystem` drives the
joint from position error (D-002's caveat), so a colliding structure fights the drive instead of
stopping it. Picking is a `DetachableJoint` at the TCP (P7), not a contact.

**Decision — a joint value *is* a coordinate.** The gantry's axis frames carry no offsets, so
`Gantry_tcp` sits at `gantry.mount + (x, y, z)` — the same arithmetic `check_line.py`'s reach check
does (`pose - mount`). A spindle frame's `+z` points along the drive direction, down, up or
inward, so a P7 action advances any spindle the same way.

**Decision — `opposed: true` is two rams.** A URDF joint moves one child link, and two banks
closing on a workpiece from both sides are two links: `<id>_stroke_1` and `<id>_stroke_2`, same
limits, one controller over both. An `opposed` head that is not `horizontal` is a spec error.

**Consequence — three new spec fields.** `gang_heads.<id>.v_max` and `.a_max` (a ram's joint
cannot be written without them), and `damping`/`friction`/`position_gain` for the C axis in the dev
overlay, the rotary counterparts of D-002's values. Damping and friction stay optional: a
`<dynamics>` tag is written only for an axis the spec gives them for.

**Consequence — `tools/spec.py --dump`.** `xacro.load_yaml` takes one path and knows nothing about
the dev overlay, so a dev expansion needs the merged spec as a file. It is written to
`reports/line_spec.dev.yaml`, generated and never installed.

**Consequence — `tools/checks/descriptions.sh`** regenerates `controllers.yaml` and the merged
spec, expands every top-level xacro in `urdf/` (a `<robot>` with a `name`; the macro libraries are
recognised by not having one), runs `check_urdf` on each, and compares the result with the spec:
joint type and limits, one spindle frame per spindle at the pattern's own positions, and the mount
poses. It ends by comparing the URDF's movable joints with the joints `controllers.yaml`
configures — the only thing tying the xacro's naming to the generator's, since the two derive the
same names in different languages. Writes `reports/descriptions.json`; `--dev` by default because
the base spec's limits are still null, `--real` for the base spec alone. No Gazebo, no controller
manager: this is the description alone, and it runs in about 3 s.

## D-007 — P5.4: launching the line, and what a position-driven vertical axis needs

**Date:** 2026-09-20

**Context:** `launch/line.launch.py` starts the generated world and the one `line` robot
(D-006), and `tools/checks/line_smoke.sh` proves it: every controller active <= 30 s from launch
(measured ~6-8 s), then the gantry visits the first three stations the sequence places at with a
gantry tool, each with TCP error < 1 mm.

**Decision — the TCP error is measured against Gazebo, not against `/joint_states`.** The pose of
the `Gantry_c` link (where `Gantry_tcp` sits, no offset) comes from `/world/<w>/pose/info` and is
compared with the station's spec pose. A joint that reports its target while the link is
elsewhere would pass a joint-state check. Negative test: launching with the gantry mount shifted
5 mm makes all three stations FAIL at exactly 5.000 mm; the passing run reads <= 0.001 mm.

**Decision — links that hang on a vertical driven axis are gravity-free in Gazebo.**
`GazeboSimSystem` in position mode commands a joint *velocity* from position error (D-002's
caveat, which said to re-verify on a loaded axis). Measured on the loaded gantry: with gravity on
the Z axis never left its lower limit, and raising its derived effort 3x changed nothing; with
world gravity zeroed it reached its target. The vertical gang head's ram also sagged ~10 um at
rest. So the Z and C links and every ram carry `<gravity>false</gravity>`
(`urdf/inertia.urdf.xacro: no_gravity`) — what a real Z drive's brake or counterbalance does. The
derived efforts still include the gravity term; they are conservative, not load-bearing.

**Consequence — moves are planned at half of `v_max`/`a_max`.** The X axis (~50 kg moved,
effort = mass x `a_max`) fell 5 cm behind a cubic move that asked for 87% of `a_max` and aborted
on the 5 cm path tolerance. The cubic between rest points peaks at `1.5 d/t` and `6 d/t^2`, so
`line_measure.py` picks `t` to keep both under `--speed-fraction` (0.5) of the spec limits. The P8
sequencer has the same constraint: **`a_max` is a ceiling, not a planning value.**

**Consequence — the launch reads the installed package.** The dev spec merged with its overlay
(`config/line_spec.merged.dev.yaml`, from `spec.py --dev --dump`, gitignored) and
`controllers.dev.yaml` are generated into `config/`, so `line_smoke.sh` regenerates everything
and rebuilds `drone_line_sim` before it launches. A dev run writes `reports/line_smoke.dev.json`
and `--real` writes `reports/line_smoke.json` (CLAUDE.md P0); `dev:=true` is the launch default
until the base spec has real values, and the launch says so on the console.

**Consequence — the "GPU env" from `drone-assembly-line` is three variables.** Its launch sets
`GZ_SIM_RESOURCE_PATH` (models), `GZ_SIM_SYSTEM_PLUGIN_PATH` and `LD_LIBRARY_PATH` (both
`/opt/ros/jazzy/lib`, where `gz_ros2_control-system` lives) and nothing GPU- or Mesa-specific; a
headless `-s` server renders nothing. This launch sets the same three.

## D-008 — Zoned multi-gantry

**Date:** 2026-09-27

**Context:** one gantry cannot make 8+ picks inside a 20.2 s takt. The line's gantry steps —
load pallet, place spider, place mid plate, 4 standoffs, place pod, fit top plate (qty 5) — are
13 pick-and-places per drone, at ~3–4.4 s each on the dev overlay's 1 m/s, 3 m/s² axes
(`tools/estimate_moves.py`): 46 s summed over the three zones, before the travel one machine would
add between them — more than twice the takt.

**Decision:** one gantry per zone, no overlapping zones. `gantry:` becomes `gantries:`, a map keyed
`gantry_<n>`; each entry keeps `axes`, `tools` and `mount` and adds `zone: [station ids]`. A
top-level `gantry_defaults: {grip_s, release_s, settle_s, safe_z, damping, friction}` holds the
pick-and-place timings and the P1 dynamics (D-002: 5.0 N·s/m, 0.5 N, real values in the base spec).

| Rule | Where | Why there |
|---|---|---|
| every zone station exists; no station in two zones; ids are `gantry_<n>` | `spec.py` (pydantic) | structural: the spec is wrong, not unfinished |
| a step with a `tool`, or a `part` drawn `from` somewhere, stands at a station in a zone | `spec.py` (pydantic) | skipped while any zone is null — that station may be in the unknown zone |
| one tool id is one design on every gantry that carries it | `spec.py` (pydantic) | the CAD generators build each tool id once |
| a step's `tool` is on the gantry whose zone holds its station | `check_line.py` `schema` | a reference, like every other one there |
| the station and the `from` feeder/buffer of each gantry step are inside **that** gantry's limits | `check_line.py` `reach` | needs WARN on a null, which a validator cannot give; replaces the single-gantry reach |
| each gantry's moves per drone fit the takt | `check_line.py` `gantry_load` → `estimate_moves.py` | FAIL above takt, UNRESOLVED on a null input |

**Decision — the move model** (`tools/estimate_moves.py`, no ROS): XY to the pick at `safe_z`,
Z down, grip, Z up, XY to the place, Z down, release, Z up, plus `settle_s` after each Z move.
Each axis is trapezoidal (`d/v + v/a` if `d >= v²/a`, else `2√(d/a)`); XY is the slower of X and
Y. A gantry works its steps as a tour, once per drone: each cycle starts where the last one
placed, so the first cycle of a `qty: n` step includes the move over from the previous station
and cycles 2..n start at its own. This is the static bound; D-007's "plan at half of
`v_max`/`a_max`" applies to what the sequencer commands, and would roughly double these times.

**Decision — dynamics fall back per axis kind.** An x/y/z axis without its own
`damping`/`friction` takes `gantry_defaults`; C only ever takes its own, because the defaults are
linear units. `position_gain` stays per axis.

**Consequence — descriptions.** Still one robot (D-006): `urdf/gantry.urdf.xacro`'s macro is
instantiated once per `gantries` entry as `Gantry_<n>` with joints `gantry_<n>_<axis>` and TCP link
`Gantry_<n>_Tcp`; `gen_controllers.py` writes one `gantry_<n>_controller` per gantry;
`gen_world.py` one `Rail_Gantry_<n>` each. The CAD generators read a tool at
`gantries.<id>.tools.<tool>` and a carrier's z stroke from the gantry whose zone places it.

**Consequence — dev overlay.** Zones: gantry_1 = load pallet, place spider, place mid plate;
gantry_2 = install standoffs, place pod; gantry_3 = fit top plate. Each gantry's x/y limits cover
only its zone's stations, feeders and buffers plus 0.1 m, with the mount at the zone's lowest x
minus 0.1. The existing feeder/buffer poses already sat beside their stations, so all were inside
their zone gantry's reach and none moved. `safe_z` is 0.15 m; `fd_standoff` stands at exactly
0.15, so the gantry picks standoffs without a Z move.

**Not verified:** `tools/checks/line_smoke.sh` (Gazebo) was not run for this change;
`line_measure.py` now drives the zone gantry per station but has only been compiled.

## D-009 — Gantry frame, step split, gang pick, speed scale, takt budget, safe_z

**Date:** 2026-09-27

**Context:** the first layout STEP (tools/cad/layout.py) found gantry_3's posts standing in the
conveyor: D-008 sized each gantry's y travel to its zone's items only, and gantry_3's items are all
on one side of the belt. The move estimate had four more gaps: `fit_top_plate` counted 5
top-plate picks because its qty was the screw count; the 4 standoffs were 4 separate picks; it
planned at full `v_max`/`a_max` although D-007 found the sequencer must plan at half; and it
compared with the whole takt although the pallet spends part of every takt moving.

**Decision — frame rule.** A gantry's y travel is the union of its zone items' y (plus 0.1 m) and
the whole conveyor (±`layout.conveyor.width`/2 plus 0.1 m); its x travel stays zone items plus
0.1 m. Posts, `post_size` square, stand just outside the travel corners and must clear the belt
edge by `layout.gantry_frame.conveyor_clearance`. Check `gantry_straddle` FAILs a post in the belt
or inside that clearance, on every gantry. Dev: gantry_3's y min 0.4 → 0.25 (its posts now clear
the belt by exactly 0.1 m); gantry_1/2 already spanned ±0.4.

**Decision — step semantics.** A step either moves a part with a gantry or drives fasteners, never
both (schema rule: a gantry-served step — `tool`, or `part` + `from` — may not be `drive` or name
a `fastener`). `fit_top_plate` became `place_top_plate` (place, qty 1, `tool_plate` on gantry_3)
and `drive_top_plate` (drive, qty 5, driver not yet chosen). No other step mixed the two;
`fit_side_plates` drives with no part at all, and nothing in the sequence draws from
`fd_side_plate` yet — side-plate placement is still missing from the sequence. estimate_moves.py
counts only capabilities `place`, `load` and `pick` (`pick` is new in the schema).

**Decision — gang pick.** `install_standoffs` is one pick of all 4 standoffs with a new gang
gripper `standoff_gang` on gantry_2, from `fd_standoff`, now a `nest` feeder with `per_pick: 4`.
`feeder_rate` counts parts, so its demand is qty × `per_pick`. tools/cad/tools.py builds
`standoff_gang` as its generic two-jaw finger tool for now; the 4-finger gang is not modelled.

**Decision — speed scale.** `gantry_defaults.plan_speed_scale` (dev 0.5, D-007) scales every
axis's `v_max` and `a_max` in the estimate; the table also shows scale 1.0.

**Decision — takt budget.** Each gantry's load is compared with `takt_s − transfer_s`,
`transfer_s = conveyor.pitch / layout.conveyor.belt_speed + layout.stop_unit_cycle_s`
(dev: 0.6 / 0.3 + 1.0 = 3.0 s, budget 17.2 s). `conveyor.index_s` (dev 2.0 s) is the same
quantity as pitch / belt_speed and is not read; one of the two should go.

**Decision — safe_z per gantry** (tools/gantry_geometry.py, replacing the fixed
`gantry_defaults.safe_z`): the tallest item under the gantry's XY travel — a station at its pose z
plus the tallest part standing on the pallet there (the pallet and every part placed at that
station or an earlier one; nothing is ever taken off), a feeder or buffer at its pick plane — plus
the tallest part the gantry carries, plus `gantry_defaults.z_margin` (dev 0.03 m). A build123d
part's height is its generator's output (D-003), read from `reports/cad_<part>.json`; a missing
report is UNRESOLVED, never a guess. Check `safe_z` FAILs an item above `safe_z − z_margin` (true
by construction today, so it guards the derivation) and a safe_z outside the gantry's own z
travel, which is the constraint that can actually bite. Dev: gantry_1 0.240 m (spider carrier
standing 104.4 mm + hanging 105.4 mm), gantry_2 0.260 m (standoff nest at 0.15 + pod tray 79.5 mm),
gantry_3 0.170 m (pod tray at st_fit_top_plate 136.5 mm + top plate 3 mm).

**Result — gantry_1 is over budget at scale 0.5:** 19.54 s against 17.2 s (113.6%); 13.68 s
(79.5%) at scale 1.0. gantry_2 14.73 s (85.6%), gantry_3 4.87 s (28.3%). Per gantry_1 cycle, the
four Z moves (0.24 m down to a station, 0.14 m to a buffer) take 3.25 s and the tour's return from
the mid-plate feeder at x = 1.2 to `buf_plate` at x = 0 takes 2.73 s. Candidate fixes, none
applied: let sub-line C load the bottom plate onto the pallet itself, so `load_pallet` leaves
gantry_1 (~12.2 s, 71%); move `buf_plate` next to `buf_spider` to shorten the tour; or plan
gantry_1 above 0.5 once its X axis is shown to track at that speed (D-007).
