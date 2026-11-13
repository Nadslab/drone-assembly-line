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

## D-010 — Actor audit, side plates, single-purpose loader at pallet return

**Date:** 2026-10-01

**Context:** D-009 left two gaps it could see: nothing drew from `fd_side_plate`, and
`drive_top_plate` had no driver. Neither was caught by a check, so a rule was written first and run
on the D-009 spec before anything was fixed.

**Decision — check `actors`** (`tools/check_line.py`, after `schema`). Actor rules:
- every station with an active step has **exactly one actor type**: gantry (a step with a `tool`,
  or a part drawn `from` a feeder/buffer), driver (a drive step's `head`, gang head or scara), test
  (a test step), or the station's stated `actor`. A station whose steps are all parked is skipped.
- a station states `actor` only where no step can say it: `operator`, or `sub_line` with `source`
  (the buffer or source it loads from). `actor: null` = not decided: UNRESOLVED, not a FAIL.
- a drive step without a `head` FAILs.

Material rules:
- a step's part comes `from` a feeder that holds that part, from a buffer a source fills with it (or
  with what it carries), or from a sub_line station's `source`. Anything else FAILs.
- a step's fastener comes from that fastener's feeder. A drive step that names no `fastener` is
  UNRESOLVED: its screw is not chosen yet.
- every feeder and every buffer is drawn from by at least one active step.

Each rule has a fixture in `tools/tests/conftest.py` (12 cases).

**Audit of the D-009 spec** (same result with and without `--dev`):

| # | Finding | Level | Now |
|---|---|---|---|
| 1 | `feeder fd_side_plate`: no active step draws from it | FAIL | fixed: `place_side_plates` |
| 2 | `drive_top_plate`: drive step has no driver | FAIL | fixed: `gh_top_plate` |
| 3 | `seat_esc`: drive step has no driver | FAIL | fixed: `gh_seat_esc` |
| 4 | `st_seat_esc`: no actor | FAIL | fixed by 3 |
| 5 | `st_audit_torque`: no actor | FAIL | **open gap A**: `actor: null` |
| 6 | `st_pack`: no actor | FAIL | **open gap A**: `actor: null` |
| 7 | `seat_esc`, `fit_side_plates`, `drive_top_plate` name no fastener | WARN | **open gap B** |

`st_mount_servo` is parked (§15.3) and has no driver either; the audit skips it (**open gap C**).

**Decision — side plates.** New step `place_side_plates` at `st_fit_side_plates`: place, qty 2,
`side_plate` from `fd_side_plate`, `tool_plate`. `fit_side_plates` now requires it.
`st_fit_side_plates` moves into gantry_3's zone. Under the D-009 frame rule (dev), gantry_3 has
x 4.1–5.5 m in the line frame (mount x 4.1, travel 0–1.4). Its y travel stays 0.25–0.9: the side
plate feeder sits at the same y as the top plate feeder. safe_z is 0.203 m
(`fd_side_plate` pick plane 0.17 + a 3 mm plate + 0.03).

**Decision — drivers.** Neither existing gang head can be reused. `gh_arm_sandwich` is the
8-hole arm pattern, and `gh_side_plates` drives horizontally. A gang head also mounts at exactly one
station (`gen_controllers.validate_heads`). So two new heads: `gh_seat_esc` and `gh_top_plate`,
both `direction: down`. Every dimension is null in the base spec. In dev they are PLACEHOLDERs:
4 and 5 spindles (one per screw), stroke 0.05, v_max 0.1, a_max 0.5. Neither has a
`pattern_from`, so the xacro spreads their spindles evenly. That gives the head's width, not screw
positions.

**Decision — single-purpose loader at pallet return.** `load_pallet` leaves gantry_1. Station
`st_pallet_loader` (index 1, where returned pallets re-enter the line) replaces `st_load_pallet`:
`actor: sub_line, source: buf_plate`. Sub-line C's bottom plate arrives on the pallet, and the step
has no `from` and no tool. gantry_1's zone is now `[st_place_spider, st_place_mid_plate]`, so its
dev x travel shrinks to 0.5–1.3 m (mount x 0.5, travel 0–0.8). The station name follows the brief.
It breaks the naming rule that station ids use a verb; `st_load_pallet` would follow it.

**Decision — `conveyor.index_s` is gone.** The index time is `conveyor.pitch /
layout.conveyor.belt_speed` (`spec.index_s()`). It is used for `transfer_s` and reported as
`index_s` in `move_estimate*.json`. The key is now a schema error, so the time cannot be stated
twice again.

**Result — move estimate (dev, budget 17.2 s = 20.2 − (0.6/0.3 + 1.0)):**

| gantry | steps | @0.5 | @1.0 | load @0.5 | status |
|---|---|---|---|---|---|
| gantry_1 | place_spider 6.11, place_mid_plate 6.11 | 12.23 s | 8.72 s | 71.1% | PASS |
| gantry_2 | install_standoffs 7.25, place_pod 7.48 | 14.73 s | 9.97 s | 85.6% | PASS |
| gantry_3 | place_side_plates ×2 11.28, place_top_plate 6.99 | 18.27 s | 12.70 s | 106.2% | **FAIL** |

gantry_1 is fixed. gantry_3 is now over budget. Its tour crosses 1.2 m twice per drone: from the
top-plate station (x 5.4) to the side-plate feeder (x 4.2), then from the side-plate station back to
the top-plate feeder. Each leg takes 2.73 s at scale 0.5, so 5.47 s of the 18.27 s. The 4 Z moves at
the station each take 0.84 s, including settle. Candidate fixes, none applied:
- a gang pick of both side plates (one cycle, about −4.7 s → about 13.5 s, 79%);
- `fd_top_plate` next to `st_fit_side_plates`, or `fd_side_plate` next to `st_fit_top_plate`;
- side plates on their own gantry.

**Open gaps** (each visible in `check_line.py`):
- **A.** Who works `st_audit_torque` and `st_pack`: `actor: null` (WARN). The dev overlay says
  `operator`, PLACEHOLDER.
- **B.** Fasteners for `seat_esc` (4), `fit_side_plates` (dev 8) and `drive_top_plate` (5): no
  fastener id, so no feeder (WARN). Choosing them needs screw lengths from CAD, plus one feeder per
  length (§7.9).
- **C.** `mount_servo` (parked) has no driver.
- **D. FAIL, `actors`:** `st_fit_side_plates` and `st_fit_top_plate` each have a gantry and a
  driver. This rule's fixes created both. Fix: split each into a place station and a drive station,
  or rule that a gang head may share a station with the gantry that places there. Not applied: the
  first moves every downstream station, and the second is the user's call.
- **E. FAIL, `layout.py --dev`:** 9 interferences between gantry_2 and gantry_3. Each guard is
  `guard_offset` 0.3 m outside its envelope. With side plates on gantry_3, gantry_3's envelope
  starts at x 4.1, 0.4 m after gantry_2's ends at 3.7. Every frame and guard of the two now
  overlaps. Before D-010 there were 0 interferences. Fix: one shared guard around adjacent
  gantries (layout.py), or a free station between their zones. The clearances were not reduced.
- **F. FAIL, `gantry_load`:** gantry_3, above.

## D-011 — Gantry plus gang head at one station, station time, paired side-plate pick, merged guards

**Date:** 2026-10-01

**Context:** D-010 left three FAILs (open gaps D, E, F). `actors` failed on the two stations where
a gantry places and a gang head drives. gantry_3 was over budget at 106.2%. layout.py found 9
interferences between gantry_2's and gantry_3's guards.

**Decision — one primary actor, plus the gang head that fastens what the gantry placed**
(replaces D-010's "exactly one actor type"). A station has exactly one primary actor: gantry,
driver, operator, sub_line or test. A gantry station may also have one gang head, if:
- it is mounted at that station: every step that names it stands there;
- its drive step `requires` a gantry step at the same station.

Any other drive step at a gantry station counts as a second primary actor and FAILs. Also FAIL:
- two gang heads at a gantry station;
- two heads at a driver station;
- a gang head named at two stations.

`st_fit_side_plates` is now gantry + gh_side_plates, and `st_fit_top_plate` is gantry +
gh_top_plate. This closes open gap D. The actors fixtures were updated: the old two-actor-types
case is now a passing test, and 5 new failing cases were added.

**Decision — check `station_time`** (`check_line.py`, computed by
`estimate_moves.station_times`). For each station with a gantry or drive step:
`station_time = its gantry step times + drive time + index_s`.
- A gantry step time is the estimate's per-step time at `plan_speed_scale`. It includes the tour
  leg that brings the gantry to the station.
- Drive time is `gang_heads.<id>.drive_s` (new: base null, dev 3.0 s PLACEHOLDER) ×
  `ceil(qty / spindles)` passes.
- index_s is `pitch / belt_speed`.

FAIL above `takt_s`; a null input is UNRESOLVED. As briefed, the stop unit's
`stop_unit_cycle_s` is not in the sum, though `gantry_load`'s budget includes it. Stations with
no gantry or drive step (the sub_line loader, operator stations, test) are not in the table.
`takt` still covers their `duration_s`.

**Decision — paired side-plate pick.**
- New tool `side_plate_pair` on gantry_3 (vacuum). It has two pockets and picks the left and right
  side plate together. Its dims are null in base; dev is [0.12, 0.14, 0.02] PLACEHOLDER.
  docs/naming.md adds **pair gripper**, `<part>_pair`.
- `fd_side_plate` is now a paired magazine, `per_pick: 2`: one left and one right plate per pick.
- `place_side_plates` is qty 1 and uses `side_plate_pair`.
- Not modelled, as with `standoff_gang` in D-009: tools/cad/tools.py builds the pair as its generic
  vacuum tool (2 cups). tools/cad/magazine.py still builds one 40-plate stack and ignores
  `per_pick`.

**Decision — merged guards** (`tools/cad/layout.py`). Two gantries' guards overlap or touch when
their outer faces (envelope + `guard_offset` + wall) meet in x and y. They are then replaced by
one guard around the bounding box of both travel envelopes, plus `guard_offset`, with the
conveyor tunnel kept. It is named `Guard_Gantry_<a>_Gantry_<b>`. Grouping is transitive, so a
chain of three gantries gets one guard. `guard_offset` itself is unchanged. Dev:
`Guard_Gantry_2_Gantry_3` replaces the two guards that crossed each other's posts. This closes
open gap E.

**Decision — `st_load_pallet` again.** D-010's `st_pallet_loader` is renamed back so the station id
keeps the verb rule. It is still `actor: sub_line, source: buf_plate`. layout.py's two references
already said `st_load_pallet`.

**Result (dev, budget 17.2 s):**

| gantry | @0.5 | @1.0 | load @0.5 | status |
|---|---|---|---|---|
| gantry_1 | 12.23 s | 8.72 s | 71.1% | PASS |
| gantry_2 | 14.73 s | 9.97 s | 85.6% | PASS |
| gantry_3 | 13.53 s (side pair 6.54 + top plate 6.99) | 9.17 s | 78.7% | PASS |

This closes open gap F. The busiest station is `st_fit_side_plates` at 14.54 s of 20.2 s: pair
pick 6.54, plus 2 passes of `gh_side_plates` (8 screws, 4 spindles) at 6.0 s, plus index 2.0.
layout.py --dev has 0 interferences.

**Open gaps** (carried from D-010; still open):
- **A.** Who works `st_audit_torque` and `st_pack`: `actor: null` (WARN). Dev says `operator`,
  PLACEHOLDER.
- **B.** No fastener is chosen for `seat_esc`, `fit_side_plates` or `drive_top_plate` (WARN).
- **C.** `mount_servo` (parked) has no driver.

## D-012 — One SCARA per pick-and-place station, option B arm pins, milestones, end of line deferred

**Date:** 2026-10-01

**Context:** two problems with zoned gantries (D-008..D-011).
- **Too slow.** One gantry could not do the line's picks in a takt (D-008). Zoned, each gantry still
  toured its zone once per drone, and the tour legs dominated: gantry_2 ran at 85.6% of its
  17.2 s budget, and gantry_3 only got under budget (78.7%) with a paired side-plate pick.
- **Gang heads in gantry zones.** D-011 put `gh_side_plates` and `gh_top_plate` at stations inside
  gantry_3's zone. A head hanging over a station stands in the volume the gantry's Z column
  travels through, so the two would collide.

**Decision — one SCARA per pick-and-place station.** `gantries:` and `gantry_defaults:` are gone.
`scaras:` is a map keyed `scara_<station id without st_>`. spec.py enforces the key, so a station
has at most one SCARA. Each entry has:
- `station`, `base_side` (+y/-y), `base_offset_y`;
- `reach {r_min, r_max}`, `z_stroke`, `payload_kg`;
- `tools`, and `motion {v_max, a_max}` (TCP, Cartesian).

Shared timing moves to `scara_defaults: {grip_s, release_s, settle_s, z_margin,
plan_speed_scale}`. In the base spec everything is null; dev values are PLACEHOLDERs:
reach 0.15–0.6 m, z_stroke 0.2 m, payload 3 kg, v_max 2 m/s, a_max 10 m/s², grip/release 0.2 s,
settle 0.1 s, z_margin 0.03 m, plan_speed_scale 0.5.

Six SCARAs: place_spider, place_mid_plate, install_standoffs, place_pod, fit_side_plates,
fit_top_plate. st_load_pallet stays `actor: sub_line`. st_drive_arm_sandwich and st_seat_esc are
gang-head only, and st_mount_servo stays parked.

A base stands at its station's x, `base_offset_y` out on `base_side`, on the datum plane.
`spec.scara_base()` is the only place that sum is done; the xacro, descriptions_check and
line_measure repeat it and the descriptions check compares them. Each base is on the side of its
part source. Dev `base_offset_y` is 0.55 m: the feeders and buffers stand 0.3 m out with
footprints up to 0.194 m, so a 0.2 m base fits only beyond them, and 0.55 m keeps the place pose
inside r_max. That puts every pick 0.25 m and every place 0.55 m from its base.

The D-002 `damping`/`friction` defaults went with `gantry_defaults`: no SCARA joint reads them.

**Decision — actors.** A station has at most one SCARA plus at most one gang head, and the gang
head must fasten what the SCARA placed (D-011's rule, with "gantry" read as "scara"). The actor
type `gantry` is now `scara`. A drive step's `head` must be a gang head; a SCARA no longer drives
screws. The actors fixtures were renamed to match.

**Decision — gang-head slide.** At a station with both a SCARA and a gang head (fit_side_plates,
fit_top_plate), the head rides a slide in from the side opposite the SCARA:
`gang_heads.<id>.slide {stroke, t_in, t_out}` (dev 0.25 m, 0.5 s, 0.5 s PLACEHOLDER). The sequence
is: SCARA places, SCARA clears (its return home), slide in, drive, slide out. `station_time` adds
`t_in + t_out` once per drive step. While the SCARA works, the head waits `slide.stroke` out on
the far side (`robot_geometry.head_box`).

**Decision — checks** (check_line.py; gantry_straddle, reach and gantry_load are removed):

| check | rule |
|---|---|
| `scara_reach` | FAIL if a pick pose (the step's feeder or buffer) or a place pose (the station) is outside [r_min, r_max] of its SCARA's base |
| `scara_clearance` | FAIL if a base footprint (`layout.scara_base.diameter`) overlaps the conveyor, another base, a feeder or a buffer. WARN if two reach discs overlap; the WARN names the shared zone and what stands in it (needs an interlock) |
| `safe_z` | per SCARA: tallest item in reach + the part it carries + z_margin. FAIL above `z_stroke` |
| `robot_load` | each SCARA's per-drone time at plan_speed_scale against takt − transfer |
| `station_time` | SCARA steps + drive_s × passes + slide t_in + t_out + index_s, against takt |
| `arm_pin_clearance` | FAIL if a tooling hole is within 2 × screw Ø of an arm screw hole, or there is not one tooling hole per arm |

"In reach" means any part of the item's footprint is within r_max of the base: a pallet block at
a station, a feeder footprint, or a gang head box. safe_z items:
- each station's stack;
- each feeder's or buffer's pick plane;
- each gang head body where it waits. That is `layout.gang_head_box`, its underside one ram
  stroke above the finished stack (on the stack for a horizontal head).

**Changed from D-009:** the SCARA's own station counts at its stack *before* its own place. The
part it brings in is the hang. Counting it twice (on the pallet and on the tool) was wrong, and
with one SCARA per station it would fail every place by the part's own height.

**Decision — the move model** (estimate_moves.py). One pick-and-place is:
1. horizontal move, home to pick (straight line);
2. Z down, grip, Z up;
3. horizontal move to the place;
4. Z down, release, Z up;
5. horizontal move back home.

Every move is a trapezoid on the TCP distance at `motion.v_max`/`a_max` × plan_speed_scale, plus
`settle_s` after each Z move. qty n is n full cycles. Home is the arm folded in to r_min, pointing
from the base at its station: the brief names a home and the spec has none, so this is the
tightest fold the reach allows. The table shows both speed columns, the budget and PASS/FAIL per
SCARA, then the station_time table.

**Decision — option B for the spider.** `place_spider` now takes `spider` from `buf_spider` with a
new tool, `spider_gripper` (finger, dev dims [0.20, 0.20, 0.02] PLACEHOLDER). It grips all 4 arms
in their X pattern and lifts them out of `carrier_spider`. The carrier stays in buf_spider and goes
back to sub-line A. That is `sources.src_A.returns: carrier_spider`, a no-op in every tool (the
schema only checks the part exists). seat_esc loses its "releases carrier_spider" note.
- **Arm pins.** On the pallet the arms are located by one pin each. The pin passes through a
  dedicated arm tooling hole in the bottom plate and goes `pin_engagement` into the arm. The arm
  screw holes cannot be used: they must stay free for gh_arm_sandwich's spindles.
- **Spec.** New section `drone: {arm_tooling_holes, tooling_pin_d, pin_engagement}`: null in base;
  dev is one hole per arm on the arm axis 48 mm from the plate centre (the arm root is at 45 mm),
  Ø3.0 and 2.0 mm, PLACEHOLDER. `fasteners.m3_x8.d: 0.003` is real: M3 is 3 mm by definition.
- **pallet.py** builds `Pin_Arm_1..4`, pressed through the body, up to the bottom plate thickness
  plus pin_engagement, and reports them under `derived.arm_pins`. Dev: the nearest screw is 7.0 mm
  away against the 6 mm limit.
- **spider_carrier.py.** When no step places `carrier_spider`, no main-line robot lifts the
  carrier. It still builds the pick boss but skips the tool and z-stroke checks, and warns.
  **Not modelled:** whether spider_gripper's fingers reach the arms past the carrier's bridge and
  ESC nest. The carrier was designed to be lifted off the spider, not to have the spider lifted
  out of it. tools.py builds spider_gripper as its generic two-jaw tool.

**Decision — end of line deferred.** st_audit_torque, st_test (5 cells), st_pack, their steps
(audit_torque, test, pack) and `layout.test_branch` (the test divert) moved verbatim, base and dev
values, to `docs/deferred_stations.yaml`. No tool loads that file. The line now ends at
st_fit_top_plate. None of the three drew from a feeder. `layout.test_branch` is optional in the
schema: layout.py still draws cells for any multi-cell station. **Pallet return path:** the spec
has no return-path entity to keep. st_load_pallet ("where returned pallets re-enter the line")
stays as it was.

**Decision — milestones.** Three milestones:
- `frame`: st_load_pallet, st_place_spider, st_place_mid_plate, st_drive_arm_sandwich;
- `electronics`: st_install_standoffs, st_seat_esc, st_place_pod;
- `closeout`: st_fit_side_plates, st_mount_servo, st_fit_top_plate.

spec.py checks that every station is in exactly one milestone and that each milestone is a
contiguous run in line order. Milestones are taken by their first station, so a YAML dump that
sorts keys still validates. `spec.milestone_subset()` cuts the spec down to one milestone: its
stations and their steps, SCARAs and gang heads, plus the feeders, buffers, fasteners and sources
those steps draw on. A `requires` on an upstream step is dropped. `check_line.py --milestone NAME`
writes `reports/line_check.<name>[.dev].json`. `layout.py --milestone NAME` draws on a conveyor
just long enough for those stations, into `60_layout/[dev/]<name>/`.

**Decision — plan_speed_scale needs re-deriving for a SCARA.** The dev 0.5 is D-007's gantry
finding (a 50 kg X axis lagged above half of a_max). It says nothing about a SCARA's arm, and
every time below scales with it. Re-measure it on a SCARA in Gazebo before any time is trusted.

**Consequence — descriptions.**
- **xacro.** `urdf/scara.urdf.xacro` replaces gantry.urdf.xacro. line.urdf.xacro instantiates it
  once per `scaras` entry as `Scara_<Name>`, with joints `scara_<station>_j1` (shoulder, revolute),
  `_j2` (elbow, revolute), `_j3` (Z, prismatic, value = TCP z) and `_j4` (wrist, revolute).
- **Links.** L1 = L2 = r_max/2, a PLACEHOLDER split.
- **Limits.** j1 and j4: ±π. j2: ±acos((r_min² − 2L²)/2L²), the fold that brings the TCP in to
  r_min. j3: [0, z_stroke]. Velocity is v_max on j3 and v_max/L on the revolute joints.
- **Controllers.** gen_controllers.py writes one `scara_<station>_controller` per SCARA
  (`scara_limits`, the same arithmetic).
- **World.** gen_world.py drops the `Rail_Gantry_<n>` envelopes: a SCARA base is a link of the robot.
- **line_measure.py** drives each SCARA to its station by two-link IK and compares Gazebo's pose
  of `Scara_<Name>_wrist`. The IK was checked against FK for the three dev stations. **It has not
  run in Gazebo:** line_smoke.sh was not run (headless rule: no Gazebo for this change).

**Consequence — layout.** Gantry solids are gone. New group `Robots`:
- `Scara_<Name>_Base`, a cylinder from the floor to `layout.scara_base.height` (new, dev 1.25 m);
- `GangHead_<Name>`, a `layout.gang_head_box` (new, dev [0.30, 0.30, 0.20]), drawn where
  robot_geometry puts it.

Reference holds:
- `Scara_<Name>_Envelope`, an annular cylinder r_min..r_max from the datum to z_stroke;
- `GangHead_<Name>_Slide`, waiting to working position;
- one `Guard_<Milestone>` per milestone around the union of its robots' envelopes plus
  guard_offset, with the conveyor tunnel.

Robots are teal, gang heads orange-brown, all opaque. A group with nothing in it is left out of
the STEP. For example, closeout draws from no buffer, so its STEP has no Buffers group.

**Result (dev, budget 17.2 s = 20.2 − (0.6/0.3 + 1.0)):**

| SCARA | step | @0.5 | @1.0 | load @0.5 | safe_z | status |
|---|---|---|---|---|---|---|
| scara_place_spider | place_spider | 3.34 s | 2.56 s | 19.4% | 0.160 | PASS |
| scara_place_mid_plate | place_mid_plate | 3.16 s | 2.44 s | 18.4% | 0.133 | PASS |
| scara_install_standoffs | install_standoffs | 3.33 s | 2.56 s | 19.4% | 0.192 | PASS |
| scara_place_pod | place_pod | 4.57 s | 3.34 s | 26.6% | **0.446** | **FAIL** (safe_z) |
| scara_fit_side_plates | place_side_plates | 3.31 s | 2.55 s | 19.3% | **0.203** | **FAIL** (safe_z) |
| scara_fit_top_plate | place_top_plate | 3.39 s | 2.60 s | 19.7% | 0.170 | PASS |

Every station fits takt. The busiest is st_fit_side_plates at 12.31 s of 20.2 s: pair pick 3.31,
plus 2 drive passes (6.0), plus slide 1.0, plus index 2.0. scara_reach passes.
scara_clearance passes with **no WARN**: neighbouring SCARAs sit on opposite sides (bases 1.253 m
apart against 1.2 m of r_max) or on the same side at 1.2 m, where the discs touch without
overlapping. `check_line.py --dev --milestone frame` passes. `layout.py --dev --milestone frame`
has 0 interferences.

**Unresolved (not tuned away):**
- **E. safe_z, scara_place_pod 0.446 > 0.2.** gh_side_plates waits on its slide 0.25 m to −y of
  st_fit_side_plates (scara_fit_side_plates is on +y). Its box (x 4.05–4.35, y −0.40..−0.10, top
  0.337) is 0.474 m from scara_place_pod's base at (3.6, −0.55), inside r_max. Fixes:
  - put place_pod's SCARA on +y, which needs buf_pod moved to +y;
  - slide gh_side_plates vertically instead;
  - leave a free station between st_place_pod and st_fit_side_plates.
- **F. safe_z, scara_fit_side_plates 0.203 > 0.2.** The side-plate magazine's pick plane is 0.17
  (the 218 mm magazine), plus the 3 mm plate, plus 0.03 margin. Fixes: a z_stroke of at least
  0.21 m, or a shorter fd_side_plate.
- **G. layout.py --dev: 2 interferences,** Guard_Frame ∩ Guard_Electronics (30.8e6 mm³) and
  Guard_Electronics ∩ Guard_Closeout (24.6e6 mm³). With r_max 0.6 and guard_offset 0.3,
  neighbouring milestones' guard walls run along the same y lines and overlap in x. Fix: neighbours
  share one partition wall at the boundary (the gang head between them), instead of each having
  its own closed box. That is a layout.py change to how a guard is drawn; it needs the user's call.

**Open gaps** (carried):
- **A.** Who works st_audit_torque and st_pack: deferred with them.
- **B.** No fastener is chosen for seat_esc, fit_side_plates or drive_top_plate (WARN).
- **C.** mount_servo (parked) has no driver.

## D-013 — Five SCARA stations, Epson LS10-B, anchors, path-based safe_z, carrier returns, fence

**Date:** 2026-10-02 (docs/12_line_architecture_update.md is the source of every decision here)

**Context.** D-012 left six single-step SCARA stations, a per-SCARA reach/stroke typed into every
entry, absolute poses for every station and source, safe_z judged against everything in a reach
circle (items E and F failed on equipment the SCARA never passes over), and per-milestone guard
boxes that overlapped (item G). docs/12 fixes the station plan (loader + A–E), the robot (one
Epson model everywhere) and the spider handling (the carrier rides along to B).

**Decision — five working stations, not four, not six.** `st_build_frame` (A) merges
st_place_spider, st_place_mid_plate and st_drive_arm_sandwich: `extend_pins` (actuate) →
`place_spider` (carrier_spider from buf_spider, tool_carrier_spider) → `place_mid_plate`
(tool_plate_vacuum) → `drive_arm_sandwich` (gh_arm_sandwich on a slide, 8). `st_seat_esc` (B):
`install_standoffs` (one gang pick) → `drive_esc` (gh_esc, 4) → `remove_carrier_spider` onto
`rt_carrier_spider`. `st_fit_pod` (C): `place_pod` (carrier_pod) → `drive_pod` (gh_pod) →
`remove_carrier_pod` onto `rt_carrier_pod`. `st_fit_side_plates` (D): paired pick → opposed
heads, 2 passes → `mount_servo` (parked, no actor). `st_fit_top_plate` (E): place → drive 5.
Milestones: frame [st_load_pallet, st_build_frame], electronics [st_seat_esc, st_fit_pod],
closeout [st_fit_side_plates, st_fit_top_plate].
**Why not four:** the first merge put ESC and pod at one station, but it missed three steps of the
design summary §5 — the pod screws, the spider-carrier removal and the pod-tray removal. With them
a combined ESC + pod station is ~24.5 s, over the 20.2 s takt. With real SCARA speeds B and C may
merge again; it is a spec edit and `station_time` will say whether it fits.

**Decision — the carrier rides along to B.** carrier_spider is placed at A (it holds the 4 arms
and the suspended ESC), stays on the pallet, and is lifted off at B once the ESC is fastened. The
pod tray is placed and lifted off at C. New capability **`remove`** (a SCARA lifts `part` off the
pallet and puts it on return path `to`); `robot_geometry.stack_parts` drops a removed part, so
heights downstream are right. **Return paths** `return_paths.rt_<carrier>`: a chute (or conveyor
segment) owned by its station, from the put-down point (`offset`) `length` out through the fence
to `sink`, the sub-line source that refills the carrier. D-012's option B is gone: no
spider_gripper, no `sources.*.returns` no-op; `drone.arm_tooling_holes` and `arm_pin_clearance`
stay. The pallet's arm pins are retractable: `actuate` step `extend_pins` (device arm_pins) at A,
`drone.arm_pins.retract_at: unload` (the end-of-line station, deferred).

**Decision — Epson LS10-B, 700 mm, Z 300 mm.** `robot_models.epson_ls10b_700_z300` carries the
published values (reach 0.70, z_stroke 0.30, payload 5/10 kg, j1 ±132°, j2 ±150°, j4 ±360°,
inertia_rated 0.02 kg·m², repeatability 0.02 mm, the vendor STEP path). `link1`/`link2` and
`motion {v_max, a_max, j4_speed}` are null in base, PLACEHOLDER in dev (0.425/0.275 m, 2 m/s,
10 m/s², 20 rad/s). Every `scaras` entry names the model; tools are listed once under `tools:`
(grip, dims, mass, offset_r) and a SCARA lists its end-effector: one tool or a **dual-tool** pair
(A: carrier gripper + plate vacuum; B: standoff gang + carrier gripper), switched by a 180° J4
turn timed π / (j4_speed × plan_speed_scale), counted in series (conservative: a real cell turns
J4 during the travel). **Open item — wrist inertia:** docs/12 §4; the check exists (below) but
its inputs are placeholders until the carrier and end-effector are modelled.
**The vendor STEP measures link1/link2 = 0.325/0.375 m** (J2 bore and Z shaft axes,
tools/cad/vendor.py), not the dev placeholder 0.425/0.275. layout.py WARNs about it; the
datasheet value should replace both.

**Decision — anchors.** A station is `anchor: {after: <previous station>, gap}` (the first:
`after: origin`); everything a station owns — SCARA base, gang head working pose, feeders,
buffers, return paths — is `offset: [dx, dy, dz, yaw]` in that station's frame. spec.py
`resolve_anchors` derives every `pose` (idempotent; a milestone subset keeps the poses it was cut
with and records `pallet_in`, what the pallet brings from upstream); every generator reads only
resolved poses. A `pose` written next to an anchor/offset is refused at load. A resolved pose
component computed from a placeholder is itself listed as a placeholder (`derived_from`). New
check `anchors`: FAIL a station without an anchor, an anchor not on the previous station, or an
owned entity with an absolute pose.

**Decision — path-based safe_z.** A SCARA clears only what lies under the band its tool (and the
part it carries) sweeps: each leg home → pick → place → home is a joint-space move (j1, j2
interpolated on the elbow solution valid at both ends), sampled, buffered by the tool's radius
(offset_r + half-diagonal) or the carried part's; a step that starts with a tool switch also
clears the end-effector's disc at home. Per step: tallest item under the band + the carried part's
height + z_margin; estimate_moves times each step at its own. FAIL above the model's stroke top
(base z + z_stroke). This retires D-012 items E and F.

**Decision — checks added/changed:** `scara_reach` (IK inside the links and the j1/j2 limits,
j1 measured from the base yaw — not a full circle — and z inside the stroke; radius-only and
UNRESOLVED while the links are null), `payload` (tool masses + heaviest lifted part with what rides
in it ≤ payload_rated), `wrist_inertia` (Σ(I_cm + m r²) about J4 for the tools at offset_r and the
carried part as a plate of its plan size; WARN above inertia_rated, UNRESOLVED on a null mass),
`station_time` and `robot_load` (tool switches and `actuate` durations included).

**Decision — guards.** One continuous fence guard_offset outside everything worked at (reach
discs, gang heads and slides, feeders, buffers, return-path put-down ends), tunnelled where the
conveyor passes each end and where a return chute leaves; one partition wall per milestone
boundary, half-way between the boundary stations, tunnelled for the conveyor. The sinks stand
outside. All guard solids stay in the Reference group. This retires D-012 item G.

**Decision — layout.** Each SCARA is its vendor STEP posed folded (j1 = 0, j2 at its limit) on a
`layout.scara_base` pedestal (falls back to the cylinder with a WARN when the STEP is missing);
carriers are drawn on the pallet at every station they ride through; return paths and sinks are
drawn. layout.py renders reports/render/<milestone|line>_{top,iso}.png with pyvista (offscreen).

**Dev layout (PLACEHOLDER).** Every working station is one cell: SCARA base 0.45 m out on −y
facing the line, sources and the return chute 0.35 m either side in x, 0.25 m out on −y, the gang
head waiting on its slide (0.40 m) on +y. Gaps 1.4 m inside a milestone (no shared reach zone),
2.0 m across a milestone boundary (the partition clears both envelopes by guard_offset). The
slide stroke was sized from safe_z: the spider carrier's band reaches y = 0.21 m on +y, so a
0.15 m half-width head must wait ≥ 0.36 m out (at 0.35 m safe_z needed 0.49 m > 0.30).

**Layout memory (found while verifying).** build123d's `moved()` / `Location * shape` deep-copy
the shape first; for a shape already in an assembly that copies the whole tree. With the vendor
arms that ran the full-line layout past 7.8 GB (twice, taking the session with it). layout.py
`relocated()` and vendor.py now place shapes at a new OCC location over the same geometry (each
arm keeps its own copy, ~15 MB, so STEP re-import keeps every name). Frame milestone: 4.5 GB →
1.0 GB, ~2 min → ~25 s; full line: 1.7 GB, ~2 min.

**Result on the dev overlay (PLACEHOLDER values, not deliverable).** `check_line.py --dev` PASS
on the full line and on each milestone; `estimate_moves.py --dev` PASS; `descriptions.sh --dev`
PASS (26 joint limits, 25 spindle frames); 438 pytest pass.

| station | SCARA steps (+ switches) | drive | slide | actuate | index | total / 20.2 s |
|---|---|---|---|---|---|---|
| st_build_frame | 7.97 | 3.0 | 1.0 | 0.5 | 2.0 | **14.47** |
| st_seat_esc | 8.13 | 3.0 | 1.0 | — | 2.0 | **14.13** |
| st_fit_pod | 7.79 | 3.0 | 1.0 | — | 2.0 | **13.79** |
| st_fit_side_plates | 3.53 | 6.0 (2 passes) | 1.0 | — | 2.0 | **12.53** |
| st_fit_top_plate | 3.38 | 3.0 | 1.0 | — | 2.0 | **9.38** |

robot_load (budget 17.2 s): 46.3 / 47.2 / 45.3 / 20.5 / 19.6 %. safe_z (stroke 0.30):
0.235 / 0.240 / 0.246 / 0.203 / 0.133 m. payload (rated 5 kg): 2.10 / 1.75 / 0.92 / 0.52 /
0.89 kg. wrist_inertia (rated 0.02): 0.0112 / 0.0091 / 0.0009 / 0.0014 / 0.0045 kg·m² — no WARN,
on placeholder masses. layout.py --dev: 108 solids, 0 interferences, STEP hierarchy and opacity
verified.

**Unresolved (not tuned away):**
- **H. The parked arm at A meets gh_arm_sandwich.** Folded at j1 = 0 (the home the estimate
  uses), A's link 1 (underside 0.321 m) and J2 housing overlap the head's working box (top
  0.354 m: it works over the 0.104 m carrier) by 128 + 14 cm³. B–E clear. The layout draws the
  head retracted, so `interference` cannot see it. Fixes: park A's arm folded away from the
  pallet (e.g. j1 ≈ ±120°) while the head drives, raise A's mount, or add a check of the parked
  arm against each head's working box.
- **I. Link split.** Dev link1/link2 0.425/0.275 m vs 0.325/0.375 m measured in the vendor
  STEP: reach, r_min and the IK use the placeholder. Put the datasheet value in the base spec.
- **J. Gang head supports.** Heads are drawn at their waiting pose with no slide frame under
  them; the frame is not modelled.
- Gap B (no fastener for drive_esc, drive_pod, drive_side_plates, drive_top_plate) and gap C
  (mount_servo parked) carried; the spider-carrier bridge vs the mid plate and the arm-sandwich
  head (docs/12 §7) is not modelled; plan_speed_scale 0.5 still the D-007 linear-axis value.

## D-014 — Park pose, real links, slide stroke and frames, per-station index, Epson calibration

**Date:** 2026-10-02

**Context.** D-013 left three issues open: H (the folded arm at A met gh_arm_sandwich), I (the
link split was a placeholder) and J (no slide frame under any head). Two timing inputs were also
weak. The index was one fixed pitch / belt_speed for every station. The SCARA speed was D-007's
gantry `plan_speed_scale`.

**Decision — park pose + `park_clearance` (fixes H).** Each SCARA has `park_pose {j1, j2}` (rad;
null in base). While its station's gang head is in its working position, the SCARA holds that
pose with the TCP at the top of its stroke.
- **Timing.** The SCARA step before a drive step ends at park, and the next step starts there,
  read cyclically (`robot_geometry.park_steps`). place → park and park → pick replace the two
  home legs, so the park move is in `station_time`. Both legs also count in path-based safe_z.
- **Dev values.** `tools/choose_park.py --dev --write` folds j2 to its limit on the place pose's
  elbow side. It searches j1 over ±132° in 0.5° steps for the value nearest the place j1 that
  clears the working envelope by ≥ 0.03 m. The working envelope is the head at its working
  position plus its slide frame. Results are tagged `# PLACEHOLDER (auto-chosen)`.
- **New check `park_clearance`.** It models the arm at park_pose as capsules sized from the
  vendor STEP:
  - link1 (J1 → J2), radius 60.6 mm, z +21..+88 mm above the TCP;
  - link2 (J2 → shaft), radius 80.5 mm, z +47..+321 mm;
  - the shaft, radius 21 mm;
  - the end-effector disc (`end_effector_radius`), down by its longest tool reach.

  The capsules are measured by `tools/cad/vendor.py --model <id>`, which writes
  `reports/vendor_<id>.json`; layout.py rewrites it on every run. Posing the STEP inside
  check_line was rejected: about 0.5 GB and 10 s per load. The check FAILs on any overlap or a
  park pose outside the joint limits, and reports the clearance per station.
- **Cross-check.** layout.py poses the real STEP at park_pose and intersects it with the same
  envelope (`layout.json park`). Any overlap is a layout FAIL. It also renders
  `reports/render/<station>_park.png`.

**Decision — real link lengths (fixes I).** `link1 = 0.325`, `link2 = 0.375` in the base spec,
"measured from Epson STEP LS10-B703S". The dev placeholder (0.425 / 0.275) and layout.py's
mismatch warning are gone. The xacro, the IK, r_min and the reach checks now use the real split.

**Decision — slide stroke as a requirement.** `gh_arm_sandwich.slide.stroke: 0.40` moves from the
overlay to the base spec as a requirement: the carrier must clear the parked head at
st_build_frame (measured need ≥ 0.36 m, D-013). The other heads keep their 0.40 m PLACEHOLDER.
New check **`slide_clearance`**, run for every head on a slide:
- it takes the swept band of each part the station's SCARA carries (the part's radius plus the
  tool's offset_r);
- along the carrying leg, the band spans from the part's bottom at the step's safe_z down to the
  pick or place at the two ends;
- that band must clear the head waiting retracted by ≥ 0.02 m (3D: plan gap, vertical gap, or
  both).

**Decision — slide frames (fixes J).** `gang_heads.<id>.slide.frame {post: [x, y], rail: [w, h]}`
is null in base; dev uses post 0.12 × 0.12 and rail 0.10 × 0.08 (PLACEHOLDER). The frame is
placed from the head's working pose, so it is anchored to the station.
- **Rail.** It runs along the slide axis from the head's outer face at the working position to
  the far face of the post. It is centred on the head's top face, so the head's carriage runs in
  it.
- **Post.** It stands beyond the retracted head, from the floor to the rail top.

layout.py draws `GangHead_<Name>_Rail` and `GangHead_<Name>_Post` in Robots (category
`slide_frame`) and adds them to the fence's extent. They are in the interference check, which
allows only head-on-rail and rail-on-post, and only within the same head
(`ALLOWED_SAME_OWNER`, `Item.owner`). The rail starts at the head's outer face rather than over
the pallet: a rail over the pallet would sit in link2's z band (0.347..0.621 m) where the arm
reaches in to place.

**Decision — per-station index.**
- **Queue slots.** `layout.buffer_slots_between_stations` is replaced by `layout.slot_gap`
  (null; dev 0.05). Slots fill each gap at pitch p = pallet length + slot_gap:
  floor(gap / p) − 1 of them, leaving one pitch before the next station (`spec.queue_gaps`).
- **Index.** A station's index (`spec.station_index`) is the distance the pallet travels to
  enter it (from the last slot, or the previous station when there is none) / belt_speed +
  stop_unit_cycle_s.
- **Edge cases.** The line's first station is entered from an in-feed slot one pitch upstream:
  the pallet return's queue position, which the spec does not lay out. A milestone subset
  records the station upstream of it (`entry`), so its first station keeps the real gap.
- **Where it is used.** `station_time` uses this per-station index. `robot_load` compares each
  SCARA with takt − its station's index. The old pitch / belt_speed is reported as `idx_old`;
  it never included the stop unit.

**Decision — Epson standard-cycle calibration, replacing `plan_speed_scale`.**
- **Standard cycle.** `robot_models.<id>.standard_cycle {time_s 0.41, horizontal_m 0.300,
  vertical_m 0.025, payload_kg 2.0, motion round_trip_arch}` is in the base spec ("Epson LS10-B
  700 mm spec sheet").
- **Calibration.** `estimate_moves.py --calibrate` keeps the a_max / v_max ratio and solves the
  one factor at which the move model covers that round trip in 0.41 s. The round trip is up, across,
  down, and back; grip and settle are excluded. The ratio kept is the earlier dev 10 / 2.
- **Result.** v_max 14.2089 m/s, a_max 71.0444 m/s² (×7.10), stored in the overlay as
  "PLACEHOLDER (derived from standard_cycle)". The ratio is a placeholder, so they are too.
  Every run recomputes the standard-cycle time at the stored values and FAILs when it misses
  0.41 s by more than 1 ms.
- **Derate.** `scara_defaults.cycle_derate` (null; dev 0.5 PLACEHOLDER, "vendor standard cycle is
  speed-optimised; replace with a measured ratio"). Moves and the J4 switch run at the
  calibrated values × cycle_derate. `plan_speed_scale` is gone from the schema, spec, tools and
  tests.
- **Payload.** A WARN marks every step whose payload exceeds standard_cycle.payload_kg, because
  its times are optimistic there. The payload counts every tool on the end-effector plus the part
  and what rides in it.

**Result on the dev overlay (PLACEHOLDER values, not deliverable).** pytest: 457 passed.
`check_line.py --dev` PASS on the full line and on each milestone. `estimate_moves.py --dev`
PASS. `layout.py --dev`: 138 solids, 0 interferences, STEP hierarchy and opacity verified.
`descriptions.sh --dev` PASS (26 joint limits, 25 spindle frames).

| station | SCARA (+ switches, park) | drive | slide | actuate | index old | index new | total | D-013 total |
|---|---|---|---|---|---|---|---|---|
| st_build_frame | 4.46 | 3.0 | 1.0 | 0.5 | 2.00 | 1.83 | **10.79** | 14.47 |
| st_seat_esc | 4.46 | 3.0 | 1.0 | — | 2.00 | 1.83 | **10.29** | 14.13 |
| st_fit_pod | 3.93 | 3.0 | 1.0 | — | 2.00 | 2.33 | **10.27** | 13.79 |
| st_fit_side_plates | 1.86 | 6.0 | 1.0 | — | 2.00 | 1.83 | **10.69** | 12.53 |
| st_fit_top_plate | 1.82 | 3.0 | 1.0 | — | 2.00 | 2.33 | **8.15** | 9.38 |

Queue slots: 3 / 7 / 4 / 7 / 4 per gap (gaps 1.0 / 2.0 / 1.4 / 2.0 / 1.4 m). The pallet enters
over 0.25 m from a slot (0.40 m after a 1.4 m gap), so the index is 1.83 or 2.33 s.

The other dev results:

| station | park pose j1 | park_clearance | slide_clearance | robot_load (takt − index) |
|---|---|---|---|---|
| st_build_frame | +46° | 32 mm (link2 to head) | 40 mm | 24.3% |
| st_seat_esc | −52.5° | 30 mm (tools to head) | 40 mm | 24.3% |
| st_fit_pod | −55° | 52 mm | 180 mm | 22.0% |
| st_fit_side_plates | −55° | 55 mm | 194 mm | 10.1% |
| st_fit_top_plate | −46° | 30 mm | 73 mm | 10.2% |

Every park pose folds j2 to +150°. layout.py's STEP overlap at every park pose is 0 mm³. A parks
on the far side: folding on the place side puts its 0.135 m dual-tool disc over the head. The one
payload WARN is scara_build_frame place_spider: 2.10 kg (tools 1.65 + carrier and spider 0.45) >
2.0 kg.

**Why the times fell ~3.5 s.** The calibrated motion at ×0.5 (7.1 m/s, 35.5 m/s²) is far faster
than D-013's 1 m/s, 5 m/s². A pick-and-place is now ~1.8–2.3 s, against ~3.3 s before.

**Unresolved (not tuned away):**
- **K. The calibrated v_max is not physical.** With the dev ratio kept, every leg of the
  standard cycle is triangular, so the 0.41 s fixes only a_max; v_max comes out at 14.2 m/s. The
  vendor's arch motion blends the vertical and horizontal moves, and the model runs them in
  series, so it puts all of that blending on acceleration. Fix: take the real ratio (or a
  measured v_max) from Epson and re-run `--calibrate`. Replace cycle_derate with a measured ratio.
- **L. Park legs are timed on TCP distance.** A swings j1 by 101° (−55° → +46°) while its TCP
  moves ~0.4 m, and the model times the TCP's straight line. The real move is likely slower.
- **M. The slide frame is checked only against the parked arm and the folded layout pose,** not
  along the place paths. With the rail outboard, the arm's reach to the pallet stays clear in
  plan, but no check proves it.
- Wrist inertia, the spider-carrier bridge vs the mid plate and head, gap B (fasteners for
  drive_esc / drive_pod / drive_side_plates / drive_top_plate) and gap C (mount_servo parked) are
  carried.

---

## D-015 — Batch 1: real SCARA speed, arc park timing, path and refill checks, pallet 0.32

**Date:** 2026-10-06

**Context.** D-014 left K (calibrated v_max 14.2 m/s, not physical), L (park legs timed on the
TCP's straight line) and M (slide frames not checked along the arm's paths) open. This batch
takes them on for st_load_pallet, st_build_frame (A) and st_seat_esc (B) only, and sizes the
pallet for a 7" drone.

**Decision — station batches (`--stations`).** `check_line.py`, `estimate_moves.py`,
`layout.py` and `choose_park.py` take `--stations a,b,...`:
- `spec.station_subset` cuts the spec to a run of consecutive stations. It keeps their SCARAs,
  heads, feeders, buffers, return paths and sources. `milestone_subset` is now the same cut,
  by name.
- The first station keeps its real upstream gap (`entry`), so its index is unchanged.
- Each milestone keeps its stations in the run, so a partition inside the batch is still drawn.
- A run that is not consecutive, or names an unknown station, is a SpecError.
- Reports are `<tool>.<--name>[.dev].json`; layout writes `60_layout/dev/batch/`.

**Decision — real SCARA speed (fixes K).** In the base spec, "Epson LS10-B 700 spec":
- `robot_models.epson_ls10b_700_z300.motion.v_max: 9.8` m/s (J1+J2 max operating speed);
- `inertia_max: 0.3` kg·m².

`--calibrate` now keeps v_max and solves a_max only. Result: **a_max = 71.0444 m/s²**, unchanged
from D-014. At 9.8 m/s no leg of the standard cycle reaches v_max (v²/a = 1.35 m, longer than the
0.30 m horizontal leg), so the 0.41 s fixes a alone. a_max stays a PLACEHOLDER in the overlay:
the series move model is the assumption in it. `cycle_derate` 0.5 is kept (PLACEHOLDER), so moves
plan at 4.9 m/s and 35.5 m/s².

**Decision — wrist inertia limits.** `wrist_inertia` WARNs above inertia_rated (J4 must slow
down) and FAILs above inertia_max. A null inertia_max is UNRESOLVED. A: 0.0112, B: 0.0091 kg·m²,
both under 0.02.

**Decision — park legs in joint space (fixes L).** place → park and park → pick are now J1/J2
rotations:
- Each joint runs its own trapezoid. Angular limits are v_max / a_max × derate at the joint's link
  radius: link1 + link2 (0.70 m) for J1, link2 (0.375 m) for J2. At ×0.5: J1 7.0 rad/s,
  50.7 rad/s²; J2 13.1 rad/s, 94.7 rad/s².
- The slower joint sets the time.
- The pick/place end takes its IK solution on the park pose's elbow side with the least j1
  travel, the same rule as the paths.

The other legs stay straight-line TCP trapezoids.

**Decision — new check `path_clearance` (fixes M).** It covers every leg of every SCARA step:
start → pick, pick → place, place → end (home or park), and the dual-tool switch at the step's
start.
- Each leg is sampled along its joint path (48 steps).
- The arm is its capsules (link1, link2, shaft, tools disc). It is checked against every head at
  the station, waiting retracted on its slide, and that head's rail and post.
- TCP z at each sample:
  - the step's safe_z;
  - the end sample also goes down to the pick or place z;
  - on a park leg, Z is interpolated with J1/J2 from safe_z to the top of the stroke.
- The links do not move in z: only the shaft and tools follow the TCP (`arm_capsules(tcp=)`).
- 3D clearance; FAIL on contact or under 0.02 m.

The first version held a park leg at safe_z..top along the whole leg. That put the tools beside
the rail at the place point and failed E at 15.1 mm. A point-to-point move interpolates Z with
the joints, as the plan path already does, so the leg was corrected; the threshold is unchanged.

**Decision — `park_clearance` in plan.** The parked arm must clear the working head and its
frame in XY. Height separation is only extra margin, reported as `clearance_3d_m`. FAIL on any
plan overlap. The fixture shows the difference: link2 2 mm above a head but 30 mm over it in
plan now FAILs.

**Decision — new check `refill_clearance`.**
- **Envelope.** A buffer at a SCARA's station that a source refills has a refill envelope: the
  column over its footprint, from its pick plane up.
- **Rule.** The parked arm, and its legs into and out of park, must not cross it.
- **Exception.** A leg out of park that picks from that buffer ends in it by design. It is listed
  as `excluded`: the refill must finish first, which is an interlock, not geometry.
- **Not covered.** Operator-refilled magazines (`feeders`) are not covered.

**Decision — park re-chosen for A and B.**
- `choose_park` now requires all of:
  - plan park clearance ≥ 0.03 m;
  - parked refill clearance ≥ 0.03 m;
  - refill paths ≥ 0;
  - path_clearance ≥ 0.02 m.
- Search order: j2 at its limit first (D-014). Only if no pose works there is j2 eased in 5°
  steps; the most folded level that works wins.
- **A.** No folded pose works:
  - with the elbow on the −x side, the elbow stands over `buf_spider` at the place pose;
  - with the elbow on the +x side, the TCP sweeps across `buf_spider` on its way to a −x park;
  - a +x park folded to 150° brings the tool disc back within 30 mm of the head in plan.

  A now parks at **j1 −130°, j2 130°** (fold eased 20°) on the fd_mid_plate side.
- **B.** Unchanged: j1 −52.5°, j2 150°.

**Decision — pallet 0.32 m.** `layout.pallet_block.length/width: 0.32` (PLACEHOLDER: "7-inch
footprint ~244 mm + carrier/gripper/pin margin; final from pallet.py"). The slot pitch follows:
0.37 m.
- **Fewer slots.** The 1.0 m gap into A holds 1 slot (was 3), entered over 0.63 m. The 2.0 m gap
  into B holds 4 (was 7), entered over 0.52 m.
- **Layout interferences.** layout.py then found two:
  - **Pallet lane.** The pallets leaving A ran 7 mm into `fd_mid_plate` (magazine edge at
    y −0.153, pallet edge at −0.16).
  - **Partition.** A queued pallet hit the partition: the guard tunnels were cut conveyor-wide
    (0.30 m), narrower than the pallet.
- **Fixes.**
  - Tunnels are now cut to the wider of belt and pallet.
  - A's and B's sources (`fd_mid_plate`, `buf_spider`, `fd_standoff`, PLACEHOLDER) moved from
    y −0.25 to −0.27: 13–20 mm clear of the pallet lane.

**Result on the dev overlay (PLACEHOLDER values, not deliverable).** pytest: 478 passed.
`check_line.py --dev` PASS on the full line. `--stations st_load_pallet,st_build_frame,
st_seat_esc` PASS. `estimate_moves.py --dev --stations ...` PASS. `layout.py --dev --stations
...`: 58 solids, 0 interferences, STEP hierarchy and opacity verified, 0 mm³ STEP overlap at
both park poses.

| station | SCARA | drive | slide | pins | index | total | % takt | D-014 total |
|---|---|---|---|---|---|---|---|---|
| st_load_pallet | — (load 6.00, PH) | — | — | — | 2.23 | 8.23 | 40.8% | — |
| st_build_frame | 4.79 | 3.00 | 1.00 | 0.50 | 3.10 | **12.39** | 61.3% | 10.79 |
| st_seat_esc | 4.46 | 3.00 | 1.00 | — | 2.73 | **11.20** | 55.4% | 10.29 |

- **Index** (+1.27 s at A, +0.90 s at B): the wider pallet leaves fewer, coarser slots, so the
  pallet enters over a longer distance.
- **A's SCARA** (+0.32 s): its park legs swing j1 75° (place → park, 0.33 s) and 131°
  (park → buf_spider, 0.46 s, J1-limited).
- **B's park legs** move j2 50° each (0.19 s).
- The loader's time is its `load_pallet` duration (PLACEHOLDER) plus its index; station_time
  has no row for it.

| | A | B |
|---|---|---|
| park pose | −130° / 130° | −52.5° / 150° |
| park_clearance (plan) | 32 mm (tools to head) | 30 mm (tools to head) |
| path_clearance | 70 mm (link2 to rail, place_spider) | 47 mm (tools to rail, park → pick) |
| slide_clearance | 40 mm | 40 mm |
| refill_clearance (buf_spider) | parked 215 mm, path 181 mm | — (no refilled buffer) |
| wrist inertia (rated 0.02 / max 0.3) | 0.0112 | 0.0091 |

**Unresolved (not tuned away):**
- **Parked arms over operator-refilled sources.** The park renders show A's link2 over
  `fd_mid_plate` and B's over `fd_standoff` in plan. refill_clearance covers only buffers a
  source refills. If an operator refills a magazine during the drive, the arm is over their
  hands. Fix: give magazines a refill envelope too, or refill only behind a light curtain with
  the SCARA locked out.
- **C–E keep their D-014 park poses.** They pass, but E's plan park clearance is 12 mm, below
  choose_park's 30 mm. Their overlay comments still quote D-014's 3D values. Their sources are
  still at 0.25 m: `fd_top_plate` at E will run 7 mm into the 0.32 m pallet lane, as
  fd_mid_plate did. Fix: `choose_park.py --dev --write --stations st_fit_pod,st_fit_side_plates,
  st_fit_top_plate`, and move their sources to 0.27 m, in the next batch.
- **Per-joint speed limits.** J1/J2 limits come from v_max at the link radius (the brief's rule).
  Epson's per-joint maximum speeds, if they are lower, would slow the park legs. Take them from
  the Epson spec sheet.
- **Payload.** A's place_spider carries 2.10 kg, over the standard cycle's 2.0 kg (WARN, carried
  from D-014).
- Gap B (fasteners for drive_esc etc.) and gap C are carried.

## D-016 — Batch 2: loader row, magazine refill zones, C–E sources and parks

**Date:** 2026-10-07

**Context.** D-015 left four items open: the parked arms over operator-refilled magazines, C–E
not re-run for the 0.32 m pallet, Epson's per-joint speeds, and no station_time row for the
loader. This batch covers st_fit_pod (C), st_fit_side_plates (D) and st_fit_top_plate (E). The
magazine rule is also applied back to A and B.

**Decision — loader row (`load_s`).** `estimate_moves.station_times` (and so the station_time
check) counts the `duration_s` of a `load` step that no SCARA serves as `load_s`. That step is
the single-purpose loader's. The loader now has its own row: load 6.00 (PLACEHOLDER) + index
2.23 = **8.23 s, 40.8 %**, the same as D-015's hand total. A null duration_s is UNRESOLVED. A
`load` step that a SCARA serves (the test fixture's st_load) is timed as a SCARA step, as before.

**Decision — refill envelopes cover operator-refilled magazines.**
- **Which sources.** `feeders` of type `magazine` or `nest` (`OPERATOR_REFILLED`) at the
  SCARA's station, beside D-015's source-refilled buffers. At A that is fd_mid_plate, at B
  fd_standoff, at D fd_side_plate and at E fd_top_plate. C has none (buf_pod, a buffer, was
  already covered). Blow feeders are not refilled by hand.
- **The envelope.** The open-topped column over the footprint, as in D-015.
- **Plan, at every height (judgement call).** The clearance is now in plan for every refill
  envelope, buffers included. D-015 used plan above the pick plane and 3D below it. Below the
  pick plane the source's own body fills the column, so height never separates an arm from it.
  The D-015 buffer numbers are unchanged (A buf_spider parked 215 mm, path 181 mm).
- **The head too (judgement call).** The station's gang head must stay out of every column:
  - in its working position, retracted on its slide, and its rail and post;
  - plan gap of the box to the footprint, FAIL on overlap.
  All five stations clear by 105–168 mm.
- **Exclusion, narrowed (judgement call).** D-015 dropped a park → pick leg from every envelope
  when it picked from one of them. Now the leg is excluded only from the envelope it picks from,
  so a leg into fd_mid_plate must still keep out of buf_spider's column. No station's worst
  value comes from such a leg.
- **Report shape.** `refill_clearance` reports `envelopes: {id: {kind, parked, path, head}}`
  (was `buffers`) and `head_clearance_m`.

**Decision — j2 is never eased for a magazine.** `choose_park` still eases the fold only when no
fully folded pose works (D-015). The first fold level where a pose meets every rule *except* the
magazine ones is the last level tried. If no pose there clears the magazines too, it returns an
error naming the SCARA and the magazine (`magazine_blocked`), and nothing is written. That
station keeps its old park pose and fails `refill_clearance`.

**Decision — C–E sources at y −0.27 m (PLACEHOLDER).** buf_pod, fd_side_plate and fd_top_plate
moved from −0.25 m to −0.27 m, as A and B did in D-015. Clearance to the 0.32 m pallet lane:
- fd_top_plate: **13 mm** (≥ 10 mm required; was −7 mm at −0.25);
- buf_pod: 20 mm;
- fd_side_plate: 78 mm.

The return chutes (rt_carrier_spider, rt_carrier_pod) stay at −0.25 m. Their footprint starts
0.25 m out, 90 mm clear of the lane, and D-015 did not move rt_carrier_spider either.

**Decision — parks re-chosen** (`choose_park.py --dev --write --stations st_fit_pod,
st_fit_side_plates,st_fit_top_plate`, then `--stations st_seat_esc`, because the magazine rule
now applies to B).

| SCARA | park pose (was) | plan | result |
|---|---|---|---|
| A scara_build_frame | −130° / 130° (unchanged) | 32 mm | **blocked**: no pose clears fd_mid_plate |
| B scara_seat_esc | **127.5° / −150°** (−52.5° / 150°) | 31 mm | the old pose had link2 78 mm into fd_standoff's column |
| C scara_fit_pod | −55° / 150° (unchanged) | 43 mm | D-014's 52 mm was 3D |
| D scara_fit_side_plates | **55° / −150°** (−55° / 150°) | 43 mm | the old pose had link2 54 mm into fd_side_plate's column |
| E scara_fit_top_plate | −46° / 150° (unchanged) | **12 mm** | **blocked**: no pose clears fd_top_plate |

- **A — stopped.** At the 20° eased fold, the most folded level that meets the other rules, the
  parked tools disc stands 130.8 mm inside fd_mid_plate's column. That level is fixed by
  buf_spider (D-015). Diagnostic only, not applied: with easing allowed for the magazine too, the
  chooser still finds no pose at any fold. Every pose that clears one of buf_spider and
  fd_mid_plate stands over the other, or its park legs cross it. A needs a layout change, not a
  park pose.
- **E — stopped.** At the full fold, all 346 plan-clear poses either park over fd_top_plate or
  sweep link2 across its column on place → park (nearest: j1 46°, leg 80.5 mm into it). Two
  diagnostics, not applied:
  - **Without the magazine rule:** the chooser reaches **j1 46°, j2 150°, 31 mm in plan** (vs the
    12 mm kept). This answers the brief's "what the chooser achieves" for the head alone.
  - **With easing allowed:** j1 130°, j2 −130° (20° eased), 32 mm plan, refill 172 mm.

  E keeps its D-014 pose (12 mm in plan, below the 30 mm target, still ≥ 0 so park_clearance
  passes), and its overlay comment now says so.
- **Seen in the renders, not a rule.** B's and C's parked link2 lie over their return chutes in
  plan (0 mm³ overlap in the STEP). A chute is not a refill zone: nothing drops onto it while
  the SCARA is parked.

**Decision — Epson per-joint speeds: assumption kept.** Neither the repo nor the docs give
LS10-B J1 or J2 maximum speeds separately:
- docs/12 §4 quotes only Epson's product pages;
- the spec carries only "J1+J2 max operating speed" 9.8 m/s;
- cad_raw holds only the STEP.

So the J1/J2 limits stay derived (D-015): v_max / a_max × derate at the joint's link radius.
- J1: 7.0 rad/s, 50.7 rad/s².
- J2: 13.1 rad/s, 94.7 rad/s².

**Assumption (explicit):** Epson's separate J1/J2 maximum speeds, which the datasheet gives in
°/s, are no lower than these. If they are lower, every J1-limited park leg gets slower. Those
are A's, B's 72° legs, and D's and E's park → pick legs. Replace the derivation when the
datasheet is in the repo.

**Result on the dev overlay (PLACEHOLDER values, not deliverable).**
- **pytest:** 482 passed.
- **check_line.py --dev:** FAILs on the full line, with **4 findings, all refill_clearance at A
  and E** (parked and place → park each). Every other check passes or WARNs as in D-015.
- **Batch and milestones.** `--stations st_fit_pod,st_fit_side_plates,st_fit_top_plate` FAILs on
  E's two refill findings only. The frame milestone FAILs on A's, closeout on E's, and
  electronics passes.
- **estimate_moves.py --dev --stations ...:** PASS.
- **layout.py --dev --stations ...:** 70 solids, 0 interferences, STEP verified, 0 mm³ at all
  three parks (renders `reports/render/st_fit_*_park.png`). B was re-rendered from an A+B batch,
  then the C–E batch was re-run so `60_layout/dev/batch/` is batch 2.

| station | SCARA | drive | slide | pins | load | index | total | % takt | D-015 total |
|---|---|---|---|---|---|---|---|---|---|
| st_load_pallet | — | — | — | — | 6.00 | 2.23 | 8.23 | 40.8% | 8.23 (by hand) |
| st_build_frame | 4.79 | 3.00 | 1.00 | 0.50 | — | 3.10 | 12.39 | 61.3% | 12.39 |
| st_seat_esc | 4.72 | 3.00 | 1.00 | — | — | 2.73 | **11.45** | 56.7% | 11.20 |
| st_fit_pod | 3.94 | 3.00 | 1.00 | — | — | 3.20 | **11.14** | 55.2% | 11.14 |
| st_fit_side_plates | 1.92 | 6.00 | 1.00 | — | — | 2.73 | **11.65** | 57.7% | 11.67 |
| st_fit_top_plate | 1.92 | 3.00 | 1.00 | — | — | 3.20 | **9.12** | 45.1% | 9.10 |

- **B (+0.25 s).** The new park is 72° of j1 from the place pose, so both park legs are
  J1-limited at 0.32 s (were 0.19 s, j2-limited).
- **D (−0.02 s) and E (+0.01 s).** fd_side_plate and fd_top_plate moved 20 mm out, and D's park
  flipped elbow side.

| | C | D | E |
|---|---|---|---|
| park pose | −55° / 150° | 55° / −150° | −46° / 150° (kept) |
| park_clearance (plan / 3D) | 43 / 52 mm (link2 to head) | 43 / 55 mm (link2 to head) | **12** / 30 mm (link2 to head) |
| path_clearance | 70 mm (link2 to rail, place_pod) | 62 mm (tools to rail, place → park) | 70 mm (link2 to rail, place → park) |
| slide_clearance | 180 mm | 194 mm | 73 mm |
| refill parked / path | buf_pod 72 / 71 mm | fd_side_plate 94 / 92 mm | fd_top_plate **−61 / −81 mm** |
| refill head | 114 mm | 168 mm | 106 mm |
| wrist inertia (rated 0.02 / max 0.3) | 0.00085 | 0.00144 | 0.00448 |

A and B under the new rule: A fd_mid_plate parked −131 mm (FAIL), buf_spider 215 / 181 mm. B
fd_standoff 215 / 196 mm.

**Test fixture.** pass_spec.yaml's fd_mid_plate footprint was 0.15 m and is now 0.12 m. With the
gang head moved to st_place (`_slide_at_place`), the fixture's park pose stood 7.7 mm into the
magazine's column, so the fixture no longer passed. Moving the magazine instead broke a dozen
other fixture cases.

**Unresolved (not tuned away):**
- **A: fd_mid_plate vs buf_spider.** No park pose clears both. Options:
  - move fd_mid_plate or buf_spider (e.g. both to one side of the SCARA, or the magazine
    further out in x);
  - make the mid-plate magazine a drawer refilled from outside the guard;
  - refill only behind a light curtain with the SCARA locked out.
- **E: fd_top_plate.** No fully folded pose clears it. Options: move the magazine (or mirror it
  to −x), or accept a 20° eased fold (needs the user's approval: the brief forbids easing j2 for
  the magazine). E's plan park clearance stays at 12 mm.
- **Epson J1/J2 speeds:** the assumption above.
- Carried: A's payload 2.10 kg over the standard cycle's 2.0 kg (WARN), gaps B and C.

## D-017 — MuJoCo replaces Gazebo as the sim backend (M1: look and move)

**Date:** 2026-11-13

**Context.** Gazebo (gz-harmonic, D-005..D-007) needed hand-tuned workarounds for the line's
robots: position mode cannot lift a vertical axis under gravity, moves are planned at half speed,
and the world is a Jinja template that duplicates the layout. The next milestones need three
things it does poorly: an interactive editor whose edits go back to the spec (M2), fast exact
kinematics for the SCARAs and gang heads, and a contact-rich physics cell for the pick and place
at one station.

**Decision.**
- **MuJoCo is the sim backend:** for the editor (M2 writes viewer edits back through spec-ID body
  names), for kinematics, and later for the physics cell.
- **Renders come from Blender**, not from the simulator. The MuJoCo offscreen PNGs in verify/
  are only for checking.
- **ROS 2 comes later**, through mujoco_ros2_control (`ros-jazzy-mujoco-ros2-control` 0.1.2,
  built on `ros-jazzy-mujoco-vendor` with MuJoCo 3.12.0). Until then the generated MJCF stays
  inside MuJoCo 3.12's MJCF (`tools/checks/mjcf_gate_312.sh`).
- **Gazebo code is kept as reference** — gen_world.py, gen_models.py, templates/, launch/,
  urdf/, tools/checks/*smoke* — untouched and no longer maintained.
- **One layout implementation.** `tools/gen_mjcf.py` builds the MJCF with MjSpec from
  `tools/cad/layout.py`'s `plan()` solids, `robot_geometry.kin()` and the vendor arm capsules. It
  adds no placement of its own. Bodies with a spec ID (st_, scara_, gh_, fd_, buf_, rt_) sit at
  their resolved spec pose. Everything else is named after its layout solid, and those names
  are not stable.
- **The MJCF is generated** (drone_line_sim/mjcf/line[.dev].xml, header "GENERATED — do not
  edit"). Dev output is gitignored like the dev SDF. The base spec cannot be exported until its
  nulls are filled: the run exits 1 and lists them.
- **tools/check_mjcf_match.py** is the gate between the two. It compares every spec-ID body,
  site and layout solid's AABB (1 mm / 0.1°) and the overlap and bbox pair lists, which must be
  equal to `layout.interference` with the same ALLOWED_OVERLAPS. It does this at the `folded`
  keyframe and at each `parked_<scara>` keyframe. Pairs inside one assembly (one SCARA, one
  carrier) are not compared: their convex proxies overlap where the STEP solids do not.
- **Kinematic, not physical, for now:** gravity off, `implicitfast` integrator (Euler diverges
  on the light J4 shaft under the actuator damping), PLACEHOLDER actuator gains, joint
  stiffness/damping unset.

**Status (M1).**
- Loader + station A: match PASS.
- Full line: the MJCF matches every pose, AABB and overlap set. The check's second attempt was
  flaky: its shape cache reused shapes across parked samples. That is a check bug, not an MJCF
  mismatch, and its fix waits on the user (KNOWN_ISSUES).
- Headless physics runs at 30–120× real time.
- The viewer's 4.9 % real time was rendering on WSL's D3D12 layer, with one physics step per
  frame. `tools/view_mjcf.py` steps physics to the wall clock instead.
