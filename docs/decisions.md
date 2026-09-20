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

