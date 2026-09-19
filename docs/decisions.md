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
