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
