#!/usr/bin/env bash
# M1 gate: generated MJCF must also load in MuJoCo 3.12.0, the version ros-jazzy-mujoco-vendor
# (and so mujoco_ros2_control) bundles. tools/.venv runs 4.0.1; this builds a throwaway venv at
# /tmp, loads each file (load only, no stepping), and deletes the venv again.
#   tools/checks/mjcf_gate_312.sh FILE.xml [FILE.xml ...]
# Prints one line per file and exits non-zero when any fails to load.
set -uo pipefail

VER=3.12.0
VENV="$(mktemp -d /tmp/mj312.XXXXXX)"
trap 'rm -rf "$VENV"' EXIT

if (( $# == 0 )); then
  echo "mjcf_gate_312: no MJCF file given" >&2
  exit 2
fi
python3 -m venv "$VENV" || exit 2
"$VENV/bin/pip" install -q "mujoco==$VER" || { echo "mjcf_gate_312: cannot install mujoco $VER" >&2; exit 2; }

"$VENV/bin/python" -I - "$@" <<'EOF'
import sys
import mujoco
bad = 0
for path in sys.argv[1:]:
    try:
        m = mujoco.MjModel.from_xml_path(path)
        print(f"mjcf_gate_312: OK mujoco {mujoco.__version__} {path} "
              f"({m.nbody} bodies, {m.ngeom} geoms, {m.njnt} joints)")
    except Exception as exc:            # a parse or compile error names the attribute
        bad += 1
        print(f"mjcf_gate_312: FAIL mujoco {mujoco.__version__} {path}: {exc}")
sys.exit(1 if bad else 0)
EOF
