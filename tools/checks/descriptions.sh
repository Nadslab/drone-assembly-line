#!/usr/bin/env bash
# P5.3 check: the robot descriptions expand, parse, and say what the spec says.
#   1. regenerate config/controllers.yaml and the merged spec (never pass on a stale file)
#   2. xacro every top-level urdf/*.urdf.xacro, then check_urdf on the result
#   3. compare every joint limit, spindle frame and mount pose with the spec
#   4. write reports/descriptions.json, exit non-zero on failure
#
# Headless and offline: no Gazebo, no controller manager — this is the description alone.
# Everything runs against the dev overlay by default (the base spec's limits are still null, so
# --dev is what there is to check). Pass --real to run it against line_spec.yaml alone.
#   tools/checks/descriptions.sh [--dev | --real]
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$(cd "$REPO/../.." && pwd)"
PY="$REPO/tools/.venv/bin/python"

DEV="--dev"
while (( $# )); do
  case "$1" in
    --real) DEV=""; shift ;;
    --dev) DEV="--dev"; shift ;;
    -h|--help) sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "descriptions: unknown argument $1" >&2; exit 2 ;;
  esac
done

# xacro and check_urdf are ROS tools; the description itself needs nothing else from the
# workspace, but $(find drone_line_sim) in a default arg does, so source the overlay too when it
# is built. ROS setup scripts reference unset variables.
set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
set -u

exec "$PY" "$REPO/tools/checks/descriptions_check.py" ${DEV:+$DEV}
