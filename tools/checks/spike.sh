#!/usr/bin/env bash
# P1 check: one-axis ros2_control spike, headless.
#   1. launch drone_line_sim spike.launch.py (gui:=false)
#   2. wait <= 30 s (from launch) for joint_state_broadcaster + joint_trajectory_controller
#      to be active
#   3. send a 0.5 m trajectory, measure overshoot/settling from /joint_states
#   4. write reports/spike.json, exit non-zero on failure, kill gz on exit
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$(cd "$REPO/../.." && pwd)"
REPORT_DIR="$REPO/reports"
REPORT="$REPORT_DIR/spike.json"
LOG="$REPORT_DIR/spike_launch.log"
CONTROLLER_TIMEOUT_S="${SPIKE_CONTROLLER_TIMEOUT_S:-30}"
mkdir -p "$REPORT_DIR"
rm -f "$REPORT"

# Isolate from other ROS / Gazebo sessions on this machine: a stale controller_manager on the
# same domain would otherwise answer for ours.
export ROS_DOMAIN_ID=$(( 100 + $$ % 32 ))
export GZ_PARTITION="spike_$$"
SID_FILE="$(mktemp)"

# ROS setup scripts reference unset variables.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

T_START=$(date +%s.%N)
elapsed() { python3 -c "import sys; print(round($(date +%s.%N) - $T_START, 2))"; }

fail() {
  # fail <stage> <message>
  python3 - "$REPORT" "$1" "$2" "$(elapsed)" <<'EOF'
import json, sys
out, stage, msg, t = sys.argv[1:5]
json.dump({'pass': False, 'stage': stage, 'errors': [msg], 'elapsed_s': float(t),
           'launch_log': 'reports/spike_launch.log'}, open(out, 'w'), indent=2)
EOF
  echo "spike: FAIL [$1] $2 (see $REPORT, $LOG)" >&2
  exit 1
}

LAUNCH_SID=""
cleanup() {
  [[ -z "$LAUNCH_SID" ]] && LAUNCH_SID=$(cat "$SID_FILE" 2>/dev/null)
  if [[ -n "$LAUNCH_SID" ]]; then
    # The launch runs in its own session, so everything it started - gz sim server, bridge,
    # spawners - shares session id $LAUNCH_SID.
    pkill -INT -s "$LAUNCH_SID" 2>/dev/null
    for _ in $(seq 1 25); do
      pgrep -s "$LAUNCH_SID" >/dev/null || break
      sleep 0.2
    done
    pkill -KILL -s "$LAUNCH_SID" 2>/dev/null
  fi
  rm -f "$SID_FILE"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# setsid may fork, so $! is not reliably the session id; record it from inside the session.
setsid bash -c 'echo $$ >"$0"; exec ros2 launch drone_line_sim spike.launch.py gui:=false' \
  "$SID_FILE" >"$LOG" 2>&1 &
for _ in $(seq 1 50); do [[ -s "$SID_FILE" ]] && break; sleep 0.1; done
LAUNCH_SID=$(cat "$SID_FILE")
[[ -n "$LAUNCH_SID" ]] || fail launch "could not start ros2 launch"
echo "spike: launch session $LAUNCH_SID, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# --- wait for controllers, then step response ----------------------------------------------
# The controller wait queries /controller_manager/list_controllers (the service behind
# `ros2 control list_controllers`) from rclpy; ros2controlcli is not a dependency.
# rclpy lives on the ROS system python, not tools/.venv.
python3 "$REPO/tools/checks/spike_measure.py" --out "$REPORT" --distance 0.5 \
  --controller-timeout "$CONTROLLER_TIMEOUT_S" --launch-start "$T_START"
rc=$?
[[ -f "$REPORT" ]] || fail measure "spike_measure.py exited $rc without writing a report"

python3 - "$REPORT" "$(elapsed)" <<'EOF'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r['elapsed_s'] = float(sys.argv[2])
r['launch_log'] = 'reports/spike_launch.log'
json.dump(r, open(p, 'w'), indent=2)
EOF

if (( rc == 0 )); then
  echo "spike: PASS ($REPORT)"
else
  echo "spike: FAIL (see $REPORT, $LOG)" >&2
fi
exit "$rc"
