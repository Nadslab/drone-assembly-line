#!/usr/bin/env bash
# P5.4 check: the whole line launches headless, every controller comes up, and SCARAs
# reach three stations.
#   1. regenerate models, world, controllers and the merged spec, and rebuild drone_line_sim
#      (the launch reads the *installed* package, so it must never run against a stale one)
#   2. launch drone_line_sim line.launch.py gui:=false, on a private ROS domain and gz partition
#   3. tools/checks/line_measure.py: every controller active <= 30 s from launch, then SCARAs
#      visit three stations, TCP error < 1 mm against Gazebo's own link pose
#   4. write the report, kill everything the launch started, exit non-zero on failure
#
# Runs against the dev overlay by default: the base spec's limits and poses are still null. A dev
# run writes reports/line_smoke.dev.json and --real writes reports/line_smoke.json, so a run on
# placeholders can never be mistaken for one on real values (CLAUDE.md P0).
#   tools/checks/line_smoke.sh [--real] [--tol-mm F]
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$(cd "$REPO/../.." && pwd)"
PY="$REPO/tools/.venv/bin/python"
REPORT_DIR="$REPO/reports"
CONTROLLER_TIMEOUT_S="${LINE_CONTROLLER_TIMEOUT_S:-30}"

DEV="--dev"; SUFFIX=".dev"; DEV_ARG="true"
TOL_MM=1.0
while (( $# )); do
  case "$1" in
    --real) DEV=""; SUFFIX=""; DEV_ARG="false"; shift ;;
    --tol-mm) TOL_MM="$2"; shift 2 ;;
    -h|--help) sed -n '2,15p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "line_smoke: unknown argument $1" >&2; exit 2 ;;
  esac
done
REPORT="$REPORT_DIR/line_smoke${SUFFIX}.json"
LOG="$REPORT_DIR/line_smoke_launch.log"
SPEC_MERGED="$REPO/drone_line_sim/config/line_spec.merged.dev.yaml"
CONTROLLERS="$REPO/drone_line_sim/config/controllers${SUFFIX}.yaml"
mkdir -p "$REPORT_DIR"
rm -f "$REPORT"

T_START=$(date +%s.%N)
elapsed() { "$PY" -c "import sys; print(round($(date +%s.%N) - $T_START, 2))"; }

fail() {
  # fail <stage> <message>
  "$PY" - "$REPORT" "$1" "$2" "$(elapsed)" "$DEV_ARG" <<'PYEOF'
import json, sys
out, stage, msg, t, dev = sys.argv[1:6]
json.dump({'pass': False, 'dev': dev == 'true', 'stage': stage, 'errors': [msg],
           'elapsed_s': float(t), 'launch_log': 'reports/line_smoke_launch.log'},
          open(out, 'w'), indent=2)
PYEOF
  echo "line_smoke: FAIL [$1] $2 (see $REPORT, $LOG)" >&2
  exit 1
}

# Isolate from other ROS / Gazebo sessions: a stale controller_manager on the same domain would
# answer for ours.
export ROS_DOMAIN_ID=$(( 100 + $$ % 32 ))
export GZ_PARTITION="line_smoke_$$"
SID_FILE="$(mktemp)"

LAUNCH_SID=""
cleanup() {
  [[ -z "$LAUNCH_SID" ]] && LAUNCH_SID=$(cat "$SID_FILE" 2>/dev/null)
  if [[ -n "$LAUNCH_SID" ]]; then
    # The launch runs in its own session, so everything it started shares its session id.
    pkill -INT -s "$LAUNCH_SID" 2>/dev/null
    for _ in $(seq 1 25); do pgrep -s "$LAUNCH_SID" >/dev/null || break; sleep 0.2; done
    pkill -KILL -s "$LAUNCH_SID" 2>/dev/null
  fi
  rm -f "$SID_FILE"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# --- 1. regenerate and rebuild ---------------------------------------------------------------
: >"$LOG"
for gen in gen_models gen_world gen_controllers; do
  "$PY" "$REPO/tools/$gen.py" $DEV >>"$LOG" 2>&1 || fail generate "$gen.py failed (see $LOG)"
done
if [[ -n "$DEV" ]]; then
  "$PY" "$REPO/tools/spec.py" --dev --dump "$SPEC_MERGED" >>"$LOG" 2>&1 \
    || fail generate "spec.py --dump failed (see $LOG)"
fi

# ROS setup scripts reference unset variables.
set +u
source /opt/ros/jazzy/setup.bash
( cd "$WS" && colcon build --packages-select drone_line_sim >>"$LOG" 2>&1 ) \
  || fail build "colcon build drone_line_sim failed (see $LOG)"
source "$WS/install/setup.bash"
set -u

# --- 2. launch, headless ---------------------------------------------------------------------
T_LAUNCH=$(date +%s.%N)
# setsid may fork, so $! is not reliably the session id; record it from inside the session.
setsid bash -c 'echo $$ >"$0"; exec ros2 launch drone_line_sim line.launch.py gui:=false dev:="$1"' \
  "$SID_FILE" "$DEV_ARG" >>"$LOG" 2>&1 &
for _ in $(seq 1 50); do [[ -s "$SID_FILE" ]] && break; sleep 0.1; done
LAUNCH_SID=$(cat "$SID_FILE")
[[ -n "$LAUNCH_SID" ]] || fail launch "could not start ros2 launch"
echo "line_smoke: launch session $LAUNCH_SID, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# --- 3. controllers, then the SCARAs ---------------------------------------------------------
# --spec is the merged overlay spec for a dev run, the base spec otherwise.
SPEC_ARG="$SPEC_MERGED"; [[ -z "$DEV" ]] && SPEC_ARG="$REPO/drone_line_sim/config/line_spec.yaml"
EXTRA=$("$PY" -c "import json; print(json.dumps({'dev': '$DEV_ARG' == 'true'}))")
python3 "$REPO/tools/checks/line_measure.py" --out "$REPORT" --spec "$SPEC_ARG" \
  --controllers "$CONTROLLERS" --tol-mm "$TOL_MM" --controller-timeout "$CONTROLLER_TIMEOUT_S" \
  --launch-start "$T_LAUNCH" --extra "$EXTRA"
rc=$?
[[ -f "$REPORT" ]] || fail measure "line_measure.py exited $rc without writing a report"

"$PY" - "$REPORT" "$(elapsed)" <<'PYEOF'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r['elapsed_s'] = float(sys.argv[2])
r['launch_log'] = 'reports/line_smoke_launch.log'
json.dump(r, open(p, 'w'), indent=2)
PYEOF

if (( rc == 0 )); then
  echo "line_smoke: PASS in $(elapsed) s ($REPORT)"
else
  echo "line_smoke: FAIL (see $REPORT, $LOG)" >&2
fi
exit "$rc"
