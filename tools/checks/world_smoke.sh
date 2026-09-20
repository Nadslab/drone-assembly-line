#!/usr/bin/env bash
# P5 check: the generated world loads headless and every model is at its spec pose.
#   1. regenerate models + world (so the check never passes on a stale file)
#   2. start the gz server headless (-s -r) on a private partition
#   3. wait for it to list its models, then run tools/checks/world_measure.py:
#      every model in the gen_world manifest, compared with the spec, <= 1 mm
#   4. write reports/world.json, exit non-zero on failure, kill gz on exit
#
# Everything runs against the dev overlay by default: the base spec's poses are still null, so
# --dev is what there is to check. Pass --real to run it against line_spec.yaml alone.
#   tools/checks/world_smoke.sh [--real] [--iterations N] [--tol-mm F] [--keep]
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PY="$REPO/tools/.venv/bin/python"
REPORT_DIR="$REPO/reports"
REPORT="$REPORT_DIR/world.json"
LOG="$REPORT_DIR/world_smoke_gz.log"

DEV="--dev"
SUFFIX=".dev"
ITERATIONS=300          # 0.3 s of physics: long enough for anything unsupported to move
TOL_MM=1.0
SERVER_TIMEOUT_S=25     # the whole check has a 30 s budget
KEEP=0

while (( $# )); do
  case "$1" in
    --real) DEV=""; SUFFIX=""; shift ;;
    --iterations) ITERATIONS="$2"; shift 2 ;;
    --tol-mm) TOL_MM="$2"; shift 2 ;;
    --keep) KEEP=1; shift ;;
    -h|--help) sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "world_smoke: unknown argument $1" >&2; exit 2 ;;
  esac
done

WORLD_NAME="main_line"          # matches tools/gen_world.py
MODELS="$REPO/drone_line_sim/models${DEV:+/dev}"
WORLD="$REPO/drone_line_sim/worlds/main_line${SUFFIX}.sdf"
MANIFEST="$REPORT_DIR/gen_world${SUFFIX}.json"
mkdir -p "$REPORT_DIR"
rm -f "$REPORT"

T_START=$(date +%s.%N)
elapsed() { "$PY" -c "import sys; print(round($(date +%s.%N) - $T_START, 2))"; }

fail() {
  # fail <stage> <message>
  "$PY" - "$REPORT" "$1" "$2" "$(elapsed)" <<'EOF'
import json, sys
out, stage, msg, t = sys.argv[1:5]
json.dump({'passed': False, 'stage': stage, 'errors': [msg], 'elapsed_s': float(t),
           'gz_log': 'reports/world_smoke_gz.log'}, open(out, 'w'), indent=2)
EOF
  echo "world_smoke: FAIL [$1] $2 (see $REPORT, $LOG)" >&2
  exit 1
}

# Isolate from any other Gazebo on this machine: a stale server would answer `gz model` for us.
export GZ_PARTITION="world_smoke_$$"
export GZ_SIM_RESOURCE_PATH="$MODELS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

GZ_PID=""
cleanup() {
  if [[ -n "$GZ_PID" ]]; then
    kill -INT "$GZ_PID" 2>/dev/null
    for _ in $(seq 1 25); do kill -0 "$GZ_PID" 2>/dev/null || break; sleep 0.2; done
    kill -KILL "$GZ_PID" 2>/dev/null
    wait "$GZ_PID" 2>/dev/null
  fi
  (( KEEP )) || rm -f "$LOG.tmp"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# --- 1. regenerate, so the check is never run against a stale world -------------------------
"$PY" "$REPO/tools/gen_models.py" $DEV >>"$LOG" 2>&1 \
  || fail generate "gen_models.py failed (see $LOG)"
"$PY" "$REPO/tools/gen_world.py" $DEV >>"$LOG" 2>&1 \
  || fail generate "gen_world.py failed (see $LOG)"
[[ -f "$WORLD" ]] || fail generate "no world at $WORLD"
[[ -f "$MANIFEST" ]] || fail generate "no manifest at $MANIFEST"

# --- 2. start the server headless -----------------------------------------------------------
# It runs open-ended rather than with --iterations: the server exits when those are done, and
# the poses have to be read out of a *running* world. Step 3 waits for the iterations instead.
: >"$LOG"
gz sim -s -r -v 2 "$WORLD" >>"$LOG" 2>&1 &
GZ_PID=$!

# --- 3. wait for the world to load and to step, then measure --------------------------------
READY=0
for _ in $(seq 1 $(( SERVER_TIMEOUT_S * 5 ))); do
  if gz model --list 2>/dev/null | grep -q '^\s*-\s'; then READY=1; break; fi
  kill -0 "$GZ_PID" 2>/dev/null || break
  sleep 0.2
done
if (( ! READY )); then
  kill -0 "$GZ_PID" 2>/dev/null \
    && fail load "the server did not list any model within ${SERVER_TIMEOUT_S} s" \
    || fail load "gz sim exited without loading the world (a missing model or mesh?)"
fi

# Physics has to have actually run: a model that is going to fall has to have had time to.
STEPPED=0
for _ in $(seq 1 $(( SERVER_TIMEOUT_S * 5 ))); do
  ITERS=$(timeout 5 gz topic -e -t "/world/${WORLD_NAME}/stats" -n 1 2>/dev/null \
          | sed -n 's/^iterations: *//p' | head -1)
  if [[ -n "$ITERS" ]] && (( ITERS >= ITERATIONS )); then STEPPED=1; break; fi
  kill -0 "$GZ_PID" 2>/dev/null || break
  sleep 0.2
done
(( STEPPED )) || fail step "the world did not reach ${ITERATIONS} iterations within ${SERVER_TIMEOUT_S} s"

"$PY" "$REPO/tools/checks/world_measure.py" --manifest "$MANIFEST" --report "$REPORT" \
  --iterations "$ITERATIONS" --tol-mm "$TOL_MM" --elapsed-s "$(elapsed)" $DEV
rc=$?
[[ -f "$REPORT" ]] || fail measure "world_measure.py exited $rc without writing a report"

"$PY" - "$REPORT" "$(elapsed)" <<'EOF'
import json, sys
p = sys.argv[1]
doc = json.load(open(p))
doc['elapsed_s'] = float(sys.argv[2])
doc['gz_log'] = 'reports/world_smoke_gz.log'
json.dump(doc, open(p, 'w'), indent=2)
EOF

if (( rc == 0 )); then
  echo "world_smoke: PASS in $(elapsed) s ($REPORT)"
else
  echo "world_smoke: FAIL (see $REPORT, $LOG)" >&2
fi
exit "$rc"
