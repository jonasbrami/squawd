#!/usr/bin/env bash
# Host-side sweep driver: run a tier sweep in its sim container, and when the
# in-run infra fuse aborts (unrecoverable PX4 state — e.g. a drone parked away
# from home that RTL/ferry can't move), ESCALATE TO A FRESH SIM: restart the
# container, wait for PX4, resume (results.jsonl resume skips scored cells).
# Bounded: max $MAX_ROUNDS restarts, per-round timeout $ROUND_TIMEOUT_S.
#
# usage: drive_sweep.sh <container> <tier> <out_subdir> <seed> <task...>
set -u
CONTAINER=$1; TIER=$2; OUT=$3; SEED=$4; shift 4
TASKS="$*"
MAX_ROUNDS=${MAX_ROUNDS:-5}
GZ_WORLD=${GZ_WORLD:-default}   # export GZ_WORLD=dynamic for the mover ladder
ROUND_TIMEOUT_S=${ROUND_TIMEOUT_S:-5400}

wait_ready() {
  for _ in $(seq 1 30); do
    n=$(docker exec "$CONTAINER" bash -lc 'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; timeout 10 ros2 topic list 2>/dev/null | grep -c vehicle_local_position' 2>/dev/null || echo 0)
    [ "$n" -ge 1 ] && return 0
    sleep 15
  done
  return 1
}

for round in $(seq 1 "$MAX_ROUNDS"); do
  echo "=== round $round/$MAX_ROUNDS ($TIER) ==="
  timeout "$ROUND_TIMEOUT_S" docker exec "$CONTAINER" bash -lc \
    "source /opt/ros/jazzy/setup.bash; source /opt/px4_ws/install/setup.bash;
     cd /workspace && PYTHONPATH=/workspace:\$PYTHONPATH GZ_WORLD=$GZ_WORLD \
     uv run --no-project python -m evals.run_evals --tasks $TASKS \
       --assignments 'drones=$TIER' --k 2 --seed $SEED --out evals/out/$OUT" 2>&1
  # done when every expected cell is scored (run_evals skips-then-exits cleanly)
  remaining=$(docker exec "$CONTAINER" bash -lc \
    "cd /workspace && PYTHONPATH=/workspace uv run --no-project python - << 'PYEOF'
import json, os
p = 'evals/out/$OUT/results.jsonl'
rows = [json.loads(l) for l in open(p)] if os.path.exists(p) else []
done = {f\"{r['task_id']}|{r['repeat']}\" for r in rows if not r.get('infra_fail')}
print(len([1 for t in '$TASKS'.split() for k in (0,1)]) - len(done))
PYEOF" 2>/dev/null || echo 999)
  echo "round $round done; remaining cells: $remaining"
  if [ "$remaining" -le 0 ]; then echo "SWEEP COMPLETE ($TIER)"; exit 0; fi
  echo "escalating: restarting $CONTAINER for a fresh sim"
  docker restart "$CONTAINER" >/dev/null
  wait_ready || { echo "SIM NEVER READY after restart"; exit 1; }
done
echo "GAVE UP after $MAX_ROUNDS rounds ($TIER); resume manually"
exit 1
