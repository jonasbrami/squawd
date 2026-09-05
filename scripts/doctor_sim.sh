#!/usr/bin/env bash
# doctor_sim.sh — preflight checks gating run_single_demo.sh (design §13 item 1).
# Every check is hard-deadlined; FAIL prints a legible reason and exits 1.
# Run INSIDE the container via run_single_demo.sh, or standalone for diagnosis.
set -uo pipefail

FAILS=0
ok()   { echo "  PASS  $1"; }
bad()  { echo "  FAIL  $1"; FAILS=$((FAILS+1)); }
warn() { echo "  WARN  $1"; }

echo "doctor_sim: preflight for the single-drone sim"

PY=python3
command -v uv >/dev/null 2>&1 && PY="uv run --no-project python"

# 1. gz + ROS + project deps in the python env the agents will actually use
$PY - <<'EOF' && ok "python deps (mavsdk, claude_agent_sdk, openai_codex, rclpy, px4_msgs)" || bad "python deps missing (mavsdk/claude_agent_sdk/openai_codex/rclpy/px4_msgs)"
import importlib.util, sys
sys.exit(0 if all(importlib.util.find_spec(m) for m in
    ("mavsdk", "claude_agent_sdk", "openai_codex", "rclpy", "px4_msgs")) else 1)
EOF

# 1b. Backend credentials and runtime contract (presence only; never print data)
case "${SQUAWD_BACKEND:-claude}" in
  codex)
    [ -f "${CODEX_HOME:-/root/.codex}/auth.json" ] \
      && ok "Codex subscription login present (model=${SQUAWD_MODEL:-gpt-5.6-terra}, effort=${SQUAWD_CODEX_EFFORT:-low})" \
      || bad "Codex auth missing at CODEX_HOME/auth.json"
    ;;
  kimi)
    [ -n "${KIMI_API_KEY:-}" ] && ok "Kimi subscription key present" \
      || bad "KIMI_API_KEY missing"
    command -v claude >/dev/null 2>&1 && ok "external Claude CLI present for Kimi route" \
      || bad "external Claude CLI missing for Kimi route"
    ;;
  claude)
    [ -f /root/.claude/.credentials.json ] && ok "Claude OAuth login present" \
      || bad "Claude OAuth credentials missing"
    ;;
  *) bad "invalid SQUAWD_BACKEND=${SQUAWD_BACKEND}" ;;
esac

# 2. ROS_DOMAIN_ID set (avoid cross-talk with stray ROS graphs)
if [ -n "${ROS_DOMAIN_ID:-}" ]; then ok "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
else warn "ROS_DOMAIN_ID unset (default domain 0 — ok on a dedicated box)"; fi

# 3. uXRCE-DDS agent alive (px4 <-> ROS2 bridge)
if pgrep -fa "MicroXRCEAgent" >/dev/null 2>&1; then ok "uXRCE-DDS agent running"
else bad "uXRCE-DDS agent NOT running (MicroXRCEAgent)"; fi

# 4. PX4 instance 0 publishing telemetry (hard 20s deadline)
n=0
for _ in $(seq 1 20); do
  n=$(timeout 2 ros2 topic list 2>/dev/null | grep -c "vehicle_local_position" || true)
  [ "$n" -ge 1 ] && break
  sleep 1
done
if [ "$n" -ge 1 ]; then ok "PX4 instance 0 telemetry on the bus"
else bad "no vehicle_local_position after 20s (PX4 SITL down?)"; fi

# 5. Gazebo alive: world topics present
if timeout 5 gz topic -l 2>/dev/null | grep -q "/world/"; then ok "gazebo topics present"
else bad "gazebo not answering (gz topic -l shows no /world/)"; fi

# 6. Drone model + camera topic (hard 20s deadline — sensors patch in late)
c=0
for _ in $(seq 1 20); do
  c=$(timeout 3 gz topic -l 2>/dev/null | grep -c "IMX214/image" || true)
  [ "$c" -ge 1 ] && break
  sleep 1
done
if [ "$c" -ge 1 ]; then ok "camera sensor topic publishing ($c)"
else bad "no IMX214/image topic after 20s (x500_depth model/camera missing)"; fi

# 7. Camera actually streaming, not just advertised (8s deadline)
if timeout 8 gz topic -e -n 1 --count 1 "$(timeout 3 gz topic -l 2>/dev/null | grep 'IMX214/image' | head -1)" >/dev/null 2>&1; then
  ok "camera frames flowing"
else
  warn "camera topic advertised but no frame in 8s (renderer still warming?)"
fi

if [ "$FAILS" -gt 0 ]; then
  echo "doctor_sim: $FAILS FAIL(S) — refusing to start."
  exit 1
fi
echo "doctor_sim: all checks pass."
