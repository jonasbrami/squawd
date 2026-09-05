#!/usr/bin/env bash
# deep_perception.sh — host GPU sidecar launcher (deep-perception plan §Ops).
# Runs agents.vision.deep.service inside .venv-train-gpu (the `deep` extra),
# bound to the discovered docker0 gateway with DEEP_TOKEN bearer auth
# (codex R5). The container reaches it as host.docker.internal:8100.
#
# Usage:  ./scripts/deep_perception.sh             # serve on :8100
#         ./scripts/deep_perception.sh --selftest  # health-check a RUNNING
#                                                  # service (exits non-zero
#                                                  # when unreachable)
# M1a: the selftest is a health-check stub — the real detect+segment smoke on
# recorded frames lands in M1b (plan §Testing: live smoke gate for M2/M4).
set -eo pipefail
cd "$(dirname "$0")/.."

TOKEN_FILE=.deep_token
if [ ! -s "$TOKEN_FILE" ]; then
  openssl rand -hex 24 > "$TOKEN_FILE"
  chmod 600 "$TOKEN_FILE"
  echo "[deep] generated $TOKEN_FILE (gitignored)"
fi
export DEEP_TOKEN="$(cat "$TOKEN_FILE")"

if [ "${1:-}" = "--selftest" ]; then
  HOST_ADDR="$(ip -4 route show dev docker0 2>/dev/null | sed -n 's/.*src \([0-9.]*\).*/\1/p')"
  URL="${DEEP_PERCEPTION_URL:-http://${HOST_ADDR:-127.0.0.1}:8100}"
  echo "[deep] selftest: GET $URL/v1/health"
  exec env PYTHONPATH=. python3 - "$URL" <<'EOF'
import os, sys
from agents.perception.deep_client import DeepClient
r = DeepClient(sys.argv[1], token=os.environ["DEEP_TOKEN"]).health()
print(f"[deep] selftest: {r.status} {r.data if r.ok else r.detail}")
sys.exit(0 if r.ok else 1)
EOF
fi

VENV=.venv-train-gpu
[ -f "$VENV/bin/activate" ] || {
  echo "ERROR: $VENV missing — create it with the deep extra:" >&2
  echo "  uv venv $VENV && uv pip install -p $VENV -e '.[deep]'" >&2
  exit 1; }
source "$VENV/bin/activate"
echo "[deep] starting sidecar ($(ls models/*.pt 2>/dev/null | wc -l) .pt in models/)"
exec python -m agents.vision.deep.service
