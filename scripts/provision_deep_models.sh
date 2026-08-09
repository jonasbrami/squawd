#!/usr/bin/env bash
# provision_deep_models.sh — pinned official weights for the deep sidecar
# (deep-perception plan §2, codex CR2: official ultralytics-assets URLs +
# HARD-CODED sha256, verified BEFORE the manifest is written; nothing is ever
# copied from ~/perception-lab). Writes models/<name>.json manifests in the
# existing {sha256, source, ...} shape and updates models/README.md.
#
# sha256 constants filled from the verified M1b downloads (2026-08-03); the
# script re-verifies every byte it moves into models/ and refuses to run if a
# constant is ever replaced by a placeholder again.
set -eo pipefail
cd "$(dirname "$0")/.."

BASE="https://github.com/ultralytics/assets/releases/download/v8.3.0"

NAMES=(yolov8s-worldv2 sam2.1_t)
declare -A URL=(
  [yolov8s-worldv2]="$BASE/yolov8s-worldv2.pt"   # YOLO-World v2 s, AGPL-3.0
  [sam2.1_t]="$BASE/sam2.1_t.pt"                 # SAM 2.1 tiny, Apache-2.0
)
declare -A SHA256=(
  [yolov8s-worldv2]="9b2c17ab6124a913e9b3a5c170617920d91b0f01111a8479da69f00e2cf27792"
  [sam2.1_t]="3c1e81ca9b037dd39d70a014ddb9a813d6c4c4e12555420db7eaff31689bd4e3"
)
declare -A LICENSE=(
  [yolov8s-worldv2]="YOLO-World (Ultralytics), AGPL-3.0"
  [sam2.1_t]="Meta SAM 2.1, Apache-2.0"
)

for name in "${NAMES[@]}"; do
  if [[ ! "${SHA256[$name]}" =~ ^[0-9a-f]{64}$ ]]; then
    echo "ERROR: SHA256[$name] is not a real digest — refusing to provision" >&2
    echo "unverified weights. Restore the pinned constant (M1b 2026-08-03)." >&2
    exit 1
  fi
done

for name in "${NAMES[@]}"; do
  pt="models/$name.pt"
  tmp="$pt.tmp"
  echo "[provision] $name <- ${URL[$name]}"
  curl -fL --retry 3 -o "$tmp" "${URL[$name]}"
  got="$(sha256sum "$tmp" | cut -d' ' -f1)"
  if [ "$got" != "${SHA256[$name]}" ]; then
    rm -f "$tmp"
    echo "ERROR: sha256 mismatch for $name: got $got, want ${SHA256[$name]}" >&2
    exit 1
  fi
  mv "$tmp" "$pt"
  python3 - "$name" "${URL[$name]}" "$got" <<'EOF'
import json, sys, datetime
name, url, sha = sys.argv[1:4]
manifest = {"sha256": sha, "source": url,
            "downloaded_at": datetime.datetime.now(datetime.timezone.utc)
            .isoformat()}
with open(f"models/{name}.json", "w") as f:
    json.dump(manifest, f, indent=2)
    f.write("\n")
print(f"[provision] wrote models/{name}.json")
EOF
  grep -q "$name.pt" models/README.md || cat >> models/README.md <<EOF
- \`$name.pt\` — ${LICENSE[$name]}; ${URL[$name]} (sha256-pinned, scripts/provision_deep_models.sh).
EOF
done
echo "[provision] done — weights verified, manifests written, README updated"
