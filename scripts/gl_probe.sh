#!/usr/bin/env bash
# Milestone-1 gate: determine the in-container rendering path for Gazebo.
# Gazebo sensor/camera rendering uses EGL (not GLX-over-Xvfb), so we probe
# EGL via /dev/dri. Hardware (non-llvmpipe) EGL => GPU-accelerated sensors.
set -euo pipefail

echo "=== /dev/dri ==="
ls -la /dev/dri 2>&1 || { echo "NO /dev/dri" >&2; exit 1; }

echo "=== EGL renderers (the path Gazebo sensor rendering uses) ==="
HW="$(eglinfo -B 2>/dev/null | grep -i 'renderer' | grep -vi llvmpipe | head -1 || true)"
if [ -n "$HW" ]; then
  echo "HARDWARE_EGL_OK:${HW}"
  echo "GL PROBE PASS (hardware EGL — GPU-accelerated sensors)"
  exit 0
fi

if eglinfo -B 2>/dev/null | grep -qi llvmpipe; then
  echo "SOFTWARE_ONLY: llvmpipe (no hardware EGL; set LIBGL_ALWAYS_SOFTWARE=1, slow)"
  echo "GL PROBE PASS (software fallback)"
  exit 0
fi

echo "NO_GL" >&2
exit 1
