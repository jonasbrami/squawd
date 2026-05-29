# In-container rendering verdict (Milestone 1)

**Date:** 2026-05-29 · **Host:** Ubuntu 24.04, Docker 29.1

## Verdict: GO — hardware EGL available in-container

- `eglinfo` (Device platform / surfaceless) reports **`Mesa Intel(R) Iris(R) Xe Graphics (ADL GT2)`** — real GPU, not llvmpipe.
- Render node `/dev/dri/renderD128` is present and passed through via `--device /dev/dri`.
- **Gazebo sensor/camera/depth rendering uses EGL**, so it is **GPU-accelerated**. This is the path that matters for the perception layer / camera feed.

## Caveats
- **GLX over Xvfb = software (llvmpipe).** The noVNC GUI window renders in software. Acceptable — it only affects the 3D *view*, not sensor data. If the GUI is too slow, run Gazebo's GUI client on the host instead, or accept the soft view.
- **Permissions:** the container user MUST be in the group owning `renderD128`.
  - Host GIDs: **`render` = 992**, **`video` = 44**.
  - Build with: `--build-arg RENDER_GID=992 --build-arg VIDEO_GID=44`
    (compose: `RENDER_GID=992 VIDEO_GID=44 docker compose up --build`).
- **Fallback:** if hardware EGL is ever unavailable, set `LIBGL_ALWAYS_SOFTWARE=1` (correct but slow) — `start-all.sh` already does this when the GL probe fails.

## NVIDIA RTX 3070 Ti (deferred)
Present but host driver not loaded (`nvidia-smi` fails) and `nvidia-container-toolkit` absent. Future toggle: fix host driver + install toolkit + run with `--gpus all`; would accelerate the GUI too.
