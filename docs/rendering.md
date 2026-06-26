# Rendering & GPU

Camera rendering is the expensive part of the swarm. This page explains the render
paths the launcher supports and why it's wired the way it is. For everyday use, the
default (Intel iGPU) just works — see the [README quickstart](../README.md#quickstart).

## With GPU vs without

The launcher has two paths, selected with `GPU=1` (default) or `GPU=0`:

| | **GPU (default, `GPU=1`)** | **No GPU (`GPU=0`)** |
|---|---|---|
| Renderer | Intel iGPU (Iris Xe, `i915`) via **EGL headless** | Software GL (**llvmpipe**) |
| Camera POV tiles | ✅ yes, real-time | ❌ disabled (too slow) |
| Real-time factor (3 cams) | **~1.0** | ~0.004 with cameras → flight only |
| Drone model | `gz_x500_depth` (has camera) | `gz_x500` (no camera) |
| Devices passed in | only `/dev/dri/renderD128` + `card1` | none |

```bash
./scripts/run_swarm_demo.sh 3          # GPU=1 default — iGPU cameras, real time
GPU=0 ./scripts/run_swarm_demo.sh 3    # software rendering — flight + chat only
```

## Why it's set up this way

- Only **`renderD128`** is exposed to the container on purpose: if Mesa can see an
  NVIDIA node with no Mesa driver, `ogre2` segfaults. Intel iGPU → EGL is the
  reliable headless path here.
- Gazebo always runs **server-only** (`HEADLESS=1`) — no Qt GUI. The GUI aborts
  under offscreen Qt and would take down the gz-launching PX4 instance.

## NVIDIA dGPU (future toggle)

An NVIDIA discrete GPU can render the camera sensors headlessly too, for larger or
faster feeds — but it isn't required; the iGPU holds 3×640×360 at real time. The
investigation confirmed ogre2/EGL camera sensors render on an RTX dGPU **with no
segfault**, attributed to the dGPU by `nvidia-smi`. The only blocker is plumbing the
NVIDIA userspace driver into the container (install `nvidia-container-toolkit` and
run with `--gpus all`, or bind-mount the NVIDIA EGL libs).

See **[nvidia-render-investigation.md](nvidia-render-investigation.md)** for the
full proof-of-concept, the exact minimal config, and the proposed `RENDER_BACKEND`
switch (cpu / intel / nvidia).
