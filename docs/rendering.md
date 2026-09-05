# Rendering & GPU

Camera rendering is the expensive part of the simulation, so the launcher lets you
choose the renderer via `RENDER_BACKEND`. For everyday use the default Intel iGPU
path is supported; see the [demo runbook](RUN-DEMO.md).

## The three backends

| | **`intel`** (default) | **`nvidia`** | **`cpu`** |
|---|---|---|---|
| Renderer | Intel iGPU (Iris Xe, `i915`) via **EGL headless** | NVIDIA dGPU via **EGL headless** (`--gpus all`) | Software GL (**llvmpipe**) |
| Camera POV tiles | ✅ real-time | ✅ real-time, most headroom | ❌ disabled (sim can't even ready) |
| Drone model | `gz_x500_depth` (camera) | `gz_x500_depth` (camera) | `gz_x500` (no camera) |
| Needs | `/dev/dri` (out of the box) | `nvidia-container-toolkit` + driver | nothing |
| Use it when | the everyday default | more camera headroom | camera-free eval work |

```bash
RENDER_BACKEND=intel  ./scripts/run_single_demo.sh demo
RENDER_BACKEND=nvidia ./scripts/run_single_demo.sh demo
```

## Resolution & rate knobs

| Var | Values | Default | What it does |
|-----|--------|---------|--------------|
| `RENDER_BACKEND` | `intel` · `nvidia` · `cpu` | `intel` | Which GPU (or none) renders cameras |
| `CAM_W` × `CAM_H` | px | `640` × `360` | Camera resolution |
| `CAM_FPS` | Hz | `10` | Camera update rate |

## How to choose (measured)

- Historical multi-instance measurements found single-thread Gazebo physics was
  the high-end limiter; the retained evidence is in
  [benchmarks/RESULTS.md](benchmarks/RESULTS.md).
- **`cpu`/llvmpipe is flight-only** — software camera render is too slow for the sim
  to reach "ready," so cameras are disabled (`gz_x500`, no camera model).

## Why it's wired this way

- Under `intel`, only **`renderD128`** is exposed to the container on purpose: if
  Mesa can see an NVIDIA node with no Mesa driver, `ogre2` segfaults.
- **NVIDIA** needs the glvnd EGL vendor ICD, which `--gpus all` does *not* create;
  `swarm_sim.sh` writes it in-container (idempotent). Full root-cause story:
  [nvidia-render-investigation.md](nvidia-render-investigation.md).
- Gazebo always runs **server-only** (`HEADLESS=1`) — no Qt GUI. The GUI aborts
  under offscreen Qt and would take down the gz-launching PX4 instance.
