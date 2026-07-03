# NVIDIA headless ogre2 camera render — investigation

**Date:** 2026-06-26
**Question:** Can Gazebo Harmonic camera *sensors* (ogre2 / EGL) render on the
**NVIDIA RTX 3070 Ti Laptop GPU** inside `dronebot-swarm:dev`, headless? If so,
what is the exact minimal config for a `RENDER_BACKEND=nvidia` benchmark switch?

## VERDICT: **WORKS-WITH-CAVEATS**

> **Update 2026-06-27:** `nvidia-container-toolkit` v1.19.1 has since been INSTALLED and the Docker `nvidia` runtime registered, so the benchmark uses the clean `--gpus all` path (Option A), NOT the bind-mount workaround (Option B). `docker run --gpus all squawd:dev nvidia-smi` works.

ogre2 camera sensors render correctly and headlessly on the NVIDIA dGPU. The
render is confirmed attributed to the RTX 3070 Ti by `nvidia-smi` (the gz process
holds GPU memory on GPU 0). **No segfault** in `handleContextCreationFailure` —
that failure only happens when Mesa is allowed to grab the NVIDIA DRI node; it is
*not* an intrinsic NVIDIA/ogre2 problem.

The single caveat is purely about plumbing the NVIDIA userspace driver into the
container: **the host has no `nvidia-container-toolkit` installed and Docker has
no `nvidia` runtime**, so the normal `--gpus all` path does not work today. Two
ways to fix this are given below. Everything else (EGL device selection, ogre2,
camera pipeline) works out of the box once the NVIDIA EGL libs are present.

---

## Environment found

Host (driver present, fully functional):
- NVIDIA RTX 3070 Ti, driver **580.159.03**, CUDA 13. `nvidia-smi` works.
- NVIDIA userspace EGL/GL stack present in `/usr/lib/x86_64-linux-gnu/`
  (`libEGL_nvidia.so.0`, `libnvidia-eglcore`, `libnvidia-glcore`, `libGLX_nvidia`,
  GBM/allocator, `libnvidia-ml`, …) and EGL ICD `/usr/share/glvnd/egl_vendor.d/10_nvidia.json`.
- DRI node mapping (from `/dev/dri/by-path`):
  - **NVIDIA** PCI `01:00.0` → `card2` / `renderD129`
  - **Intel iGPU** PCI `00:02.0` → `card1` / `renderD128`
- NVIDIA char devices: `/dev/nvidia0`, `/dev/nvidiactl`, `/dev/nvidia-uvm`(+`-tools`), `/dev/nvidia-modeset`.

The blocker:
- `nvidia-ctk` / `nvidia-container-toolkit`: **NOT installed.**
- `docker info` Runtimes: `runc` only — **no `nvidia` runtime.**
- `docker run --gpus all …` → `could not select device driver "" with capabilities: [[gpu]]`
- `docker run --runtime=nvidia …` → `unknown or invalid runtime name: nvidia`

Image (`dronebot-swarm:dev`, `ubuntu:24.04` base):
- Has GLVND (`libEGL.so.1` dispatcher, `libGLdispatch`, `libOpenGL`) and the
  ogre2 plugin (`libgz-rendering8-ogre2.so.8.2.3`).
- Only `50_mesa.json` EGL ICD; **no NVIDIA EGL libs and no NVIDIA ICD** (those are
  injected at runtime by the container toolkit, which isn't installed).

So the problem reduces to: get NVIDIA's `libEGL_nvidia` + friends into the
container and force glvnd to load *only* the NVIDIA ICD.

---

## What was proven (reproducible)

All experiments ran in a throwaway container named `nv-probe` (never touched the
user's `swarm-multi`). NVIDIA libs were copied flat (relative symlinks preserved)
from the host into a scratch dir and bind-mounted at `/nvidia-libs`.

### 1. Headless EGL binds the NVIDIA device

A ctypes EGL probe (`eglQueryDevicesEXT` → `eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT)`
→ `eglInitialize`) with `__EGL_VENDOR_LIBRARY_FILENAMES` forced to the NVIDIA ICD:

```
EGL devices found: 1
  dev[0] vendor='EGL_NV_device_cuda EGL_EXT_device_drm EGL_EXT_device_drm_render_node ...' drm='/dev/dri/card2'
NVIDIA device at index 0
eglInitialize OK: EGL 1.5 on NVIDIA device -> HEADLESS EGL WORKS
  EGL_VENDOR: NVIDIA
  EGL_VERSION: 1.5
```

Forcing `__EGL_VENDOR_LIBRARY_FILENAMES` to nvidia-only is the key lever: glvnd
then enumerates exactly one EGL device (NVIDIA) and ogre2 has nothing else to pick.

### 2. ogre2 camera sensor renders on the dGPU

`gz sim -s -r --headless-rendering /usr/share/gz/gz-sim8/worlds/camera_sensor.sdf`
loaded `gz-rendering-ogre2` with **no segfault**, created the camera, and published:

- `/camera` topic: 320x240 `RGB_INT8`, full of non-zero pixels (non-black frame).
- `real_time_factor: 1.0`.

Host `nvidia-smi` attributed the render to the dGPU:

```
|  GPU  ...  PID   Type   Process name                       GPU Memory |
|   0   ... 34676   G    ...gz-sim8/worlds/camera_sensor.sdf   156MiB   |
```

That `156MiB` + the `G` (graphics) type on GPU 0 is the proof the camera pipeline
runs on the RTX 3070 Ti, not llvmpipe and not the iGPU.

### 3. Relative perf (1 camera)

| backend | device nodes | camera Hz (10s window) | RTF | nvidia-smi attribution |
|---|---|---|---|---|
| Intel iGPU (iris) | renderD128/card1 | ~29.3 Hz | 1.00 | — |
| **NVIDIA dGPU** | renderD129/card2 + /dev/nvidia* | ~29.3 Hz | 1.00 | 156 MiB on GPU 0 ✓ |

Note: `camera_sensor.sdf` is a single low-res camera on a trivial scene, so both
backends saturate the sensor's update-rate cap and the run is **not render-bound**
— it cannot separate the two on FPS. The real benchmark (many cameras / higher
resolution / heavier world) is where the dGPU should pull ahead; this investigation
only establishes that the NVIDIA path is correct and produces real frames.

---

## How to wire `RENDER_BACKEND=nvidia`

### Option A (recommended, clean) — install `nvidia-container-toolkit`

This is the proper fix; it auto-injects the NVIDIA userspace driver and avoids the
~650 MB bind-mount. Requires interactive sudo (manual step, not done here):

```bash
# Host, one-time:
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

Then the container args become just:

```
--gpus all
-e NVIDIA_VISIBLE_DEVICES=all
-e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility,display
-e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
# and DO NOT set MESA_LOADER_DRIVER_OVERRIDE
```

`NVIDIA_DRIVER_CAPABILITIES` **must include `graphics`** (and `display`) or the
toolkit injects only the compute libs and EGL/ogre2 stays broken. `__EGL_VENDOR_LIBRARY_FILENAMES`
forces glvnd to the NVIDIA ICD so ogre2 can't accidentally pick Mesa.

### Option B (works today, no sudo) — manual lib injection

Exactly what was proven above. Copy the host NVIDIA libs once, then bind-mount:

```bash
# one-time, on host:
mkdir -p /tmp/nvlibs
cp -d /usr/lib/x86_64-linux-gnu/libnvidia-*.so*    /tmp/nvlibs/
cp -d /usr/lib/x86_64-linux-gnu/libEGL_nvidia.so*  /tmp/nvlibs/
cp -d /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so*  /tmp/nvlibs/
cp -d /usr/lib/x86_64-linux-gnu/libGLESv*_nvidia.so* /tmp/nvlibs/
cp -d /usr/lib/x86_64-linux-gnu/libcuda.so*        /tmp/nvlibs/
printf '{\n  "file_format_version":"1.0.0",\n  "ICD":{"library_path":"libEGL_nvidia.so.0"}\n}\n' > /tmp/10_nvidia.json
```

Container args:

```
--device /dev/nvidia0 --device /dev/nvidiactl --device /dev/nvidia-uvm
--device /dev/dri/renderD129 --device /dev/dri/card2 --group-add <render gid>
-v /tmp/nvlibs:/nvidia-libs:ro
-v /tmp/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json:ro
-e LD_LIBRARY_PATH=/nvidia-libs
-e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
# and DO NOT set MESA_LOADER_DRIVER_OVERRIDE
```

> The NVIDIA lib version is pinned to the host driver (580.159.03). After a host
> driver upgrade, re-copy `/tmp/nvlibs`. Option A avoids this by tracking the driver
> automatically. The lib version inside the container **must** match the host kernel
> module, so you cannot apt-install a different NVIDIA userspace in the image.

### One-line proof command (either option)

```bash
docker exec <ctr> bash -lc 'export GZ_CONFIG_PATH=/usr/share/gz PATH=/usr/bin:$PATH; \
  gz sim -s -r --headless-rendering /usr/share/gz/gz-sim8/worlds/camera_sensor.sdf & sleep 12'; \
nvidia-smi | grep -i 'gz\|camera_sensor'   # must show a `G` process holding GPU mem on GPU 0
```

---

## Proposed (NOT applied) diffs

These are proposals only — review before applying. They add a `RENDER_BACKEND`
switch (`cpu` | `intel` | `nvidia`) while keeping the current Intel iGPU path as
the default behaviour of `GPU_RENDER=1`.

### `sim/launch/swarm_sim.sh`

```diff
@@
-# Rendering backend: software GL (llvmpipe) by default so cameras work with no GPU.
-# Set GPU_RENDER=1 (and pass --device /dev/dri) to use hardware EGL instead — far
-# faster, lets N camera feeds + live flight coexist.
-if [ "${GPU_RENDER:-0}" = "1" ]; then
-  unset LIBGL_ALWAYS_SOFTWARE
-  # Headless EGL on the Intel iGPU. Expose ONLY renderD128 to the container so
-  # Mesa can't grab the NVIDIA node (no Mesa driver there -> ogre2 segfault).
-  export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-iris}"
-  export QT_QPA_PLATFORM=offscreen
-else
-  export LIBGL_ALWAYS_SOFTWARE=1
-fi
+# Rendering backend selector.
+#   RENDER_BACKEND=cpu     -> software GL (llvmpipe), no GPU needed (default if GPU_RENDER!=1)
+#   RENDER_BACKEND=intel   -> headless EGL on the Intel iGPU (current GPU_RENDER=1 behaviour)
+#   RENDER_BACKEND=nvidia  -> headless EGL on the NVIDIA dGPU (needs NVIDIA EGL libs + /dev/nvidia*)
+# Back-compat: GPU_RENDER=1 with no RENDER_BACKEND == intel.
+RENDER_BACKEND="${RENDER_BACKEND:-}"
+if [ -z "$RENDER_BACKEND" ]; then
+  [ "${GPU_RENDER:-0}" = "1" ] && RENDER_BACKEND=intel || RENDER_BACKEND=cpu
+fi
+case "$RENDER_BACKEND" in
+  nvidia)
+    unset LIBGL_ALWAYS_SOFTWARE
+    # ogre2 must use NVIDIA's EGL, never Mesa: force the glvnd vendor ICD and
+    # make sure no Mesa driver override is set.
+    unset MESA_LOADER_DRIVER_OVERRIDE
+    export __EGL_VENDOR_LIBRARY_FILENAMES="${__EGL_VENDOR_LIBRARY_FILENAMES:-/usr/share/glvnd/egl_vendor.d/10_nvidia.json}"
+    export QT_QPA_PLATFORM=offscreen
+    ;;
+  intel)
+    unset LIBGL_ALWAYS_SOFTWARE
+    # Headless EGL on the Intel iGPU. Expose ONLY renderD128 to the container so
+    # Mesa can't grab the NVIDIA node (no Mesa driver there -> ogre2 segfault).
+    export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-iris}"
+    export QT_QPA_PLATFORM=offscreen
+    ;;
+  *)  # cpu / llvmpipe
+    export LIBGL_ALWAYS_SOFTWARE=1
+    ;;
+esac
```

### `scripts/run_swarm_demo.sh`

```diff
@@
 N="${1:-3}"
 WORLD="${WORLD:-baylands}"
 IMG="dronebot-swarm:dev"
 GPU="${GPU:-1}"
+# RENDER_BACKEND: cpu | intel | nvidia. Default mirrors old behaviour: GPU=1 -> intel.
+RENDER_BACKEND="${RENDER_BACKEND:-$([ "$GPU" = 1 ] && echo intel || echo cpu)}"
 RENDER_GID="$(getent group render | cut -d: -f3 || echo 992)"
 VIDEO_GID="$(getent group video | cut -d: -f3 || echo 44)"
@@
-GPU_ARGS=()
-if [ "$GPU" = "1" ]; then
-  GPU_ARGS=(--device /dev/dri/renderD128 --device /dev/dri/card1
-            --group-add "$RENDER_GID" --group-add "$VIDEO_GID"
-            -e GPU_RENDER=1 -e PX4_MODEL=gz_x500_depth)
-else
-  GPU_ARGS=(-e PX4_MODEL=gz_x500)
-fi
+GPU_ARGS=()
+case "$RENDER_BACKEND" in
+  intel)
+    GPU_ARGS=(--device /dev/dri/renderD128 --device /dev/dri/card1
+              --group-add "$RENDER_GID" --group-add "$VIDEO_GID"
+              -e RENDER_BACKEND=intel -e PX4_MODEL=gz_x500_depth)
+    ;;
+  nvidia)
+    # Prefer the NVIDIA container runtime if it's installed; otherwise fall back
+    # to manual lib injection from the host driver (must match host driver version).
+    if docker info 2>/dev/null | grep -qi 'Runtimes:.*nvidia'; then
+      GPU_ARGS=(--gpus all
+                -e NVIDIA_VISIBLE_DEVICES=all
+                -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility,display)
+    else
+      NVLIBS=/tmp/swarm-nvlibs
+      mkdir -p "$NVLIBS"
+      cp -d /usr/lib/x86_64-linux-gnu/libnvidia-*.so* "$NVLIBS/" 2>/dev/null || true
+      cp -d /usr/lib/x86_64-linux-gnu/libEGL_nvidia.so* "$NVLIBS/" 2>/dev/null || true
+      cp -d /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so* "$NVLIBS/" 2>/dev/null || true
+      cp -d /usr/lib/x86_64-linux-gnu/libGLESv*_nvidia.so* "$NVLIBS/" 2>/dev/null || true
+      cp -d /usr/lib/x86_64-linux-gnu/libcuda.so* "$NVLIBS/" 2>/dev/null || true
+      printf '{\n  "file_format_version":"1.0.0",\n  "ICD":{"library_path":"libEGL_nvidia.so.0"}\n}\n' > /tmp/10_nvidia.json
+      GPU_ARGS=(--device /dev/nvidia0 --device /dev/nvidiactl --device /dev/nvidia-uvm
+                --device /dev/dri/renderD129 --device /dev/dri/card2
+                --group-add "$RENDER_GID"
+                -v "$NVLIBS":/nvidia-libs:ro
+                -v /tmp/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json:ro
+                -e LD_LIBRARY_PATH=/nvidia-libs)
+    fi
+    GPU_ARGS+=(-e RENDER_BACKEND=nvidia
+               -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
+               -e PX4_MODEL=gz_x500_depth)
+    ;;
+  *)  # cpu
+    GPU_ARGS=(-e RENDER_BACKEND=cpu -e PX4_MODEL=gz_x500)
+    ;;
+esac
```

Usage after the change:

```bash
RENDER_BACKEND=nvidia ./scripts/run_swarm_demo.sh 1   # dGPU
RENDER_BACKEND=intel  ./scripts/run_swarm_demo.sh 3   # iGPU (current default)
RENDER_BACKEND=cpu    ./scripts/run_swarm_demo.sh 1   # llvmpipe
```

---

## Notes for the benchmark harness

- `RENDER_BACKEND=nvidia` is **viable** — do not mark it N/A.
- For a meaningful CPU/iGPU/dGPU sweep, push the workload past the sensor update-rate
  cap (raise camera resolution and/or drone count); the single 640x360@10Hz feed in
  the swarm, like the probe world here, is too light to separate backends.
- Verify each run's backend with `nvidia-smi` (dGPU) or absence-of-it + `glxinfo`/iris
  (iGPU) rather than trusting env vars.
- Gotcha: if both Mesa and NVIDIA EGL ICDs are visible and `__EGL_VENDOR_LIBRARY_FILENAMES`
  is unset, glvnd may enumerate multiple EGL devices and ogre2's pick is
  non-deterministic. Always force the ICD filename for a clean per-backend run.
- The `handleContextCreationFailure` segfault is a *Mesa-grabbing-the-NVIDIA-node*
  symptom, not an NVIDIA limitation; it does not occur on the NVIDIA EGL path.

---

## Integrated swarm path (2026-06-27)

**VERDICT: FIXED.** `RENDER_BACKEND=nvidia ./scripts/run_swarm_demo.sh 1` now
brings up the full swarm on the RTX 3070 Ti dGPU with a real camera frame and no
segfault, for both `WORLD=default` and `WORLD=baylands` (the benchmark world).

### Root cause (one sentence)

The `--gpus all` (Option A toolkit) path injects `libEGL_nvidia.so.0` but the
nvidia-container-toolkit does **not** create the glvnd vendor ICD
`/usr/share/glvnd/egl_vendor.d/10_nvidia.json`, so the script's
`__EGL_VENDOR_LIBRARY_FILENAMES=…/10_nvidia.json` pointed at a *non-existent file*
→ glvnd loaded no NVIDIA ICD → ogre2 null-dereferenced in
`Ogre2RenderEngine::CreateRenderSystem()` (the reported `Address not mapped to
object [(nil)]` segfault).

### How it was bisected

Standalone `--gpus all` probe (`nv-probe`, `squawd:dev`) showed:
- `/dev/dri` is **absent** in the `--gpus all` container (only `/dev/nvidia*`
  char devs) — so hypothesis #1 (missing DRM render node) looked plausible, BUT…
- the only EGL ICD present is `50_mesa.json`; there is **no** `10_nvidia.json`
  anywhere (`find / -name '*nvidia*egl*.json'` empty), and the NVIDIA EGL libs
  (`libEGL_nvidia.so.0`, `libnvidia-eglcore`, `libGLX_nvidia`) **are** injected.
- Writing the trivial ICD json (pointing at `libEGL_nvidia.so.0`) and running
  `gz sim -s -r --headless-rendering camera_sensor.sdf` → **no segfault**, create
  service live, `nvidia-smi` shows the gz process at 156 MiB `G` on GPU 0 — all
  **without** adding `/dev/dri` (NVIDIA's EGL enumerates via `/dev/nvidia0`, the
  `EGL_EXT_device_drm` node is not required for headless ogre2).
- Deleting the ICD again (env var still pointing at it) reproduced the exact
  reported crash: `Ogre2RenderEngine::CreateRenderSystem() … Segmentation fault
  (Address not mapped to object [(nil)])`, `gz exit=139`.

So hypothesis #2 (glvnd has no NVIDIA ICD to enumerate) was the real cause;
hypothesis #1 (DRI render node) and #3 (caps/lib mismatch) were **not** the
blocker — `graphics` caps already inject the libs, and `/dev/dri` is unnecessary.

### The fix (unified diff — `sim/launch/swarm_sim.sh`, `nvidia)` case)

```diff
   nvidia)
     unset LIBGL_ALWAYS_SOFTWARE
     # ogre2 must use NVIDIA's EGL, never Mesa: force the glvnd vendor ICD and
     # make sure no Mesa driver override is set.
     unset MESA_LOADER_DRIVER_OVERRIDE
-    export __EGL_VENDOR_LIBRARY_FILENAMES="${__EGL_VENDOR_LIBRARY_FILENAMES:-/usr/share/glvnd/egl_vendor.d/10_nvidia.json}"
+    NV_ICD="${__EGL_VENDOR_LIBRARY_FILENAMES:-/usr/share/glvnd/egl_vendor.d/10_nvidia.json}"
+    # The nvidia-container-toolkit (`--gpus all`, caps incl. graphics) injects
+    # libEGL_nvidia.so.0 but does NOT create the glvnd vendor ICD json. Without it
+    # __EGL_VENDOR_LIBRARY_FILENAMES points at a missing file -> glvnd loads no
+    # NVIDIA ICD -> ogre2 null-derefs in Ogre2RenderEngine::CreateRenderSystem
+    # (the reported segfault). Create it here, idempotently, when the NVIDIA EGL
+    # lib is actually present.
+    if [ ! -f "$NV_ICD" ] && ls /usr/lib/x86_64-linux-gnu/libEGL_nvidia.so.0 >/dev/null 2>&1; then
+      mkdir -p "$(dirname "$NV_ICD")"
+      printf '{\n  "file_format_version":"1.0.0",\n  "ICD":{"library_path":"libEGL_nvidia.so.0"}\n}\n' > "$NV_ICD"
+    fi
+    export __EGL_VENDOR_LIBRARY_FILENAMES="$NV_ICD"
     export QT_QPA_PLATFORM=offscreen
     # NVIDIA EGL needs the explicit headless-rendering surface or ogre2 segfaults
     # in CreateRenderSystem (the Mesa/iris path tolerates its absence).
     GZ_HR="--headless-rendering"
     ;;
```

`scripts/run_swarm_demo.sh` was left unchanged — the `--gpus all` args (no
`/dev/dri`, no bind-mount) are correct; the only missing piece was the ICD file,
which is now created in-container by `swarm_sim.sh`. The `intel` and `cpu` paths
are untouched.

### Proof

```
RENDER_BACKEND=nvidia WORLD=default  ./scripts/run_swarm_demo.sh 1  -> "sim ready (1/1 drones)."
RENDER_BACKEND=nvidia WORLD=baylands ./scripts/run_swarm_demo.sh 1  -> "sim ready (1/1 drones)."
docker exec swarm-multi grep -i segfault /tmp/gz.log                -> NO SEGFAULT
camera topic /world/<w>/.../IMX214/image                            -> non-black frame (0xDA bytes)
nvidia-smi:
  GPU 0 ... <pid> G  .../gz/worlds/default.sdf   285MiB   (empty world)
  GPU 0 ... <pid> G  .../gz/worlds/baylands.sdf   891MiB  (benchmark world)
```

The `G` (graphics) type on GPU 0 confirms the camera pipeline renders on the dGPU.

### Notes

- `--gpus all` does not need `/dev/dri/renderD129`/`card2`; NVIDIA EGL enumerates
  the device from `/dev/nvidia0`. (Option B exposed `/dev/dri/card2` only because
  it had no toolkit; it is not required on the toolkit path.)
- Only one-line piece of plumbing was wrong; everything else in the
  already-committed `RENDER_BACKEND=nvidia` wiring was correct.
