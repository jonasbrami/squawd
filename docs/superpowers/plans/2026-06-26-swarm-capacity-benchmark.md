# Swarm Capacity Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a benchmark harness that measures, per render backend (CPU/llvmpipe, Intel iGPU, NVIDIA dGPU), the maximum sustainable drone count at each camera resolution, gated on delivered camera FPS + real-time-factor + flight liveness, and emits a capacity-frontier table/heatmap.

**Architecture:** A host-side Python orchestrator drives the existing Docker swarm stack once per `(backend, resolution, N)` run. Pure, dependency-free logic (FPS/verdict math, tool-output parsers, the capacity search, frontier rendering) lives in a top-level `bench/` package and is fully unit-tested without ROS or Docker; thin CLI glue (`sample_host.py`, `run_bench.py`) shells out to the real tools and is integration-verified. Two small enabling changes expose the camera frame counter on the observatory `/state` endpoint and parametrize the sim launch (`RENDER_BACKEND`, `CAM_W/H/FPS`).

**Tech Stack:** Python 3.10+, pytest, Docker, Gazebo Harmonic (gz), ROS2 Jazzy, PX4 SITL, `nvidia-smi`, `intel_gpu_top`, bash.

## Global Constraints

- Python `>=3.10` (per `pyproject.toml`).
- Run code/tests via `uv run --extra dev pytest <path>` (fallback `python -m pytest`); pytest config in `pyproject.toml` sets `pythonpath = ["."]`, so top-level `bench/` is importable from tests.
- Benchmark world is `baylands`. Camera sensor rate default `CAM_FPS=10`.
- Render backends and their exact launch env (already proven, see `docs/nvidia-render-investigation.md`):
  - `cpu` → `LIBGL_ALWAYS_SOFTWARE=1`, model `gz_x500_depth`.
  - `intel` → `MESA_LOADER_DRIVER_OVERRIDE=iris`, `QT_QPA_PLATFORM=offscreen`, devices `/dev/dri/renderD128 /dev/dri/card1`.
  - `nvidia` → `--gpus all`, `NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility,display`, `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`, **no** `MESA_LOADER_DRIVER_OVERRIDE`. (`nvidia-container-toolkit` v1.19.1 is installed; Docker `nvidia` runtime registered.)
- Pass bar (a run PASSES iff all hold over the measurement window): `min_i fps_i ≥ 0.90 × CAM_FPS` **AND** `real_time_factor ≥ 0.90` **AND** all `N` drones armed/alive at window end.
- Defaults: resolutions {320×180, 640×360, 960×540, 1280×720, 1920×1080}@10 Hz; N ramp cap 32; per run 30 s settle + 60 s measure; per-run hard time-box ~10 min.
- **Do not change the interactive demo's default behaviour** — `./scripts/run_swarm_demo.sh` with no new env must behave exactly as before (intel backend, 640×360@10).
- Time-box every Docker bring-up / poll loop with a hard deadline so nothing hangs for hours.
- Branch: `bench/swarm-capacity` (already created). Commit after every task.

---

### Task 1: Expose the camera frame counter on `/state`

**Files:**
- Modify: `agents/observatory/metrics.py:43-69` (`build_drone_state`)
- Modify: `agents/observatory/server.py:62-75` (`state`)
- Test: `tests/test_observatory_metrics.py`

**Interfaces:**
- Consumes: existing `GzCameras.seq(i) -> int` (camera.py:64), monotonic per-drone frame counter (0 if no frame yet).
- Produces: `/state` JSON drones now carry `cam_seq: int`. The orchestrator (Task 8) reads `drones[i]["cam_seq"]` and `drones[i]["armed"]`.

- [ ] **Step 1: Write the failing test**

Add to `tests/test_observatory_metrics.py`:

```python
def test_build_drone_state_includes_cam_seq():
    d = metrics.build_drone_state(2, None, None, None, None, None, True, cam_seq=57)
    assert d["cam_seq"] == 57

def test_build_drone_state_cam_seq_defaults_zero():
    d = metrics.build_drone_state(0, None, None, None, None, None, False)
    assert d["cam_seq"] == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run --extra dev pytest tests/test_observatory_metrics.py::test_build_drone_state_includes_cam_seq -v`
Expected: FAIL — `TypeError: build_drone_state() got an unexpected keyword argument 'cam_seq'`.

- [ ] **Step 3: Add the `cam_seq` parameter and field**

In `agents/observatory/metrics.py`, change the signature and the returned dict:

```python
def build_drone_state(i, pos, status, batt, task, report, has_cam, cam_seq=0):
```

and add this entry to the returned dict (next to `"cam": has_cam,`):

```python
        "cam": has_cam,
        "cam_seq": cam_seq,
```

Update the docstring's argument list to mention `cam_seq -> GzCameras.seq(i), the per-drone frame counter`.

- [ ] **Step 4: Feed the real counter in the server**

In `agents/observatory/server.py`, in `state()`, change the `build_drone_state(...)` call's last argument block from:

```python
            cameras.has(i),
        ))
```

to:

```python
            cameras.has(i),
            cameras.seq(i),
        ))
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `uv run --extra dev pytest tests/test_observatory_metrics.py -v`
Expected: PASS (all tests, including the two new ones and the pre-existing ones unchanged).

- [ ] **Step 6: Commit**

```bash
git add agents/observatory/metrics.py agents/observatory/server.py tests/test_observatory_metrics.py
git commit -m "feat(observatory): expose per-drone cam_seq on /state for FPS measurement"
```

---

### Task 2: Bench FPS + verdict logic (`bench/metrics.py`)

**Files:**
- Create: `bench/__init__.py` (empty)
- Create: `bench/metrics.py`
- Create: `tests/bench/__init__.py` (empty)
- Test: `tests/bench/test_metrics.py`

**Interfaces:**
- Produces:
  - `compute_fps(seq_start: dict[int,int], seq_end: dict[int,int], dt: float) -> dict[int,float]`
  - `fps_summary(fps: dict[int,float]) -> dict` → `{"min","mean","p10"}` (floats; empty input → all `0.0`)
  - `evaluate_verdict(fps_min: float, cam_fps: float, rtf: float, alive: int, n: int, *, fps_frac: float = 0.90, rtf_min: float = 0.90) -> dict` → `{"pass": bool, "reasons": list[str]}`

- [ ] **Step 1: Write the failing tests**

Create `bench/__init__.py` and `tests/bench/__init__.py` as empty files, then create `tests/bench/test_metrics.py`:

```python
from bench import metrics


def test_compute_fps_per_drone():
    start = {0: 100, 1: 50}
    end = {0: 160, 1: 80}
    fps = metrics.compute_fps(start, end, dt=6.0)
    assert fps[0] == 10.0      # 60 frames / 6 s
    assert fps[1] == 5.0       # 30 frames / 6 s


def test_compute_fps_zero_dt_is_zero():
    assert metrics.compute_fps({0: 1}, {0: 9}, dt=0.0) == {0: 0.0}


def test_fps_summary():
    s = metrics.fps_summary({0: 10.0, 1: 9.0, 2: 8.0})
    assert s["min"] == 8.0
    assert round(s["mean"], 2) == 9.0
    assert s["p10"] == 8.0     # 10th percentile of 3 values -> the min here


def test_fps_summary_empty():
    s = metrics.fps_summary({})
    assert s == {"min": 0.0, "mean": 0.0, "p10": 0.0}


def test_verdict_pass():
    v = metrics.evaluate_verdict(fps_min=9.5, cam_fps=10.0, rtf=0.97, alive=4, n=4)
    assert v["pass"] is True
    assert v["reasons"] == []


def test_verdict_fails_on_low_fps():
    v = metrics.evaluate_verdict(fps_min=8.0, cam_fps=10.0, rtf=0.99, alive=4, n=4)
    assert v["pass"] is False
    assert any("fps" in r for r in v["reasons"])


def test_verdict_fails_on_low_rtf():
    v = metrics.evaluate_verdict(fps_min=9.9, cam_fps=10.0, rtf=0.5, alive=4, n=4)
    assert v["pass"] is False
    assert any("rtf" in r for r in v["reasons"])


def test_verdict_fails_on_dead_drone():
    v = metrics.evaluate_verdict(fps_min=9.9, cam_fps=10.0, rtf=0.99, alive=3, n=4)
    assert v["pass"] is False
    assert any("alive" in r for r in v["reasons"])
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run --extra dev pytest tests/bench/test_metrics.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bench.metrics'`.

- [ ] **Step 3: Implement `bench/metrics.py`**

```python
"""Pure benchmark math: delivered-FPS from frame-counter deltas, and the
PASS/FAIL verdict. No ROS, Docker, or subprocess imports — unit-tested directly.
"""


def compute_fps(seq_start: dict[int, int], seq_end: dict[int, int], dt: float) -> dict[int, float]:
    """Per-drone delivered FPS = (end_seq - start_seq) / dt. dt<=0 -> 0.0."""
    out: dict[int, float] = {}
    for i, s0 in seq_start.items():
        s1 = seq_end.get(i, s0)
        out[i] = (s1 - s0) / dt if dt > 0 else 0.0
    return out


def _percentile(values: list[float], pct: float) -> float:
    """Linear-interpolated percentile (pct in 0..100). Empty -> 0.0."""
    if not values:
        return 0.0
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    rank = (pct / 100.0) * (len(xs) - 1)
    lo = int(rank)
    frac = rank - lo
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * frac


def fps_summary(fps: dict[int, float]) -> dict:
    vals = list(fps.values())
    if not vals:
        return {"min": 0.0, "mean": 0.0, "p10": 0.0}
    return {
        "min": min(vals),
        "mean": sum(vals) / len(vals),
        "p10": _percentile(vals, 10.0),
    }


def evaluate_verdict(fps_min: float, cam_fps: float, rtf: float, alive: int, n: int,
                     *, fps_frac: float = 0.90, rtf_min: float = 0.90) -> dict:
    """PASS iff min-FPS >= fps_frac*cam_fps AND rtf >= rtf_min AND alive == n."""
    reasons: list[str] = []
    fps_bar = fps_frac * cam_fps
    if fps_min < fps_bar:
        reasons.append(f"fps {fps_min:.2f} < bar {fps_bar:.2f}")
    if rtf < rtf_min:
        reasons.append(f"rtf {rtf:.2f} < {rtf_min:.2f}")
    if alive != n:
        reasons.append(f"alive {alive}/{n}")
    return {"pass": not reasons, "reasons": reasons}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run --extra dev pytest tests/bench/test_metrics.py -v`
Expected: PASS (8 tests).

- [ ] **Step 5: Commit**

```bash
git add bench/__init__.py bench/metrics.py tests/bench/__init__.py tests/bench/test_metrics.py
git commit -m "feat(bench): pure FPS + verdict logic"
```

---

### Task 3: Tool-output parsers + limiting-resource (`bench/probes.py`)

**Files:**
- Create: `bench/probes.py`
- Test: `tests/bench/test_probes.py`

**Interfaces:**
- Produces:
  - `parse_nvidia_smi(line: str) -> dict` → `{"util","mem_used_mb","power_w"}` from one `--format=csv,noheader,nounits` line.
  - `parse_intel_gpu_top(obj: dict | str) -> dict` → `{"render_pct","video_pct"}` from one `intel_gpu_top -J` object.
  - `parse_docker_stats(line: str) -> dict` → `{"cpu_pct","mem_mb"}` from `docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}'`.
  - `parse_gz_rtf(text: str) -> float` → `real_time_factor` from a `gz topic -e -t /world/<w>/stats` dump.
  - `limiting_resource(sample: dict, *, vram_total_mb: float = 8192, ram_total_gb: float = 31.0, threshold: float = 0.6) -> str` → one of `"cpu","igpu","dgpu","vram","ram","none"`.
- A "headroom sample" dict (produced by Task 6, consumed here) has shape:
  `{"cpu_pct": float, "ram_used_gb": float, "nvidia": {"util","mem_used_mb","power_w"}, "intel": {"render_pct","video_pct"}}` (any sub-dict may be `{}` if that tool was unavailable).

- [ ] **Step 1: Write the failing tests**

Create `tests/bench/test_probes.py`:

```python
from bench import probes


def test_parse_nvidia_smi():
    d = probes.parse_nvidia_smi("42, 1536, 78.50")
    assert d == {"util": 42.0, "mem_used_mb": 1536.0, "power_w": 78.5}


def test_parse_intel_gpu_top_matches_render_and_video():
    obj = {"engines": {
        "Render/3D": {"busy": 65.4, "unit": "%"},
        "Blitter/0": {"busy": 0.0, "unit": "%"},
        "Video/0": {"busy": 12.0, "unit": "%"},
        "VideoEnhance/0": {"busy": 0.0, "unit": "%"},
    }}
    d = probes.parse_intel_gpu_top(obj)
    assert d["render_pct"] == 65.4
    assert d["video_pct"] == 12.0


def test_parse_docker_stats_gib():
    d = probes.parse_docker_stats("231.40% 4.5GiB / 31GiB")
    assert d["cpu_pct"] == 231.4
    assert round(d["mem_mb"], 1) == 4608.0


def test_parse_docker_stats_mib():
    d = probes.parse_docker_stats("12.00% 800MiB / 31GiB")
    assert round(d["mem_mb"], 1) == 800.0


def test_parse_gz_rtf():
    text = "real_time_factor: 0.984\nsim_time {\n  sec: 12\n}\n"
    assert probes.parse_gz_rtf(text) == 0.984


def test_limiting_resource_picks_max_normalized():
    sample = {"cpu_pct": 50.0, "ram_used_gb": 6.0,
              "nvidia": {"util": 95.0, "mem_used_mb": 2000.0, "power_w": 90.0},
              "intel": {"render_pct": 30.0, "video_pct": 10.0}}
    assert probes.limiting_resource(sample) == "dgpu"


def test_limiting_resource_vram_beats_util():
    sample = {"cpu_pct": 10.0, "ram_used_gb": 2.0,
              "nvidia": {"util": 40.0, "mem_used_mb": 7800.0, "power_w": 50.0},
              "intel": {}}
    assert probes.limiting_resource(sample) == "vram"


def test_limiting_resource_none_when_all_idle():
    sample = {"cpu_pct": 5.0, "ram_used_gb": 1.0,
              "nvidia": {"util": 3.0, "mem_used_mb": 200.0, "power_w": 10.0},
              "intel": {"render_pct": 2.0, "video_pct": 0.0}}
    assert probes.limiting_resource(sample) == "none"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run --extra dev pytest tests/bench/test_probes.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bench.probes'`.

- [ ] **Step 3: Implement `bench/probes.py`**

```python
"""Pure parsers for the host-sampler tool outputs, plus limiting-resource
attribution. No subprocess calls here — Task 6's CLI does the shelling out and
hands these the captured text/objects.
"""
import json
import re


def parse_nvidia_smi(line: str) -> dict:
    """One `nvidia-smi --query-gpu=utilization.gpu,memory.used,power.draw
    --format=csv,noheader,nounits` line, e.g. '42, 1536, 78.50'."""
    parts = [p.strip() for p in line.split(",")]
    util, mem, power = (parts + ["0", "0", "0"])[:3]
    return {"util": float(util), "mem_used_mb": float(mem), "power_w": float(power)}


def parse_intel_gpu_top(obj) -> dict:
    """One `intel_gpu_top -J` sample object (dict or JSON string). Engine keys
    vary by kernel ('Render/3D', 'Render/3D/0', 'Video/0', …); match by prefix.
    'VideoEnhance' is NOT the video engine."""
    if isinstance(obj, str):
        obj = json.loads(obj)
    engines = obj.get("engines", {})

    def busy(prefix: str, exclude: str | None = None) -> float:
        for name, e in engines.items():
            if name.startswith(prefix) and (exclude is None or not name.startswith(exclude)):
                return float(e.get("busy", 0.0))
        return 0.0

    return {"render_pct": busy("Render"), "video_pct": busy("Video", exclude="VideoEnhance")}


def parse_docker_stats(line: str) -> dict:
    """`docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}'`,
    e.g. '231.40% 4.5GiB / 31GiB'."""
    m = re.match(r"\s*([\d.]+)%\s+([\d.]+)\s*([KMGT]?i?B)", line)
    if not m:
        return {"cpu_pct": 0.0, "mem_mb": 0.0}
    cpu = float(m.group(1))
    val = float(m.group(2))
    unit = m.group(3).rstrip("B").rstrip("i")
    scale = {"K": 1 / 1024, "M": 1.0, "G": 1024.0, "T": 1024.0 * 1024, "": 1 / (1024 * 1024)}.get(unit, 1.0)
    return {"cpu_pct": cpu, "mem_mb": val * scale}


def parse_gz_rtf(text: str) -> float:
    """Extract real_time_factor from a `gz topic -e -t /world/<w>/stats` dump."""
    m = re.search(r"real_time_factor:\s*([\d.]+)", text)
    return float(m.group(1)) if m else 0.0


def limiting_resource(sample: dict, *, vram_total_mb: float = 8192,
                      ram_total_gb: float = 31.0, threshold: float = 0.6) -> str:
    """Which resource is closest to saturation. Returns 'none' if the most-loaded
    resource is below `threshold` of its limit (i.e. nothing is actually the
    bottleneck — the FAIL was elsewhere)."""
    nv = sample.get("nvidia") or {}
    ig = sample.get("intel") or {}
    norm = {
        "cpu": sample.get("cpu_pct", 0.0) / 100.0,
        "ram": sample.get("ram_used_gb", 0.0) / ram_total_gb,
        "dgpu": nv.get("util", 0.0) / 100.0,
        "vram": nv.get("mem_used_mb", 0.0) / vram_total_mb,
        "igpu": max(ig.get("render_pct", 0.0), ig.get("video_pct", 0.0)) / 100.0,
    }
    key = max(norm, key=norm.get)
    return key if norm[key] >= threshold else "none"
```

Note: host total CPU% is normalized against 100 (psutil's aggregate 0–100 scale, set in Task 6), so `cpu_pct` here is host-aggregate, not the container's >100% per-core figure.

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run --extra dev pytest tests/bench/test_probes.py -v`
Expected: PASS (8 tests).

- [ ] **Step 5: Commit**

```bash
git add bench/probes.py tests/bench/test_probes.py
git commit -m "feat(bench): tool-output parsers + limiting-resource attribution"
```

---

### Task 4: Capacity search (`bench/sweep.py`)

**Files:**
- Create: `bench/sweep.py`
- Test: `tests/bench/test_sweep.py`

**Interfaces:**
- Produces: `find_knee(pass_fn: Callable[[int], bool], n_cap: int, seed: int = 1) -> int` — returns the largest `N ≤ n_cap` for which `pass_fn(N)` is True, assuming `pass_fn` is monotonic (if `N` passes, every smaller `N` passes). Returns `0` if `pass_fn(1)` is False. Memoizes `pass_fn` internally (each `N` evaluated at most once).

- [ ] **Step 1: Write the failing tests**

Create `tests/bench/test_sweep.py`:

```python
from bench import sweep


def test_knee_midrange():
    calls = []

    def passes(n):
        calls.append(n)
        return n <= 7

    assert sweep.find_knee(passes, n_cap=32, seed=1) == 7
    # memoized: no N evaluated twice
    assert len(calls) == len(set(calls))


def test_knee_all_pass_returns_cap():
    assert sweep.find_knee(lambda n: True, n_cap=32, seed=1) == 32


def test_knee_none_pass_returns_zero():
    assert sweep.find_knee(lambda n: False, n_cap=32, seed=1) == 0


def test_knee_seed_above_capacity_searches_down():
    assert sweep.find_knee(lambda n: n <= 5, n_cap=32, seed=8) == 5


def test_knee_exact_at_cap():
    assert sweep.find_knee(lambda n: n <= 32, n_cap=32, seed=4) == 32
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run --extra dev pytest tests/bench/test_sweep.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bench.sweep'`.

- [ ] **Step 3: Implement `bench/sweep.py`**

```python
"""Capacity-frontier search: exponential-probe-then-bisect for the largest N
that passes, seeded by a neighbouring cell's knee. Assumes pass_fn is monotonic.
"""
from typing import Callable


def find_knee(pass_fn: Callable[[int], bool], n_cap: int, seed: int = 1) -> int:
    cache: dict[int, bool] = {}

    def ok(n: int) -> bool:
        if n not in cache:
            cache[n] = bool(pass_fn(n))
        return cache[n]

    seed = max(1, min(seed, n_cap))

    if not ok(seed):
        # seed fails: find the largest passing N in [1, seed)
        if not ok(1):
            return 0
        lo, hi = 1, seed            # lo passes, hi fails
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return lo

    # seed passes: probe upward by doubling
    last_pass = seed
    while last_pass < n_cap:
        nxt = min(last_pass * 2, n_cap)
        if ok(nxt):
            last_pass = nxt
        else:
            lo, hi = last_pass, nxt  # lo passes, hi fails
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if ok(mid):
                    lo = mid
                else:
                    hi = mid
            return lo
    return last_pass
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run --extra dev pytest tests/bench/test_sweep.py -v`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add bench/sweep.py tests/bench/test_sweep.py
git commit -m "feat(bench): capacity-frontier knee search (probe + bisect)"
```

---

### Task 5: Frontier rendering (`bench/frontier.py`)

**Files:**
- Create: `bench/frontier.py`
- Test: `tests/bench/test_frontier.py`

**Interfaces:**
- Consumes: a list of run-result rows, each `{"backend": str, "resolution": str, "knee_n": int, "limiting": str}`.
- Produces:
  - `build_frontier_table(rows: list[dict]) -> dict` → `{backend: {resolution: {"n": int, "limit": str}}}`.
  - `render_markdown(table: dict, backends: list[str], resolutions: list[str]) -> str` → a Markdown table, rows = resolutions, cols = backends, cell = `"<n> (<limit>)"`, missing cell = `"—"`.

- [ ] **Step 1: Write the failing tests**

Create `tests/bench/test_frontier.py`:

```python
from bench import frontier


ROWS = [
    {"backend": "cpu", "resolution": "640x360", "knee_n": 2, "limiting": "cpu"},
    {"backend": "intel", "resolution": "640x360", "knee_n": 10, "limiting": "igpu"},
    {"backend": "nvidia", "resolution": "640x360", "knee_n": 14, "limiting": "cpu"},
    {"backend": "nvidia", "resolution": "1920x1080", "knee_n": 4, "limiting": "vram"},
]


def test_build_frontier_table():
    t = frontier.build_frontier_table(ROWS)
    assert t["intel"]["640x360"] == {"n": 10, "limit": "igpu"}
    assert t["nvidia"]["1920x1080"] == {"n": 4, "limit": "vram"}


def test_render_markdown_has_cells_and_dash():
    t = frontier.build_frontier_table(ROWS)
    md = frontier.render_markdown(
        t, backends=["cpu", "intel", "nvidia"],
        resolutions=["640x360", "1920x1080"])
    assert "640x360" in md
    assert "10 (igpu)" in md
    assert "4 (vram)" in md
    assert "—" in md          # cpu @ 1920x1080 has no row
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run --extra dev pytest tests/bench/test_frontier.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bench.frontier'`.

- [ ] **Step 3: Implement `bench/frontier.py`**

```python
"""Turn per-run knee results into the capacity-frontier table (Markdown) and,
when matplotlib is available, a heatmap PNG.
"""


def build_frontier_table(rows: list[dict]) -> dict:
    table: dict = {}
    for r in rows:
        table.setdefault(r["backend"], {})[r["resolution"]] = {
            "n": r["knee_n"], "limit": r["limiting"]}
    return table


def render_markdown(table: dict, backends: list[str], resolutions: list[str]) -> str:
    header = "| resolution | " + " | ".join(backends) + " |"
    sep = "|" + "---|" * (len(backends) + 1)
    lines = [header, sep]
    for res in resolutions:
        cells = []
        for b in backends:
            cell = table.get(b, {}).get(res)
            cells.append(f"{cell['n']} ({cell['limit']})" if cell else "—")
        lines.append(f"| {res} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def render_heatmap(table: dict, backends: list[str], resolutions: list[str], path: str) -> bool:
    """Write a max-N heatmap PNG. Returns False if matplotlib is unavailable."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False
    grid = [[(table.get(b, {}).get(res) or {}).get("n", 0) for b in backends]
            for res in resolutions]
    fig, ax = plt.subplots(figsize=(1.6 * len(backends) + 2, 0.7 * len(resolutions) + 2))
    im = ax.imshow(grid, cmap="viridis", aspect="auto")
    ax.set_xticks(range(len(backends)), backends)
    ax.set_yticks(range(len(resolutions)), resolutions)
    for y, res in enumerate(resolutions):
        for x, b in enumerate(backends):
            cell = table.get(b, {}).get(res)
            ax.text(x, y, "—" if not cell else f"{cell['n']}\n{cell['limit']}",
                    ha="center", va="center", color="w", fontsize=8)
    ax.set_title("Max sustainable drones")
    fig.colorbar(im, ax=ax, label="drones")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return True
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run --extra dev pytest tests/bench/test_frontier.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add bench/frontier.py tests/bench/test_frontier.py
git commit -m "feat(bench): frontier table (markdown) + heatmap rendering"
```

---

### Task 6: Host sampler CLI (`bench/sample_host.py`)

**Files:**
- Create: `bench/sample_host.py`

**Interfaces:**
- Consumes: `bench.probes.parse_nvidia_smi`, `parse_intel_gpu_top`, `parse_docker_stats`.
- Produces: when run, appends one JSON object per ~1 s to a JSONL file:
  `{"t": <epoch float>, "cpu_pct", "load1", "ram_used_gb", "nvidia": {...}|{}, "intel": {...}|{}, "container": {...}|{}}`.
  Run as `python -m bench.sample_host --out <path> [--container swarm-multi] [--interval 1.0]`.
- This is integration glue: no unit test (its pure parsing is covered in Task 3). Verified by running it and inspecting output.

- [ ] **Step 1: Implement `bench/sample_host.py`**

```python
"""~1 Hz host-resource sampler. Shells out to nvidia-smi / intel_gpu_top /
docker stats, parses with bench.probes, and appends JSONL. Runs on the HOST
(GPUs are host devices), alongside the benchmarked container.
"""
import argparse
import json
import subprocess
import time

from bench import probes


def _run(cmd: list[str], timeout: float = 4.0) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except Exception:
        return ""


def _cpu_ram() -> dict:
    try:
        import psutil
        return {"cpu_pct": psutil.cpu_percent(interval=None),
                "load1": psutil.getloadavg()[0],
                "ram_used_gb": (psutil.virtual_memory().total - psutil.virtual_memory().available) / 1e9}
    except Exception:
        # /proc fallback for load + mem; cpu_pct best-effort 0.
        load1 = 0.0
        try:
            with open("/proc/loadavg") as f:
                load1 = float(f.read().split()[0])
        except Exception:
            pass
        used = 0.0
        try:
            mem = {}
            with open("/proc/meminfo") as f:
                for line in f:
                    k, v = line.split(":")
                    mem[k] = float(v.strip().split()[0])  # kB
            used = (mem.get("MemTotal", 0) - mem.get("MemAvailable", 0)) / 1e6
        except Exception:
            pass
        return {"cpu_pct": 0.0, "load1": load1, "ram_used_gb": used}


def _nvidia() -> dict:
    out = _run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,power.draw",
                "--format=csv,noheader,nounits"])
    line = out.strip().splitlines()[0] if out.strip() else ""
    return probes.parse_nvidia_smi(line) if line else {}


def _intel() -> dict:
    # one ~500ms sample; intel_gpu_top -J streams a JSON array, take the first object.
    out = _run(["sudo", "-n", "intel_gpu_top", "-J", "-s", "500", "-o", "-"], timeout=4.0)
    out = out.strip().lstrip("[").strip()
    if not out:
        return {}
    # take text up to the first top-level closing brace
    depth = 0
    end = -1
    for idx, ch in enumerate(out):
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                end = idx + 1
                break
    if end == -1:
        return {}
    try:
        return probes.parse_intel_gpu_top(out[:end])
    except Exception:
        return {}


def _container(name: str) -> dict:
    out = _run(["docker", "stats", "--no-stream", "--format", "{{.CPUPerc}} {{.MemUsage}}", name])
    line = out.strip().splitlines()[0] if out.strip() else ""
    return probes.parse_docker_stats(line) if line else {}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--container", default="swarm-multi")
    ap.add_argument("--interval", type=float, default=1.0)
    args = ap.parse_args()
    try:
        import psutil
        psutil.cpu_percent(interval=None)  # prime the first delta
    except Exception:
        pass
    with open(args.out, "a") as f:
        while True:
            sample = {"t": time.time(), **_cpu_ram(),
                      "nvidia": _nvidia(), "intel": _intel(),
                      "container": _container(args.container)}
            f.write(json.dumps(sample) + "\n")
            f.flush()
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify it samples real tools (manual integration)**

Run (host): `timeout 6 uv run --extra dev python -m bench.sample_host --out /tmp/bench-sample.jsonl --interval 1.0 ; cat /tmp/bench-sample.jsonl`
Expected: ~5 JSONL lines; each has a `nvidia` dict with non-zero `mem_used_mb` (the dGPU always reports), `cpu_pct`/`ram_used_gb` populated. `intel`/`container` may be `{}` if `intel_gpu_top` needs an interactive sudo or no container is running — that's acceptable (FPS gate is independent). If `intel` is `{}`, run `sudo -v` first (or add a NOPASSWD sudoers entry for `intel_gpu_top`) and re-check that `render_pct` then appears.

- [ ] **Step 3: Commit**

```bash
git add bench/sample_host.py
git commit -m "feat(bench): ~1Hz host resource sampler CLI"
```

---

### Task 7: Parametrize the sim launch (`RENDER_BACKEND`, `CAM_W/H/FPS`)

**Files:**
- Modify: `sim/launch/swarm_sim.sh:9-20` (backend selector) and `:74-82` (OakD sed patch)
- Modify: `scripts/run_swarm_demo.sh:16-54` (backend → docker args; forward `CAM_*`)

**Interfaces:**
- Produces: both scripts honour `RENDER_BACKEND={cpu|intel|nvidia}` and `CAM_W`/`CAM_H`/`CAM_FPS`. Defaults preserve current behaviour exactly (`intel`-equivalent when `GPU=1`, `640`/`360`/`10`). Consumed by Task 8's orchestrator.

- [ ] **Step 1: Apply the `RENDER_BACKEND` selector in `swarm_sim.sh`**

Replace the block at `sim/launch/swarm_sim.sh:9-20` (the `if [ "${GPU_RENDER:-0}" = "1" ]…` block) with the `RENDER_BACKEND` case from `docs/nvidia-render-investigation.md` (the `### sim/launch/swarm_sim.sh` diff). Keep the back-compat line so `GPU_RENDER=1` with no `RENDER_BACKEND` ⇒ `intel`.

- [ ] **Step 2: Parametrize the camera resolution/rate in `swarm_sim.sh`**

Replace the hardcoded OakD sed patch at `sim/launch/swarm_sim.sh:74-82` with `CAM_*`-driven values:

```bash
CAM_W="${CAM_W:-640}"; CAM_H="${CAM_H:-360}"; CAM_FPS="${CAM_FPS:-10}"
OAKD="Tools/simulation/gz/models/OakD-Lite/model.sdf"
if [ -f "$OAKD" ] && grep -q "<topic>camera</topic>" "$OAKD"; then
  sed -i \
    -e "s|<width>1920</width>|<width>${CAM_W}</width>|" \
    -e "s|<height>1080</height>|<height>${CAM_H}</height>|" \
    -e "s|<update_rate>30</update_rate>|<update_rate>${CAM_FPS}</update_rate>|g" \
    -e "/<topic>camera<\/topic>/d" \
    "$OAKD"
fi
```

Note: the sed only fires once (it's gated on the original `<width>1920</width>` / `<topic>camera</topic>` still being present). The orchestrator (Task 8) restores a pristine `model.sdf` before each run via `git -C PX4-Autopilot checkout` so a new `CAM_*` actually takes effect — call that out in Task 8.

- [ ] **Step 3: Wire `RENDER_BACKEND` → docker args in `run_swarm_demo.sh`**

Apply the `### scripts/run_swarm_demo.sh` diff from `docs/nvidia-render-investigation.md`, but use the **clean toolkit path** for `nvidia` (the toolkit is now installed), i.e. the `nvidia)` case is simply:

```bash
  nvidia)
    GPU_ARGS=(--gpus all
              -e NVIDIA_VISIBLE_DEVICES=all
              -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility,display
              -e RENDER_BACKEND=nvidia
              -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
              -e PX4_MODEL=gz_x500_depth)
    ;;
```

Then forward the camera knobs by adding `-e CAM_W="$CAM_W" -e CAM_H="$CAM_H" -e CAM_FPS="$CAM_FPS"` to the `docker run` invocation (near the other `-e SWARM_N=…` flags at `scripts/run_swarm_demo.sh:53`), with defaults `CAM_W="${CAM_W:-640}"` etc. defined near the top.

- [ ] **Step 4: Lint both scripts**

Run: `bash -n sim/launch/swarm_sim.sh && bash -n scripts/run_swarm_demo.sh && shellcheck sim/launch/swarm_sim.sh scripts/run_swarm_demo.sh || true`
Expected: `bash -n` clean (no syntax errors); shellcheck warnings acceptable but review any new error.

- [ ] **Step 5: Verify each backend brings up one drone with a live camera (manual integration, time-boxed)**

For each `B` in `cpu intel nvidia`:

```bash
RENDER_BACKEND=$B CAM_W=640 CAM_H=360 CAM_FPS=10 WORLD=baylands timeout 600 ./scripts/run_swarm_demo.sh 1
# wait for "sim ready", then:
docker exec swarm-multi bash -lc 'ros2 topic list | grep -c IMX214/image'   # expect >=1
docker exec swarm-multi bash -lc 'gz topic -e -t /world/baylands/stats -n 1 | grep real_time_factor'  # expect ~1.0
# for nvidia, confirm dGPU attribution:
[ "$B" = nvidia ] && nvidia-smi | grep -iE 'gz|px4|ogre' && echo "dGPU OK"
docker rm -f swarm-multi
```

Expected: each backend reaches "sim ready", publishes ≥1 `IMX214/image` topic, RTF ≈ 1.0 at N=1; `nvidia` shows a gz/px4 process holding GPU memory in `nvidia-smi`. Hard-capped at 10 min per backend; if `cpu` (llvmpipe) is too slow even at N=1, record that and continue (it's a legitimate low-capacity result, not a harness bug).

- [ ] **Step 6: Confirm the demo default is unchanged**

Run: `./scripts/run_swarm_demo.sh 1` (no new env) and confirm it uses the intel iGPU path (640×360@10) exactly as before; then `docker rm -f swarm-multi`.

- [ ] **Step 7: Commit**

```bash
git add sim/launch/swarm_sim.sh scripts/run_swarm_demo.sh
git commit -m "feat(sim): RENDER_BACKEND={cpu,intel,nvidia} + CAM_W/H/FPS knobs"
```

---

### Task 8: Orchestrator (`bench/run_bench.py`) — single run, sweep, `--smoke`

**Files:**
- Create: `bench/run_bench.py`
- Test: `tests/bench/test_run_bench.py` (pure helpers only)

**Interfaces:**
- Consumes: `bench.metrics`, `bench.probes`, `bench.sweep`, `bench.frontier`; the scripts from Task 7; the `/state` `cam_seq` from Task 1.
- Produces:
  - pure helper `slice_samples(samples: list[dict], t0: float, t1: float) -> list[dict]` → samples with `t0 ≤ t < t1`.
  - pure helper `peak_sample(samples: list[dict]) -> dict` → the sample maximizing CPU/GPU load (used for limiting-resource); empty → `{}`.
  - CLI: `python -m bench.run_bench [--smoke] [--backends cpu,intel,nvidia] [--resolutions 320x180,…] [--n-cap 32] [--settle 30] [--measure 60] [--out docs/benchmarks/<ts>]`.

- [ ] **Step 1: Write the failing tests (pure helpers)**

Create `tests/bench/test_run_bench.py`:

```python
from bench import run_bench


def test_slice_samples_window():
    s = [{"t": 0.0}, {"t": 5.0}, {"t": 9.9}, {"t": 10.0}, {"t": 12.0}]
    out = run_bench.slice_samples(s, 5.0, 10.0)
    assert [x["t"] for x in out] == [5.0, 9.9]


def test_peak_sample_picks_busiest():
    s = [
        {"cpu_pct": 10.0, "nvidia": {"util": 5.0, "mem_used_mb": 100.0}, "intel": {}},
        {"cpu_pct": 80.0, "nvidia": {"util": 90.0, "mem_used_mb": 200.0}, "intel": {}},
    ]
    assert run_bench.peak_sample(s)["cpu_pct"] == 80.0


def test_peak_sample_empty():
    assert run_bench.peak_sample([]) == {}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run --extra dev pytest tests/bench/test_run_bench.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bench.run_bench'`.

- [ ] **Step 3: Implement `bench/run_bench.py`**

```python
"""Capacity-frontier benchmark orchestrator. For each (backend, resolution)
cell, ramps N to the knee via bench.sweep, driving the existing Docker swarm
stack one run at a time, and writes per-run JSON + a frontier table.

A run: clean container -> bring up (run_swarm_demo.sh with RENDER_BACKEND/CAM_*)
-> wait sim-ready -> send one warm-up command -> settle -> measure FPS+RTF+liveness
+ host headroom -> verdict -> teardown.
"""
import argparse
import json
import os
import subprocess
import time
import urllib.request

from bench import metrics, probes, sweep, frontier

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_RES = ["320x180", "640x360", "960x540", "1280x720", "1920x1080"]
DEFAULT_BACKENDS = ["cpu", "intel", "nvidia"]
WARMUP = "everyone take off, climb to 20 m, and orbit"
WORLD = "baylands"
OBS = "http://localhost:8000"


# ---- pure helpers (unit-tested) ----
def slice_samples(samples: list[dict], t0: float, t1: float) -> list[dict]:
    return [s for s in samples if t0 <= s.get("t", 0.0) < t1]


def _load(s: dict) -> float:
    nv = s.get("nvidia") or {}
    return max(s.get("cpu_pct", 0.0), nv.get("util", 0.0),
              (s.get("intel") or {}).get("render_pct", 0.0))


def peak_sample(samples: list[dict]) -> dict:
    return max(samples, key=_load) if samples else {}


# ---- glue ----
def _sh(cmd: str, timeout: float = 60.0) -> str:
    return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                          timeout=timeout, cwd=ROOT).stdout


def _state() -> dict | None:
    try:
        with urllib.request.urlopen(OBS + "/state", timeout=5) as r:
            return json.loads(r.read())
    except Exception:
        return None


def _wait_ready(n: int, deadline: float) -> bool:
    """Sim + observatory ready: /state returns n drones with telemetry + cam_seq>0."""
    while time.time() < deadline:
        st = _state()
        if st and st.get("n") == n:
            ds = st["drones"]
            if all(d.get("cam_seq", 0) > 0 for d in ds):
                return True
        time.sleep(5)
    return False


def _command(text: str) -> None:
    data = json.dumps({"text": text}).encode()
    try:
        urllib.request.urlopen(
            urllib.request.Request(OBS + "/command", data=data,
                                   headers={"Content-Type": "application/json"}),
            timeout=5)
    except Exception:
        pass


def _rtf() -> float:
    out = _sh(f"docker exec swarm-multi bash -lc "
              f"'gz topic -e -t /world/{WORLD}/stats -n 1' 2>/dev/null", timeout=20)
    return probes.parse_gz_rtf(out)


def run_one(backend: str, res: str, n: int, settle: float, measure: float,
            outdir: str, cam_fps: int = 10) -> dict:
    """Bring up, measure, tear down. Returns a result dict with 'verdict'."""
    w, h = res.split("x")
    _sh("docker rm -f swarm-multi 2>/dev/null || true", timeout=60)
    # pristine camera model so a new CAM_* actually applies (the sed is one-shot)
    _sh("git -C PX4-Autopilot checkout -- "
        "Tools/simulation/gz/models/OakD-Lite/model.sdf 2>/dev/null || true", timeout=30)

    sample_path = os.path.join(outdir, f"samples-{backend}-{res}-{n}.jsonl")
    sampler = subprocess.Popen(
        ["python", "-m", "bench.sample_host", "--out", sample_path],
        cwd=ROOT)
    try:
        env = (f"RENDER_BACKEND={backend} CAM_W={w} CAM_H={h} CAM_FPS={cam_fps} "
               f"WORLD={WORLD}")
        _sh(f"{env} timeout 600 ./scripts/run_swarm_demo.sh {n}", timeout=620)
        if not _wait_ready(n, time.time() + 300):
            return {"backend": backend, "resolution": res, "n": n,
                    "verdict": {"pass": False, "reasons": ["infra: not ready"]},
                    "infra_fail": True}
        _command(WARMUP)
        time.sleep(settle)

        st0 = _state() or {"drones": []}
        seq0 = {d["id"]: d.get("cam_seq", 0) for d in st0["drones"]}
        t0 = time.time()
        time.sleep(measure)
        st1 = _state() or {"drones": []}
        seq1 = {d["id"]: d.get("cam_seq", 0) for d in st1["drones"]}
        dt = time.time() - t0

        fps = metrics.compute_fps(seq0, seq1, dt)
        fsum = metrics.fps_summary(fps)
        rtf = _rtf()
        alive = sum(1 for d in st1["drones"] if d.get("armed") is True)

        samples = [json.loads(l) for l in open(sample_path)] if os.path.exists(sample_path) else []
        window = slice_samples(samples, t0, t0 + dt)
        peak = peak_sample(window)
        limit = probes.limiting_resource(peak) if peak else "none"

        verdict = metrics.evaluate_verdict(fsum["min"], cam_fps, rtf, alive, n)
        return {"backend": backend, "resolution": res, "n": n,
                "fps": fsum, "rtf": rtf, "alive": alive,
                "limiting": limit, "peak": peak, "verdict": verdict}
    finally:
        sampler.terminate()
        _sh("docker rm -f swarm-multi 2>/dev/null || true", timeout=60)


def run_sweep(backends, resolutions, n_cap, settle, measure, outdir, cam_fps=10):
    os.makedirs(outdir, exist_ok=True)
    rows = []
    runs_csv = os.path.join(outdir, "runs.csv")
    with open(runs_csv, "w") as f:
        f.write("backend,resolution,n,fps_min,rtf,alive,limiting,verdict\n")
    for backend in backends:
        seed = 1
        for res in resolutions:                    # ascending res -> capacity non-increasing
            def passes(n, _b=backend, _r=res):
                r = run_one(_b, _r, n, settle, measure, outdir, cam_fps)
                with open(os.path.join(outdir, f"run-{_b}-{_r}-{n}.json"), "w") as jf:
                    json.dump(r, jf, indent=2)
                v = r["verdict"]
                fsum = r.get("fps", {})
                with open(runs_csv, "a") as cf:
                    cf.write(f"{_b},{_r},{n},{fsum.get('min',0):.2f},"
                             f"{r.get('rtf',0):.2f},{r.get('alive',0)},"
                             f"{r.get('limiting','')},{'PASS' if v['pass'] else 'FAIL'}\n")
                return v["pass"]
            knee = sweep.find_knee(passes, n_cap=n_cap, seed=seed)
            seed = max(1, knee)                    # seed next (heavier) resolution from here
            # limiting resource at the knee: re-read the knee run if present
            kr_path = os.path.join(outdir, f"run-{backend}-{res}-{knee}.json")
            limiting = "none"
            if os.path.exists(kr_path):
                limiting = json.load(open(kr_path)).get("limiting", "none")
            rows.append({"backend": backend, "resolution": res,
                         "knee_n": knee, "limiting": limiting})
    table = frontier.build_frontier_table(rows)
    md = frontier.render_markdown(table, backends, resolutions)
    with open(os.path.join(outdir, "frontier.md"), "w") as f:
        f.write("# Capacity frontier — max sustainable drones\n\n" + md)
    frontier.render_heatmap(table, backends, resolutions,
                            os.path.join(outdir, "frontier.png"))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true",
                    help="fixed N=8, sweep resolutions per backend (harness validation)")
    ap.add_argument("--backends", default=",".join(DEFAULT_BACKENDS))
    ap.add_argument("--resolutions", default=",".join(DEFAULT_RES))
    ap.add_argument("--n-cap", type=int, default=32)
    ap.add_argument("--settle", type=float, default=30.0)
    ap.add_argument("--measure", type=float, default=60.0)
    ap.add_argument("--cam-fps", type=int, default=10)
    ap.add_argument("--out", default=os.path.join("docs", "benchmarks", "run"))
    args = ap.parse_args()
    backends = args.backends.split(",")
    resolutions = args.resolutions.split(",")
    os.makedirs(os.path.join(ROOT, args.out), exist_ok=True)
    outdir = os.path.join(ROOT, args.out)

    if args.smoke:
        # one fixed N across the resolution sweep, no knee search
        rows = []
        for backend in backends:
            for res in resolutions:
                r = run_one(backend, res, 8, args.settle, args.measure, outdir, args.cam_fps)
                with open(os.path.join(outdir, f"smoke-{backend}-{res}.json"), "w") as jf:
                    json.dump(r, jf, indent=2)
                rows.append({"backend": backend, "resolution": res,
                             "knee_n": 8 if r["verdict"]["pass"] else 0,
                             "limiting": r.get("limiting", "none")})
                print(f"[smoke] {backend} {res} N=8 -> "
                      f"{'PASS' if r['verdict']['pass'] else 'FAIL'} {r.get('fps',{})}")
        return
    run_sweep(backends, resolutions, args.n_cap, args.settle, args.measure,
              outdir, args.cam_fps)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the pure-helper tests to verify they pass**

Run: `uv run --extra dev pytest tests/bench/test_run_bench.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add bench/run_bench.py tests/bench/test_run_bench.py
git commit -m "feat(bench): capacity-frontier orchestrator (run/sweep/smoke)"
```

---

### Task 9: End-to-end smoke validation

**Files:** none (validation only; may add a short `bench/README.md`).

**Interfaces:** exercises Tasks 1–8 together against the live stack.

- [ ] **Step 1: Run a single minimal cell end-to-end**

Run (host, time-boxed): a one-cell smoke — `intel` backend, one resolution, small N, short windows:

```bash
uv run --extra dev python -m bench.run_bench --smoke \
  --backends intel --resolutions 640x360 \
  --settle 15 --measure 20 --out docs/benchmarks/smoke
```

Expected: prints `[smoke] intel 640x360 N=8 -> PASS|FAIL {fps...}` within ~10 min; `docs/benchmarks/smoke/` contains `smoke-intel-640x360.json` and a `samples-*.jsonl` with non-empty windows. `swarm-multi` is removed afterward (no leftover container — verify `docker ps` is clean).

- [ ] **Step 2: Sanity-check the recorded numbers**

Open `docs/benchmarks/smoke/smoke-intel-640x360.json` and confirm: `fps.min` is a plausible number (near 9–10 if passing, lower if the host is loaded), `rtf` populated, `alive` ≤ 8, `limiting` ∈ {cpu,igpu,dgpu,vram,ram,none}. If `fps.min == 0`, the `/state` `cam_seq` wiring (Task 1) or `_wait_ready` is wrong — debug before proceeding.

- [ ] **Step 3: Write a short `bench/README.md`**

```markdown
# Swarm capacity benchmark

Measures max sustainable drones per render backend × camera resolution.
Metric: min per-drone delivered camera FPS >= 0.9x rate AND RTF >= 0.9 AND all drones alive.

## Run
- Smoke (validate harness, ~1h): `uv run --extra dev python -m bench.run_bench --smoke --out docs/benchmarks/smoke`
- Full sweep (~10h, overnight): `uv run --extra dev python -m bench.run_bench --out docs/benchmarks/$(date +%Y%m%d)`

Outputs: `runs.csv`, `run-*.json`, `frontier.md`, `frontier.png` under the `--out` dir.
Requires the `dronebot-swarm:dev` image and (for the iGPU headroom column) passwordless `intel_gpu_top` sudo.
```

- [ ] **Step 4: Commit**

```bash
git add bench/README.md docs/benchmarks/smoke
git commit -m "test(bench): end-to-end smoke validation + README"
```

---

## Self-Review

**Spec coverage:**
- §2 metric (FPS primary + RTF co-gate + liveness) → Task 2 `evaluate_verdict`, Task 8 `run_one`. ✓
- §2 headroom snapshot + limiting resource → Task 3 `limiting_resource`, Task 6 sampler, Task 8 `peak_sample`. ✓
- §4.1 parametrize launch (RENDER_BACKEND, CAM_*) → Task 7. ✓
- §4.2 delivered-FPS counter on /state → Task 1 (reuses existing `GzCameras.seq`). ✓
- §4.3 host sampler → Task 6. ✓
- §4.4 orchestrator (bring-up, warm-up command, settle/measure, teardown, time-box) → Task 8. ✓
- §4.5 capacity-frontier probe+bisect, neighbour-seeded → Task 4 + Task 8 `run_sweep`. ✓
- §4.6 frontier table + heatmap → Task 5. ✓
- §5 defaults (resolutions, N cap 32, 30/60 windows, baylands, `--smoke`) → Task 8 CLI defaults. ✓
- §6 FAIL-INFRA vs capacity FAIL distinction → Task 8 `infra_fail` branch. ✓

**Placeholder scan:** no TBD/TODO; every code step shows complete code; glue tasks (6, 7, 8-glue, 9) use explicit verification commands with expected output in lieu of unit tests, with pure helpers extracted and unit-tested.

**Type consistency:** `cam_seq` (Task 1) read as `d["cam_seq"]` (Task 8). `find_knee(pass_fn, n_cap, seed)` defined Task 4, called Task 8. `limiting_resource(sample)` shape matches the sampler dict (Task 6) and `peak_sample` output (Task 8). `compute_fps/fps_summary/evaluate_verdict` signatures consistent across Tasks 2 and 8. `build_frontier_table`/`render_markdown` rows shape `{backend,resolution,knee_n,limiting}` consistent across Tasks 5 and 8.

**Note on RTF in smoke mode:** `run_one` always computes `rtf` via `_rtf()`, so `--smoke` rows reflect the full gate. ✓
