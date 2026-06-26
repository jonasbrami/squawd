# Swarm capacity benchmark

Measures max sustainable drones per render backend × camera resolution.
Metric: min per-drone delivered camera FPS >= 0.9x rate AND RTF >= 0.9 AND all drones alive.

## Run
- Smoke (validate harness, ~1h): `uv run --extra dev python -m bench.run_bench --smoke --out docs/benchmarks/smoke`
- Full sweep (~10h, overnight): `uv run --extra dev python -m bench.run_bench --out docs/benchmarks/$(date +%Y%m%d)`

Outputs: `runs.csv`, `run-*.json`, `frontier.md`, `frontier.png` under the `--out` dir.
Requires the `squawd:dev` image and (for the iGPU headroom column) passwordless `intel_gpu_top` sudo.

## Known limitations
- `limiting_resource` attributes the bottleneck from *aggregate* host/GPU utilization. A Gazebo sim that is **single-thread-bound** (common at high drone counts) can collapse RTF (e.g. to 0.3) while no aggregate signal — host CPU%, container CPU/ncores, or load1/nproc — crosses saturation, so `limiting` reports `none`. Read it together with RTF: a FAIL with `limiting=none` **and** low RTF means the sim is single-thread/physics-bound, not starved of a poolable resource. All raw headroom (host CPU%, container CPU%, load1, both GPUs' util/VRAM) is preserved per-run in `samples-*.jsonl` for manual analysis.
