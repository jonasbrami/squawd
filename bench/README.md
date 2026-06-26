# Swarm capacity benchmark

Measures max sustainable drones per render backend × camera resolution.
Metric: min per-drone delivered camera FPS >= 0.9x rate AND RTF >= 0.9 AND all drones alive.

## Run
- Smoke (validate harness, ~1h): `uv run --extra dev python -m bench.run_bench --smoke --out docs/benchmarks/smoke`
- Full sweep (~10h, overnight): `uv run --extra dev python -m bench.run_bench --out docs/benchmarks/$(date +%Y%m%d)`

Outputs: `runs.csv`, `run-*.json`, `frontier.md`, `frontier.png` under the `--out` dir.
Requires the `squawd:dev` image and (for the iGPU headroom column) passwordless `intel_gpu_top` sudo.
