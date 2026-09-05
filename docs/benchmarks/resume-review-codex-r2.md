1. **Verdict: run S6 through `run_evals`, after wiring detection; do not use the production pilot because it discards Result/usage events.**

Create `evals/tasks/perceive/s6_kimi_spike.yaml` with world `perceive`, prompt requiring exactly `take_off → scan → detect → report`, budget four tool calls, and `alive`/step-budget checks. Minimal wiring:

- Add `pipeline` to `Deps`.
- Pass the live pipeline from `run_evals.py`.
- In `FleetHarness.client_for`, construct `make_detect_text(deps.world, deps.bridge, deps.pipeline)` and pass it as `detect_text=`.

Then, in an Intel/NVIDIA `x500_depth` perceive container:

```bash
docker exec evals-sim bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/px4_ws/install/setup.bash; cd /workspace; PYTHONPATH=/workspace:$PYTHONPATH GZ_WORLD=perceive uv run --no-project --with onnxruntime python -m evals.run_evals --tasks evals/tasks/perceive/s6_kimi_spike.yaml --assignments drones=kimi --feed vision --backend onnx --k 1 --out evals/out/m6_s6_kimi'
```

2. **Verdict: M5 uses CPU + `gz_x500`; unmodified `doctor_sim.sh` does not apply because its camera requirement is irrelevant.**

```bash
docker rm -f evals-sim 2>/dev/null || true
docker run -d --name evals-sim \
  -e RENDER_BACKEND=cpu -e PX4_MODEL=gz_x500 \
  -e SWARM_N=1 -e PX4_GZ_WORLD=dynamic -e GZ_WORLD=dynamic \
  -v "$PWD:/workspace" squawd:dev bash -lc 'sim/launch/swarm_sim.sh'

docker exec evals-sim bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /opt/px4_ws/install/setup.bash
  pgrep -f MicroXRCEAgent &&
  timeout 5 gz topic -l | grep -q /world/dynamic &&
  timeout 20 bash -c "until ros2 topic list | grep -q vehicle_local_position; do sleep 1; done"
'

docker exec evals-sim bash -lc '
  source /opt/ros/jazzy/setup.bash; source /opt/px4_ws/install/setup.bash
  cd /workspace
  PYTHONPATH=/workspace:$PYTHONPATH GZ_WORLD=dynamic \
  uv run --no-project python -m evals.run_evals \
  --tasks evals/tasks/dynamic/d[1-5]_*.yaml \
  --pilot --feed truth --k 1 --seed 0 \
  --out evals/out/m5_truth_regression_20260728
'
```

3. **Verdict: `city` remains unvalidated, but it has been superseded by the usable flat `obstacles` world.**

[make_obstacles_world.py](/home/quenouille/drone/sim/worlds/make_obstacles_world.py) adds buildings to PX4’s flat default world. Historical evidence shows obstacle pilot 4/4, including o1, in [pilot_obstacle](/home/quenouille/drone/evals/out/pilot_obstacle/results.jsonl). The README is stale. Before Kimi, rerun o1 with `--pilot` on a fresh `PX4_GZ_WORLD=obstacles` container. If that fails reproducibly, fix the world or leave M6 partial; substitution would not satisfy §5.6, and an owner scope decision would be a documented deviation, not a pass.

4. **Verdict: budget 25 requests, about 90k input/5k output tokens; impose a 200k-input ceiling.**

The explicit pilot prompt plus MCP schemas are ~8.2k characters, approximately 2.5k tokens/request:

- S6: 5 requests × 2.5k + history ≈ 16k input.
- d2: 5 × 2.5k + history ≈ 17k.
- Perceive: planned 7 × 2.5k + history ≈ 26k.
- Obstacle: 8 × 2.5k + history ≈ 31k.

Total: **25 requests, ~90k input, ~5k output**; hidden CLI prompt overhead makes 200k the safe stop ceiling.

Kimi usage is known: S0 returned `usage={input_tokens:602, output_tokens:211}`, `num_turns=3`, and `api_ms=6331` in [M0-RESULTS.md](/home/quenouille/drone/spikes/M0-RESULTS.md). Missing instrumentation is quota-error classification/count and exact HTTP-request count—`num_turns` is only the best proxy. If usage is absent, record it as null, preserve `num_turns`, tool calls, API/wall latency and quota errors, plus a clearly labelled local characters/4 token estimate.

5. **Verdict: allow at most three genuine fix iterations; after the third non-convergent fix, circle-pause and seek owner deviation approval.**

Decision tree:

1. Initial failure → repeat once in a fresh container; diagnostic, not a fix iteration.
2. Fresh failure with continuous GzPoses truth and low dwell = real regression: truth-fed d2 bypasses camera, CV-EKF and ToF, while pre-M5 truth passed 2/2.
3. Apply at most three tested fixes, recording dwell after each.
4. If the third fix shows no measurable convergence, stop before a fourth, independent review, then either legitimate design-level fix or owner-accepted “M5 deviation.” Known M3a/perception dwell physics is not a valid explanation for failure of this truth-fed control.