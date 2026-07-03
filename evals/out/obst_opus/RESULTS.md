# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c4_obstacle_run | drones=opus | 2/2 [34%–100%] | 100% | 3.2s (3.1–3.3) | 8.5 | - | - |
| o1_detour | drones=opus | 1/2 [9%–91%] | 80% | 4.0s (3.4–4.7) | 8.0 | 13.0 | step budget exceeded×1 |
| o2_slalom | drones=opus | 1/2 [9%–91%] | 90% | 3.3s (2.7–3.8) | 13.0 | 15.0 | step budget exceeded×1 |
| o3_inspect | drones=opus | 2/2 [34%–100%] | 100% | 4.2s (3.7–4.7) | 11.5 | - | - |
