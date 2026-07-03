# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c4_obstacle_run | drones=sonnet | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
| o1_detour | drones=sonnet | 0/2 [0%–66%] | 40% | 2.6s (2.3–2.9) | - | 11.0 | step budget exceeded×2 |
| o2_slalom | drones=sonnet | 0/1 [0%–79%] | 40% | 2.9s (2.9–2.9) | - | 13.0 | step budget exceeded×1 |
| o3_inspect | drones=sonnet | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
