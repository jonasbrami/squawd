# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c4_obstacle_run | drones=sonnet | 0/2 [0%–66%] | 43% | 3.6s (3.6–3.6) | - | 17.0 | step budget exceeded×2 |
| o1_detour | drones=sonnet | 0/2 [0%–66%] | 60% | 3.9s (2.8–4.9) | - | 13.0 | step budget exceeded×2 |
| o2_slalom | drones=sonnet | 0/2 [0%–66%] | 80% | 2.6s (2.5–2.7) | - | 15.0 | step budget exceeded×2 |
| o3_inspect | drones=sonnet | 1/2 [9%–91%] | 80% | 4.8s (4.5–5.1) | 12.0 | 15.0 | step budget exceeded×1 |
