# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c4_obstacle_run | drones=haiku | 1/2 [9%–91%] | 93% | 5.2s (3.9–6.5) | 8.0 | 10.0 | oracle check failed×1 |
| o1_detour | drones=haiku | 1/2 [9%–91%] | 90% | 3.9s (3.6–4.2) | 9.0 | 12.0 | oracle check failed×1 |
| o2_slalom | drones=haiku | 0/1 [0%–79%] | 60% | 4.5s (4.5–4.5) | - | 15.0 | step budget exceeded×1 |
| o3_inspect | drones=haiku | 0/2 [0%–66%] | 60% | 3.9s (3.6–4.2) | - | 15.0 | step budget exceeded×2 |
