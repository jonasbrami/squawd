# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c4_obstacle_run | drones=haiku | 0/1 [0%–79%] | 43% | 4.3s (4.3–4.3) | - | 11.0 | wall-clock deadline×1 |
| o1_detour | drones=haiku | 1/1 [21%–100%] | 100% | 6.2s (6.2–6.2) | 8.0 | - | - |
| o2_slalom | drones=haiku | 0/1 [0%–79%] | 80% | 4.8s (4.8–4.8) | - | 11.0 | oracle check failed×1 |
