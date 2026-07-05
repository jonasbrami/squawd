# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w1_split_reach | _layer=commander,commander=haiku,drones=haiku | 1/2 [9%–91%] | 83% | 4.5s (3.8–5.1) | 4.0 | 4.0 | wall-clock deadline×1 |
| w2_allocation | _layer=commander,commander=haiku,drones=haiku | 0/2 [0%–66%] | 75% | 20.4s (19.8–21.0) | - | 6.5 | commander done×1, wall-clock deadline×1 |
| w3_crossing | _layer=commander,commander=haiku,drones=haiku | 0/2 [0%–66%] | 60% | 6.0s (4.8–7.1) | - | 4.0 | wall-clock deadline×2 |
| w5_sync_mark | _layer=commander,commander=haiku,drones=haiku | 0/2 [0%–66%] | 67% | 4.9s (4.5–5.2) | - | 5.0 | wall-clock deadline×2 |
