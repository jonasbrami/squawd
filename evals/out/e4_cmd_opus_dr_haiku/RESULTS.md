# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w1_split_reach | _layer=commander,commander=opus,drones=haiku | 0/2 [0%–66%] | 67% | 4.4s (3.9–4.9) | - | 6.0 | wall-clock deadline×2 |
| w2_allocation | _layer=commander,commander=opus,drones=haiku | 0/2 [0%–66%] | 75% | 7.0s (5.7–8.3) | - | 4.0 | wall-clock deadline×2 |
| w3_crossing | _layer=commander,commander=opus,drones=haiku | 1/2 [9%–91%] | 80% | 7.7s (6.1–9.4) | 7.0 | 4.0 | wall-clock deadline×1 |
| w5_sync_mark | _layer=commander,commander=opus,drones=haiku | 1/2 [9%–91%] | 83% | 3.8s (3.4–4.1) | 7.0 | 4.0 | wall-clock deadline×1 |
