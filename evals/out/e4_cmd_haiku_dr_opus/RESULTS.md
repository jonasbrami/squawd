# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w1_split_reach | _layer=commander,commander=haiku,drones=opus | 1/2 [9%–91%] | 83% | 5.9s (5.2–6.6) | 6.0 | 5.0 | wall-clock deadline×1 |
| w2_allocation | _layer=commander,commander=haiku,drones=opus | 1/2 [9%–91%] | 88% | 10.8s (8.6–12.9) | 4.0 | 3.0 | wall-clock deadline×1 |
| w3_crossing | _layer=commander,commander=haiku,drones=opus | 2/2 [34%–100%] | 100% | 10.7s (9.6–11.9) | 6.5 | - | - |
| w5_sync_mark | _layer=commander,commander=haiku,drones=opus | 2/2 [34%–100%] | 100% | 6.3s (5.8–6.8) | 5.0 | - | - |
