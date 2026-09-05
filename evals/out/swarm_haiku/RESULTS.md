# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w1_split_reach | drones=haiku | 0/2 [0%–66%] | 50% | 6.7s (5.3–8.2) | - | 13.0 | step budget exceeded×1, wall-clock deadline×1 |
| w2_allocation | drones=haiku | 2/2 [34%–100%] | 100% | 12.2s (11.1–13.3) | 13.0 | - | - |
| w3_crossing | drones=haiku | 0/2 [0%–66%] | 40% | 8.1s (7.0–9.2) | - | 15.0 | step budget exceeded×2 |
| w4_double_intercept | drones=haiku | 0/2 [0%–66%] | 33% | 5.6s (5.2–6.0) | - | 23.0 | step budget exceeded×2 |
| w5_sync_mark | drones=haiku | 2/2 [34%–100%] | 100% | 5.4s (4.9–6.0) | 16.0 | - | - |
