# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w1_split_reach | drones=haiku | 0/2 [0%–66%] | 50% | 6.7s (5.3–8.2) | - | 13.0 | step budget exceeded×1, wall-clock deadline×1 |
| w1_split_reach | drones=opus | 2/2 [34%–100%] | 100% | 8.1s (5.7–10.5) | 7.0 | - | - |
| w1_split_reach | drones=sonnet | 1/2 [9%–91%] | 83% | 5.4s (4.9–5.8) | 9.0 | 13.0 | wall-clock deadline×1 |
| w2_allocation | drones=haiku | 2/2 [34%–100%] | 100% | 12.2s (11.1–13.3) | 13.0 | - | - |
| w2_allocation | drones=opus | 2/2 [34%–100%] | 100% | 11.2s (10.3–12.2) | 12.0 | - | - |
| w2_allocation | drones=sonnet | 2/2 [34%–100%] | 100% | 12.9s (12.3–13.5) | 11.5 | - | - |
| w3_crossing | drones=haiku | 0/2 [0%–66%] | 40% | 8.1s (7.0–9.2) | - | 15.0 | step budget exceeded×2 |
| w3_crossing | drones=opus | 2/2 [34%–100%] | 100% | 10.4s (8.7–12.1) | 14.0 | - | - |
| w3_crossing | drones=sonnet | 0/2 [0%–66%] | 50% | 5.3s (4.5–6.1) | - | 15.0 | step budget exceeded×2 |
| w4_double_intercept | drones=haiku | 0/2 [0%–66%] | 33% | 5.6s (5.2–6.0) | - | 23.0 | step budget exceeded×2 |
| w4_double_intercept | drones=opus | 0/2 [0%–66%] | 50% | 8.4s (7.0–9.9) | - | 23.0 | step budget exceeded×2 |
| w4_double_intercept | drones=sonnet | 0/2 [0%–66%] | 33% | 4.6s (4.4–4.9) | - | 19.5 | wall-clock deadline×1, step budget exceeded×1 |
| w5_sync_mark | drones=haiku | 2/2 [34%–100%] | 100% | 5.4s (4.9–6.0) | 16.0 | - | - |
| w5_sync_mark | drones=opus | 2/2 [34%–100%] | 100% | 6.7s (5.8–7.5) | 12.5 | - | - |
| w5_sync_mark | drones=sonnet | 1/2 [9%–91%] | 83% | 6.3s (5.9–6.7) | 13.0 | 17.0 | step budget exceeded×1 |
