# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w1_split_reach | drones=sonnet | 1/2 [9%–91%] | 83% | 5.4s (4.9–5.8) | 9.0 | 13.0 | wall-clock deadline×1 |
| w2_allocation | drones=sonnet | 2/2 [34%–100%] | 100% | 12.9s (12.3–13.5) | 11.5 | - | - |
| w3_crossing | drones=sonnet | 0/2 [0%–66%] | 50% | 5.3s (4.5–6.1) | - | 15.0 | step budget exceeded×2 |
| w4_double_intercept | drones=sonnet | 0/2 [0%–66%] | 33% | 4.6s (4.4–4.9) | - | 19.5 | wall-clock deadline×1, step budget exceeded×1 |
| w5_sync_mark | drones=sonnet | 1/2 [9%–91%] | 83% | 6.3s (5.9–6.7) | 13.0 | 17.0 | step budget exceeded×1 |
