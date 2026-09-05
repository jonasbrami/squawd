# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w1_split_reach | drones=opus | 2/2 [34%–100%] | 100% | 8.1s (5.7–10.5) | 7.0 | - | - |
| w2_allocation | drones=opus | 2/2 [34%–100%] | 100% | 11.2s (10.3–12.2) | 12.0 | - | - |
| w3_crossing | drones=opus | 2/2 [34%–100%] | 100% | 10.4s (8.7–12.1) | 14.0 | - | - |
| w4_double_intercept | drones=opus | 0/2 [0%–66%] | 50% | 8.4s (7.0–9.9) | - | 23.0 | step budget exceeded×2 |
| w5_sync_mark | drones=opus | 2/2 [34%–100%] | 100% | 6.7s (5.8–7.5) | 12.5 | - | - |
