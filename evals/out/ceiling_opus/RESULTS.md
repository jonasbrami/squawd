# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c3_constrained_survey | drones=opus | 1/2 [9%–91%] | 92% | 24.2s (23.9–24.6) | 15.0 | 17.0 | step budget exceeded×1 |
| p6_tight_route | drones=opus | 2/2 [34%–100%] | 100% | 4.3s (3.3–5.2) | 7.0 | - | - |
| p7_knapsack | drones=opus | 2/2 [34%–100%] | 100% | 41.0s (37.4–44.5) | 8.0 | - | - |
