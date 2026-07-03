# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c3_constrained_survey | drones=haiku | 1/2 [9%–91%] | 83% | 35.0s (34.6–35.5) | 16.0 | 17.0 | step budget exceeded×1 |
| p6_tight_route | drones=haiku | 2/2 [34%–100%] | 100% | 43.6s (23.7–63.5) | 3.5 | - | - |
| p7_knapsack | drones=haiku | 2/2 [34%–100%] | 100% | 84.0s (66.0–102.0) | 9.5 | - | - |
