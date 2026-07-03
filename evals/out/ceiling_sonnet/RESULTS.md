# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c3_constrained_survey | drones=sonnet | 0/2 [0%–66%] | 75% | 61.3s (41.4–81.2) | - | 17.0 | step budget exceeded×2 |
| p6_tight_route | drones=sonnet | 2/2 [34%–100%] | 100% | 4.0s (3.6–4.4) | 7.5 | - | - |
| p7_knapsack | drones=sonnet | 2/2 [34%–100%] | 100% | 102.3s (88.2–116.4) | 9.0 | - | - |
