# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w4_double_intercept | drones=haiku | 1/2 [9%–91%] | 83% | 51.6s (6.3–96.9) | 13.0 | 13.0 | wall-clock deadline×1 |
| w4_double_intercept | drones=opus | 1/2 [9%–91%] | 83% | 7.4s (7.2–7.5) | 19.0 | 10.0 | oracle check failed×1 |
| w4_double_intercept | drones=sonnet | 0/2 [0%–66%] | 33% | 4.7s (4.4–5.1) | - | 21.0 | wall-clock deadline×1, step budget exceeded×1 |
