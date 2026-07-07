# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d4_estimate_intercept | drones=haiku | 0/2 [0%–66%] | 75% | 4.0s (3.7–4.4) | - | 11.5 | oracle check failed×2 |
| d4_estimate_intercept | drones=opus | 0/2 [0%–66%] | 75% | 5.1s (4.3–5.9) | - | 8.5 | oracle check failed×2 |
| d4_estimate_intercept | drones=sonnet | 0/2 [0%–66%] | 75% | 4.7s (3.8–5.5) | - | 6.0 | oracle check failed×2 |
