# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d2_shadow | drones=haiku | 0/2 [0%–66%] | 50% | 3.2s (2.7–3.6) | - | 27.0 | wall-clock deadline×1, step budget exceeded×1 |
| d2_shadow | drones=opus | 2/2 [34%–100%] | 100% | 2.7s (2.5–3.0) | 7.0 | - | - |
| d2_shadow | drones=sonnet | 2/2 [34%–100%] | 100% | 2.5s (2.2–2.7) | 11.0 | - | - |
| d4_estimate_intercept | drones=haiku | 0/2 [0%–66%] | 62% | 5.0s (4.5–5.4) | - | 10.5 | step budget exceeded×1, oracle check failed×1 |
| d4_estimate_intercept | drones=opus | 0/2 [0%–66%] | 75% | 5.4s (5.0–5.7) | - | 12.0 | oracle check failed×1, wall-clock deadline×1 |
| d4_estimate_intercept | drones=sonnet | 0/2 [0%–66%] | 62% | 4.0s (3.3–4.7) | - | 9.0 | oracle check failed×2 |
