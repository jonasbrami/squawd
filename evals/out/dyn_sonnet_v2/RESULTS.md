# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d1_rendezvous | drones=sonnet | 0/2 [0%–66%] | 67% | 5.1s (4.6–5.5) | - | 13.0 | step budget exceeded×2 |
| d2_shadow | drones=sonnet | 0/2 [0%–66%] | 50% | 2.4s (2.0–2.9) | - | 28.0 | step budget exceeded×1, wall-clock deadline×1 |
| d3_timing_gate | drones=sonnet | 0/2 [0%–66%] | 50% | 3.9s (3.3–4.4) | - | 19.0 | step budget exceeded×2 |
| d4_estimate_intercept | drones=sonnet | 0/2 [0%–66%] | 50% | 3.8s (3.0–4.5) | - | 15.0 | step budget exceeded×2 |
| d5_perimeter | drones=sonnet | 0/2 [0%–66%] | 62% | 4.0s (3.7–4.2) | - | 14.5 | step budget exceeded×1, oracle check failed×1 |
