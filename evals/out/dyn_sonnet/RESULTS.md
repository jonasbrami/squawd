# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d1_rendezvous | drones=sonnet | 0/2 [0%–66%] | 33% | 3.9s (3.6–4.2) | - | 9.0 | step budget exceeded×2 |
| d2_shadow | drones=sonnet | 0/2 [0%–66%] | 67% | 2.4s (1.8–3.1) | - | 20.0 | wall-clock deadline×2 |
| d3_timing_gate | drones=sonnet | 0/2 [0%–66%] | 58% | 3.2s (2.4–4.1) | - | 15.0 | step budget exceeded×2 |
| d4_estimate_intercept | drones=sonnet | 0/2 [0%–66%] | 75% | 4.7s (4.5–4.8) | - | 9.0 | wall-clock deadline×2 |
| d5_perimeter | drones=sonnet | 0/2 [0%–66%] | 62% | 3.4s (2.4–4.3) | - | 9.0 | step budget exceeded×1, wall-clock deadline×1 |
