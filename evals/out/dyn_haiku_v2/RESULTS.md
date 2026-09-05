# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d1_rendezvous | drones=haiku | 1/2 [9%–91%] | 67% | 6.3s (5.2–7.5) | 9.0 | 13.0 | step budget exceeded×1 |
| d2_shadow | drones=haiku | 0/2 [0%–66%] | 67% | 4.0s (3.5–4.5) | - | 25.5 | wall-clock deadline×2 |
| d3_timing_gate | drones=haiku | 0/2 [0%–66%] | 58% | 5.2s (4.7–5.8) | - | 19.0 | step budget exceeded×2 |
| d4_estimate_intercept | drones=haiku | 0/2 [0%–66%] | 62% | 5.4s (5.1–5.8) | - | 14.5 | step budget exceeded×1, oracle check failed×1 |
| d5_perimeter | drones=haiku | 0/2 [0%–66%] | 62% | 5.8s (3.4–8.2) | - | 14.0 | step budget exceeded×1, oracle check failed×1 |
