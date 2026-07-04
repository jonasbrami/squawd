# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d1_rendezvous | drones=haiku | 1/2 [9%–91%] | 67% | 5.4s (4.3–6.5) | 7.0 | 9.0 | step budget exceeded×1 |
| d2_shadow | drones=haiku | 0/2 [0%–66%] | 50% | 6.7s (4.2–9.2) | - | 18.5 | wall-clock deadline×2 |
| d3_timing_gate | drones=haiku | 0/2 [0%–66%] | 67% | 5.7s (4.5–6.9) | - | 12.0 | wall-clock deadline×1, step budget exceeded×1 |
| d4_estimate_intercept | drones=haiku | 0/2 [0%–66%] | 50% | 4.2s (4.1–4.2) | - | 11.0 | step budget exceeded×2 |
| d5_perimeter | drones=haiku | 0/2 [0%–66%] | 50% | 5.0s (4.4–5.5) | - | 11.0 | step budget exceeded×2 |
