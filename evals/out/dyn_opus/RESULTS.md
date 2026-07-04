# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d1_rendezvous | drones=opus | 1/2 [9%–91%] | 83% | 10.3s (8.5–12.1) | 6.0 | 9.0 | step budget exceeded×1 |
| d2_shadow | drones=opus | 0/2 [0%–66%] | 67% | 3.2s (3.2–3.2) | - | 21.0 | wall-clock deadline×2 |
| d3_timing_gate | drones=opus | 1/2 [9%–91%] | 83% | 4.4s (4.2–4.6) | 14.0 | 15.0 | step budget exceeded×1 |
| d4_estimate_intercept | drones=opus | 0/2 [0%–66%] | 50% | 4.5s (4.3–4.7) | - | 11.0 | step budget exceeded×2 |
| d5_perimeter | drones=opus | 0/2 [0%–66%] | 50% | 4.1s (3.9–4.2) | - | 11.0 | step budget exceeded×2 |
