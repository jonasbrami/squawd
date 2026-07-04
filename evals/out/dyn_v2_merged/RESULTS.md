# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d1_rendezvous | drones=haiku | 1/2 [9%–91%] | 67% | 6.3s (5.2–7.5) | 9.0 | 13.0 | step budget exceeded×1 |
| d1_rendezvous | drones=opus | 2/2 [34%–100%] | 100% | 9.0s (8.4–9.5) | 10.5 | - | - |
| d1_rendezvous | drones=sonnet | 0/2 [0%–66%] | 67% | 5.1s (4.6–5.5) | - | 13.0 | step budget exceeded×2 |
| d2_shadow | drones=haiku | 0/2 [0%–66%] | 67% | 4.0s (3.5–4.5) | - | 25.5 | wall-clock deadline×2 |
| d2_shadow | drones=opus | 0/2 [0%–66%] | 67% | 3.4s (3.4–3.5) | - | 21.0 | wall-clock deadline×2 |
| d2_shadow | drones=sonnet | 0/2 [0%–66%] | 50% | 2.4s (2.0–2.9) | - | 28.0 | step budget exceeded×1, wall-clock deadline×1 |
| d3_timing_gate | drones=haiku | 0/2 [0%–66%] | 58% | 5.2s (4.7–5.8) | - | 19.0 | step budget exceeded×2 |
| d3_timing_gate | drones=opus | 1/2 [9%–91%] | 92% | 4.2s (4.1–4.2) | 18.0 | 19.0 | step budget exceeded×1 |
| d3_timing_gate | drones=sonnet | 0/2 [0%–66%] | 50% | 3.9s (3.3–4.4) | - | 19.0 | step budget exceeded×2 |
| d4_estimate_intercept | drones=haiku | 0/2 [0%–66%] | 62% | 5.4s (5.1–5.8) | - | 14.5 | step budget exceeded×1, oracle check failed×1 |
| d4_estimate_intercept | drones=opus | 0/2 [0%–66%] | 50% | 6.3s (6.2–6.4) | - | 15.0 | step budget exceeded×2 |
| d4_estimate_intercept | drones=sonnet | 0/2 [0%–66%] | 50% | 3.8s (3.0–4.5) | - | 15.0 | step budget exceeded×2 |
| d5_perimeter | drones=haiku | 0/2 [0%–66%] | 62% | 5.8s (3.4–8.2) | - | 14.0 | step budget exceeded×1, oracle check failed×1 |
| d5_perimeter | drones=opus | 0/2 [0%–66%] | 75% | 5.4s (4.2–6.5) | - | 12.5 | oracle check failed×2 |
| d5_perimeter | drones=sonnet | 0/2 [0%–66%] | 62% | 4.0s (3.7–4.2) | - | 14.5 | step budget exceeded×1, oracle check failed×1 |
