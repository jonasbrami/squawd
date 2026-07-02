# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c1_recon_patrol | drones=haiku | 3/3 [44%–100%] | 100% | 6.5s (6.5–10.7) | 3.0 | - | - |
| c1_recon_patrol | drones=opus | 1/3 [6%–79%] | 87% | 5.0s (4.8–5.3) | 4.0 | 8.0 | oracle check failed×2 |
| c1_recon_patrol | drones=sonnet | 0/3 [0%–56%] | 60% | 2.8s (2.5–3.5) | - | 8.7 | oracle check failed×3 |
| p1_route2 | drones=haiku | 0/3 [0%–56%] | 67% | 3.9s (3.3–3.9) | - | 8.0 | oracle check failed×3 |
| p1_route2 | drones=opus | 0/3 [0%–56%] | 67% | 5.1s (3.8–5.5) | - | 6.0 | oracle check failed×3 |
| p1_route2 | drones=sonnet | 0/3 [0%–56%] | 67% | 2.7s (2.5–2.9) | - | 6.3 | oracle check failed×3 |
| p2_route3 | drones=haiku | 0/3 [0%–56%] | 56% | 3.8s (3.2–4.8) | - | 10.7 | step budget exceeded×1, oracle check failed×2 |
| p2_route3 | drones=opus | 0/3 [0%–56%] | 67% | 4.3s (4.2–4.4) | - | 7.0 | oracle check failed×3 |
| p2_route3 | drones=sonnet | 0/3 [0%–56%] | 67% | 3.0s (2.3–3.1) | - | 8.3 | oracle check failed×3 |
| p3_route4 | drones=haiku | 2/3 [21%–94%] | 89% | 10.7s (4.9–19.6) | 4.0 | 13.0 | oracle check failed×1 |
| p3_route4 | drones=opus | 2/3 [21%–94%] | 89% | 4.0s (3.5–4.0) | 4.0 | 7.0 | oracle check failed×1 |
| p3_route4 | drones=sonnet | 0/3 [0%–56%] | 67% | 2.5s (2.2–2.6) | - | 8.0 | oracle check failed×3 |
