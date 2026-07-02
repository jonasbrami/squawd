# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| am4_briefing | drones=haiku | 2/2 [34%–100%] | 100% | 5.0s (4.5–5.5) | 6.5 | - | - |
| am4_briefing | drones=opus | 2/2 [34%–100%] | 100% | 5.5s (5.0–6.0) | 5.5 | - | - |
| am4_briefing | drones=sonnet | 2/2 [34%–100%] | 100% | 6.0s (5.2–6.9) | 6.0 | - | - |
| am5_noflyzone | drones=haiku | 1/2 [9%–91%] | 75% | 23.5s (22.8–24.2) | 10.0 | 13.0 | step budget exceeded×1 |
| am5_noflyzone | drones=opus | 2/2 [34%–100%] | 100% | 7.8s (7.6–8.0) | 11.0 | - | - |
| am5_noflyzone | drones=sonnet | 2/2 [34%–100%] | 100% | 9.0s (7.8–10.2) | 11.5 | - | - |
| c1_recon_patrol | drones=haiku | 2/2 [34%–100%] | 100% | 4.0s (3.9–4.0) | 7.0 | - | - |
| c1_recon_patrol | drones=opus | 2/2 [34%–100%] | 100% | 4.3s (3.8–4.9) | 7.0 | - | - |
| c1_recon_patrol | drones=sonnet | 2/2 [34%–100%] | 100% | 3.4s (3.1–3.8) | 8.0 | - | - |
| c2_lowsurvey | drones=haiku | 2/2 [34%–100%] | 100% | 14.8s (10.0–19.7) | 6.5 | - | - |
| c2_lowsurvey | drones=opus | 2/2 [34%–100%] | 100% | 11.1s (10.6–11.5) | 6.0 | - | - |
| c2_lowsurvey | drones=sonnet | 2/2 [34%–100%] | 100% | 18.6s (11.5–25.8) | 5.0 | - | - |
| p1_route2 | drones=haiku | 1/2 [9%–91%] | 83% | 3.3s (2.7–3.8) | 7.0 | 7.0 | wall-clock deadline×1 |
| p1_route2 | drones=opus | 2/2 [34%–100%] | 100% | 4.5s (4.3–4.6) | 6.0 | - | - |
| p1_route2 | drones=sonnet | 2/2 [34%–100%] | 100% | 2.8s (2.5–3.1) | 6.5 | - | - |
| p2_route3 | drones=haiku | 2/2 [34%–100%] | 100% | 4.3s (4.0–4.5) | 8.5 | - | - |
| p2_route3 | drones=opus | 2/2 [34%–100%] | 100% | 5.3s (3.2–7.4) | 9.5 | - | - |
| p2_route3 | drones=sonnet | 2/2 [34%–100%] | 100% | 3.9s (2.4–5.5) | 10.0 | - | - |
| p3_route4 | drones=haiku | 2/2 [34%–100%] | 100% | 6.6s (4.5–8.7) | 8.5 | - | - |
| p3_route4 | drones=opus | 2/2 [34%–100%] | 100% | 4.2s (3.8–4.5) | 7.0 | - | - |
| p3_route4 | drones=sonnet | 2/2 [34%–100%] | 100% | 4.4s (2.7–6.1) | 7.5 | - | - |
| p4_revisit | drones=haiku | 2/2 [34%–100%] | 100% | 5.5s (4.1–7.0) | 10.0 | - | - |
| p4_revisit | drones=opus | 2/2 [34%–100%] | 100% | 3.7s (3.4–3.9) | 8.0 | - | - |
| p4_revisit | drones=sonnet | 2/2 [34%–100%] | 100% | 2.3s (2.2–2.4) | 12.5 | - | - |
| p5_tsp5 | drones=haiku | 2/2 [34%–100%] | 100% | 14.6s (11.5–17.7) | 11.0 | - | - |
| p5_tsp5 | drones=opus | 2/2 [34%–100%] | 100% | 12.5s (12.4–12.5) | 9.0 | - | - |
| p5_tsp5 | drones=sonnet | 2/2 [34%–100%] | 100% | 17.8s (17.7–17.9) | 10.5 | - | - |
| s5_midpoint | drones=haiku | 2/2 [34%–100%] | 100% | 5.0s (4.7–5.2) | 4.5 | - | - |
| s5_midpoint | drones=opus | 2/2 [34%–100%] | 100% | 3.9s (3.8–4.0) | 5.0 | - | - |
| s5_midpoint | drones=sonnet | 2/2 [34%–100%] | 100% | 3.7s (2.8–4.5) | 5.5 | - | - |
| s6_bearing | drones=haiku | 1/2 [9%–91%] | 67% | 6.6s (4.0–9.2) | 7.0 | 9.0 | step budget exceeded×1 |
| s6_bearing | drones=opus | 2/2 [34%–100%] | 100% | 7.5s (6.5–8.5) | 6.0 | - | - |
| s6_bearing | drones=sonnet | 2/2 [34%–100%] | 100% | 4.4s (4.3–4.6) | 6.0 | - | - |
| s7_triangle | drones=haiku | 2/2 [34%–100%] | 100% | 16.1s (11.0–21.2) | 8.0 | - | - |
| s7_triangle | drones=opus | 2/2 [34%–100%] | 100% | 14.8s (14.2–15.4) | 7.0 | - | - |
| s7_triangle | drones=sonnet | 2/2 [34%–100%] | 100% | 12.0s (11.1–12.9) | 8.0 | - | - |
