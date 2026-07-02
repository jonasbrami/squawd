# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| am4_briefing | drones=haiku | 2/2 [34%–100%] | 100% | 5.0s (4.5–5.5) | 6.5 | - | - |
| am5_noflyzone | drones=haiku | 5/8 [31%–86%] | 84% | 24.2s (14.0–40.3) | 9.2 | 13.0 | step budget exceeded×3 |
| c1_recon_patrol | drones=haiku | 2/2 [34%–100%] | 100% | 4.0s (3.9–4.0) | 7.0 | - | - |
| c2_lowsurvey | drones=haiku | 2/2 [34%–100%] | 100% | 14.8s (10.0–19.7) | 6.5 | - | - |
| p1_route2 | drones=haiku | 6/8 [41%–93%] | 92% | 3.7s (2.7–8.7) | 6.8 | 7.5 | wall-clock deadline×2 |
| p2_route3 | drones=haiku | 2/2 [34%–100%] | 100% | 4.3s (4.0–4.5) | 8.5 | - | - |
| p3_route4 | drones=haiku | 2/2 [34%–100%] | 100% | 6.6s (4.5–8.7) | 8.5 | - | - |
| p4_revisit | drones=haiku | 2/2 [34%–100%] | 100% | 5.5s (4.1–7.0) | 10.0 | - | - |
| p5_tsp5 | drones=haiku | 2/2 [34%–100%] | 100% | 14.6s (11.5–17.7) | 11.0 | - | - |
| s5_midpoint | drones=haiku | 2/2 [34%–100%] | 100% | 5.0s (4.7–5.2) | 4.5 | - | - |
| s6_bearing | drones=haiku | 7/8 [53%–98%] | 92% | 9.0s (4.0–12.0) | 6.7 | 9.0 | step budget exceeded×1 |
| s7_triangle | drones=haiku | 2/2 [34%–100%] | 100% | 16.1s (11.0–21.2) | 8.0 | - | - |
