# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| am4_briefing | drones=haiku | 0/1 [0%–79%] | 50% | 5.5s (5.5–5.5) | - | 4.0 | wall-clock deadline×1 |
| c1_recon_patrol | drones=haiku | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
| p1_route2 | drones=haiku | 0/1 [0%–79%] | 67% | 3.1s (3.1–3.1) | - | 5.0 | wall-clock deadline×1 |
| p3_route4 | drones=haiku | 1/1 [21%–100%] | 100% | 4.5s (4.5–4.5) | 10.0 | - | - |
| p4_revisit | drones=haiku | 0/1 [0%–79%] | 50% | 5.5s (5.5–5.5) | - | 9.0 | wall-clock deadline×1 |
| s5_midpoint | drones=haiku | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
| s6_bearing | drones=haiku | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
| s7_triangle | drones=haiku | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
