# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| am4_briefing | drones=opus | 0/2 [0%–66%] | 75% | 3.4s (3.3–3.6) | - | 7.5 | oracle check failed×2 |
| am5_noflyzone | drones=opus | 0/1 [0%–79%] | 25% | 7.0s (7.0–7.0) | - | 8.0 | wall-clock deadline×1 |
| c1_recon_patrol | drones=opus | 0/1 [0%–79%] | 20% | 4.0s (4.0–4.0) | - | 10.0 | wall-clock deadline×1 |
| c2_lowsurvey | drones=opus | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
| p3_route4 | drones=opus | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
| s6_bearing | drones=opus | 0/1 [0%–79%] | 67% | 6.7s (6.7–6.7) | - | 8.0 | wall-clock deadline×1 |
