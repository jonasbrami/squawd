# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| am5_noflyzone | drones=haiku | 8/8 [68%–100%] | 100% | 25.9s (20.3–37.9) | 9.1 | - | - |
| p1_route2 | drones=haiku | 7/8 [53%–98%] | 96% | 3.6s (3.1–4.7) | 7.1 | 3.0 | oracle check failed×1 |
| s6_bearing | drones=haiku | 8/8 [68%–100%] | 100% | 7.8s (3.9–12.8) | 6.2 | - | - |
