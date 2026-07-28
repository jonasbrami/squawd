# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| p1_identify | drones=pilot | 0/3 [0%–56%] | 67% | 0.0s (0.0–0.0) | - | 11.3 | oracle check failed×3 |
| p1_identify | drones=pilot_null | 0/3 [0%–56%] | 50% | 0.0s (0.0–0.0) | - | 3.0 | oracle check failed×3 |
| p2_crossing | drones=pilot | 0/4 [0%–49%] | 69% | 0.0s (0.0–0.0) | - | 9.8 | oracle check failed×4 |
| p2_crossing | drones=pilot_null | 0/3 [0%–56%] | 50% | 0.0s (0.0–0.0) | - | 3.0 | oracle check failed×3 |
