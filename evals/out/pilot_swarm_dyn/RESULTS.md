# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w4_double_intercept | drones=pilot | 2/2 [34%–100%] | 100% | 0.0s (0.0–0.0) | 10.0 | - | - |
| w4_double_intercept | drones=pilot_null | 0/2 [0%–66%] | 67% | 0.0s (0.0–0.0) | - | 10.0 | oracle check failed×2 |
| w5_sync_mark | drones=pilot | 2/2 [34%–100%] | 100% | 0.0s (0.0–0.0) | 4.0 | - | - |
| w5_sync_mark | drones=pilot_null | 0/2 [0%–66%] | 67% | 0.0s (0.0–0.0) | - | 5.0 | oracle check failed×2 |
