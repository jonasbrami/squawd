# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d2_shadow | drones=pilot | 1/1 [21%–100%] | 100% | 0.0s (0.0–0.0) | 2.0 | - | - |
| d2_shadow | drones=pilot_null | 0/1 [0%–79%] | 67% | 0.0s (0.0–0.0) | - | 27.0 | oracle check failed×1 |
