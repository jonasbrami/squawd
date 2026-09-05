# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w7_survey_n4 | drones=pilot | 2/2 [34%–100%] | 100% | 0.0s (0.0–0.0) | 14.0 | - | - |
| w7_survey_n4 | drones=pilot_null | 0/2 [0%–66%] | 88% | 0.0s (0.0–0.0) | - | 16.0 | oracle check failed×2 |
