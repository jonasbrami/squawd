# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w7_survey_n8 | drones=pilot | 1/2 [9%–91%] | 96% | 0.0s (0.0–0.0) | 24.0 | 24.0 | oracle check failed×1 |
| w7_survey_n8 | drones=pilot_null | 0/2 [0%–66%] | 83% | 0.0s (0.0–0.0) | - | 20.0 | oracle check failed×2 |
