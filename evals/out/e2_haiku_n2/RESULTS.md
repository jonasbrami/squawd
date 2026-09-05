# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w7_survey_n2 | drones=haiku | 0/2 [0%–66%] | 75% | 5.9s (5.5–6.2) | - | 16.0 | oracle check failed×1, step budget exceeded×1 |
