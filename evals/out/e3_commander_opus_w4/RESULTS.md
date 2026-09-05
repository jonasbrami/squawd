# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w4_double_intercept | _layer=commander,commander=opus,drones=opus | 0/2 [0%–66%] | 67% | 7.3s (7.0–7.6) | - | 5.0 | wall-clock deadline×2 |
