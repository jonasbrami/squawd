# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| c2_lowsurvey | drones=sonnet | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
| p1_route2 | drones=sonnet | 0/0 [0%–100%] | 0% | 0.0s (0.0–0.0) | - | - | - |
| p4_revisit | drones=sonnet | 0/1 [0%–79%] | 17% | 3.4s (3.4–3.4) | - | 15.0 | wall-clock deadline×1 |
| s5_midpoint | drones=sonnet | 1/1 [21%–100%] | 100% | 2.9s (2.9–2.9) | 8.0 | - | - |
