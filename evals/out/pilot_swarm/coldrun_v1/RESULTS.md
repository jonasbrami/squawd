# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w1_split_reach | drones=pilot | 2/2 [34%–100%] | 100% | 0.0s (0.0–0.0) | 3.0 | - | - |
| w2_allocation | drones=pilot | 1/2 [9%–91%] | 88% | 0.0s (0.0–0.0) | 4.0 | 4.0 | oracle check failed×1 |
| w2_allocation | drones=pilot_null | 0/1 [0%–79%] | 75% | 0.0s (0.0–0.0) | - | 4.0 | oracle check failed×1 |
| w3_crossing | drones=pilot | 2/2 [34%–100%] | 100% | 0.0s (0.0–0.0) | 4.0 | - | - |
| w3_crossing | drones=pilot_null | 0/2 [0%–66%] | 80% | 0.0s (0.0–0.0) | - | 4.0 | oracle check failed×2 |
