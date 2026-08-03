# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| d1_rendezvous | drones=pilot | 1/1 [21%–100%] | 100% | 0.0s (0.0–0.0) | 4.0 | - | - |
| d2_shadow | drones=pilot | 0/1 [0%–79%] | 67% | 0.0s (0.0–0.0) | - | 2.0 | oracle check failed×1 |
| d2_shadow | drones=pilot_null | 0/1 [0%–79%] | 67% | 0.0s (0.0–0.0) | - | 27.0 | oracle check failed×1 |
| d3_timing_gate | drones=pilot | 1/1 [21%–100%] | 100% | 0.0s (0.0–0.0) | 7.0 | - | - |
| d3_timing_gate | drones=pilot_null | 0/1 [0%–79%] | 83% | 0.0s (0.0–0.0) | - | 3.0 | oracle check failed×1 |
| d4_estimate_intercept | drones=pilot | 1/1 [21%–100%] | 100% | 0.0s (0.0–0.0) | 2.0 | - | - |
| d4_estimate_intercept | drones=pilot_null | 0/1 [0%–79%] | 50% | 0.0s (0.0–0.0) | - | 10.0 | oracle check failed×1 |
| d5_perimeter | drones=pilot | 1/1 [21%–100%] | 100% | 0.0s (0.0–0.0) | 5.0 | - | - |
| d5_perimeter | drones=pilot_null | 0/1 [0%–79%] | 50% | 0.0s (0.0–0.0) | - | 10.0 | oracle check failed×1 |
