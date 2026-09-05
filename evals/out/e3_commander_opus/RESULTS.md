# Agent Task-Eval Results

success_rate carries its Wilson 95% interval; gcs = mean fraction of oracle checks passed; ttfa = time to first tool call (model-side); steps✓ = mean steps over passing repeats only.

| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |
|------|-----------|--------------|-----|------------------|--------|--------|----------|
| w1_split_reach | _layer=commander,commander=opus,drones=opus | 2/2 [34%–100%] | 100% | 4.0s (3.8–4.2) | 4.5 | - | - |
| w2_allocation | _layer=commander,commander=opus,drones=opus | 2/2 [34%–100%] | 100% | 7.1s (5.1–9.0) | 5.0 | - | - |
| w3_crossing | _layer=commander,commander=opus,drones=opus | 2/2 [34%–100%] | 100% | 7.3s (6.1–8.5) | 6.5 | - | - |
| w5_sync_mark | _layer=commander,commander=opus,drones=opus | 2/2 [34%–100%] | 100% | 3.6s (3.5–3.7) | 6.0 | - | - |
