# Agent Task-Eval Results

| task | assignment | k | success_rate | lat_p50 | lat_p95 | mean_steps | failures |
|------|-----------|---|--------------|---------|---------|------------|----------|
| am1_explicit | drones=haiku | 3 | 100% | 3.6s | 3.7s | 5.0 | - |
| am1_explicit | drones=opus | 3 | 100% | 4.5s | 5.5s | 6.0 | - |
| am1_explicit | drones=sonnet | 3 | 100% | 3.1s | 8.3s | 6.3 | - |
| am2_relative | drones=haiku | 3 | 100% | 6.0s | 6.8s | 4.3 | - |
| am2_relative | drones=opus | 3 | 100% | 5.0s | 6.6s | 5.0 | - |
| am2_relative | drones=sonnet | 3 | 100% | 3.5s | 6.3s | 6.0 | - |
| am3_search | drones=haiku | 3 | 67% | 3.5s | 6.8s | 3.3 | wall-clock deadline×1 |
| am3_search | drones=opus | 3 | 100% | 5.3s | 6.1s | 4.7 | - |
| am3_search | drones=sonnet | 3 | 100% | 4.7s | 5.9s | 11.3 | - |
| c1_recon_patrol | drones=haiku | 3 | 100% | 20.0s | 20.0s | 4.0 | - |
| c1_recon_patrol | drones=opus | 3 | 0% | 4.4s | 5.6s | 9.3 | oracle check failed×3 |
| c1_recon_patrol | drones=sonnet | 3 | 0% | 3.2s | 3.5s | 9.3 | oracle check failed×3 |
| p1_route2 | drones=haiku | 3 | 0% | 3.3s | 3.7s | 9.0 | oracle check failed×2, step budget exceeded×1 |
| p1_route2 | drones=opus | 3 | 0% | 4.1s | 4.9s | 6.0 | oracle check failed×3 |
| p1_route2 | drones=sonnet | 3 | 0% | 3.2s | 3.2s | 7.0 | oracle check failed×3 |
| p2_route3 | drones=haiku | 3 | 0% | 4.5s | 5.4s | 9.0 | oracle check failed×3 |
| p2_route3 | drones=opus | 3 | 33% | 4.0s | 4.9s | 6.3 | oracle check failed×2 |
| p2_route3 | drones=sonnet | 3 | 0% | 3.2s | 5.3s | 8.7 | oracle check failed×3 |
| p3_route4 | drones=haiku | 3 | 0% | 4.8s | 49.7s | 4.0 | oracle check failed×3 |
| p3_route4 | drones=opus | 3 | 0% | 4.7s | 5.0s | 4.0 | oracle check failed×3 |
| p3_route4 | drones=sonnet | 3 | 0% | 2.5s | 3.4s | 8.0 | oracle check failed×3 |
| s1_dist60 | drones=haiku | 3 | 100% | 4.7s | 4.9s | 7.0 | - |
| s1_dist60 | drones=opus | 3 | 100% | 4.6s | 6.0s | 6.0 | - |
| s1_dist60 | drones=sonnet | 3 | 100% | 3.5s | 6.3s | 6.7 | - |
| s2_dist130 | drones=haiku | 3 | 100% | 4.2s | 5.1s | 7.3 | - |
| s2_dist130 | drones=opus | 3 | 100% | 4.5s | 4.5s | 7.0 | - |
| s2_dist130 | drones=sonnet | 3 | 100% | 3.2s | 4.1s | 6.0 | - |
| s3_dist250 | drones=haiku | 3 | 100% | 3.7s | 4.2s | 6.0 | - |
| s3_dist250 | drones=opus | 3 | 100% | 5.5s | 5.7s | 6.7 | - |
| s3_dist250 | drones=sonnet | 3 | 100% | 2.6s | 3.4s | 6.3 | - |
| s4_altband | drones=haiku | 3 | 100% | 3.8s | 6.0s | 5.7 | - |
| s4_altband | drones=opus | 3 | 100% | 5.9s | 6.0s | 6.0 | - |
| s4_altband | drones=sonnet | 3 | 100% | 3.8s | 4.8s | 6.7 | - |
