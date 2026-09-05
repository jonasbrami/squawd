# Primitive statistics (observational only — §13 item 7)

Per-primitive call count, latency p50 (tool-call duration), and stable error-code counts (ICD §9), grouped by model / detector / difficulty. Numbers describe; they never rewrite prompts or parameters.

| primitive | model | detector | difficulty | calls | dur_p50 | errors |
|-----------|-------|----------|------------|-------|---------|--------|
| goto | drones=pilot | - | dynamic=1 | 1 | 20.6s | - |
| goto | drones=pilot | - | dynamic=3 | 2 | 14.8s | - |
| goto | drones=pilot | - | dynamic=5 | 3 | 0.0s | - |
| goto | drones=pilot_null | - | dynamic=2 | 25 | 0.0s | - |
| goto | drones=pilot_null | - | dynamic=3 | 1 | 19.1s | - |
| goto | drones=pilot_null | - | dynamic=4 | 8 | 0.0s | - |
| goto | drones=pilot_null | - | dynamic=5 | 8 | 0.0s | - |
| hover | drones=pilot | - | dynamic=1 | 1 | 40.0s | - |
| hover | drones=pilot | - | dynamic=3 | 3 | 0.0s | - |
| set_speed | drones=pilot | - | dynamic=1 | 1 | 0.0s | - |
| set_speed | drones=pilot | - | dynamic=3 | 1 | 0.0s | - |
| set_speed | drones=pilot | - | dynamic=5 | 1 | 0.0s | - |
| set_speed | drones=pilot_null | - | dynamic=2 | 1 | 0.0s | - |
| set_speed | drones=pilot_null | - | dynamic=3 | 1 | 0.0s | - |
| set_speed | drones=pilot_null | - | dynamic=4 | 1 | 0.0s | - |
| set_speed | drones=pilot_null | - | dynamic=5 | 1 | 0.0s | - |
| take_off | drones=pilot | - | dynamic=1 | 1 | 5.1s | - |
| take_off | drones=pilot | - | dynamic=2 | 1 | 5.1s | - |
| take_off | drones=pilot | - | dynamic=3 | 1 | 5.1s | - |
| take_off | drones=pilot | - | dynamic=4 | 1 | 5.1s | - |
| take_off | drones=pilot | - | dynamic=5 | 1 | 5.1s | - |
| take_off | drones=pilot_null | - | dynamic=2 | 1 | 5.1s | - |
| take_off | drones=pilot_null | - | dynamic=3 | 1 | 5.1s | - |
| take_off | drones=pilot_null | - | dynamic=4 | 1 | 20.1s | - |
| take_off | drones=pilot_null | - | dynamic=5 | 1 | 5.1s | - |
| track | drones=pilot | - | dynamic=2 | 1 | 75.3s | - |
| track | drones=pilot | - | dynamic=4 | 1 | 19.4s | - |
