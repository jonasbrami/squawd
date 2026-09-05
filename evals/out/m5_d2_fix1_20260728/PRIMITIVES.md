# Primitive statistics (observational only — §13 item 7)

Per-primitive call count, latency p50 (tool-call duration), and stable error-code counts (ICD §9), grouped by model / detector / difficulty. Numbers describe; they never rewrite prompts or parameters.

| primitive | model | detector | difficulty | calls | dur_p50 | errors |
|-----------|-------|----------|------------|-------|---------|--------|
| goto | drones=pilot_null | - | dynamic=2 | 25 | 0.0s | - |
| set_speed | drones=pilot_null | - | dynamic=2 | 1 | 0.0s | - |
| take_off | drones=pilot | - | dynamic=2 | 1 | 12.1s | - |
| take_off | drones=pilot_null | - | dynamic=2 | 1 | 5.1s | - |
| track | drones=pilot | - | dynamic=2 | 1 | 75.3s | - |
