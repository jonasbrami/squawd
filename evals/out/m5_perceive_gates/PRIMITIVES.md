# Primitive statistics (observational only — §13 item 7)

Per-primitive call count, latency p50 (tool-call duration), and stable error-code counts (ICD §9), grouped by model / detector / difficulty. Numbers describe; they never rewrite prompts or parameters.

| primitive | model | detector | difficulty | calls | dur_p50 | errors |
|-----------|-------|----------|------------|-------|---------|--------|
| goto | drones=pilot | OnnxBackend | perceive=1 | 3 | 17.6s | - |
| goto | drones=pilot | OnnxBackend | perceive=2 | 4 | 17.6s | - |
| goto | drones=pilot_null | ColorBlobBackend | perceive=1 | 3 | 17.6s | - |
| goto | drones=pilot_null | ColorBlobBackend | perceive=2 | 3 | 17.6s | - |
| hover | drones=pilot | OnnxBackend | perceive=1 | 26 | 0.0s | - |
| hover | drones=pilot | OnnxBackend | perceive=2 | 28 | 0.0s | - |
| hover | drones=pilot_null | ColorBlobBackend | perceive=1 | 3 | 45.0s | - |
| hover | drones=pilot_null | ColorBlobBackend | perceive=2 | 3 | 60.0s | - |
| take_off | drones=pilot | OnnxBackend | perceive=1 | 3 | 2.1s | - |
| take_off | drones=pilot | OnnxBackend | perceive=2 | 4 | 2.1s | - |
| take_off | drones=pilot_null | ColorBlobBackend | perceive=1 | 3 | 2.1s | - |
| take_off | drones=pilot_null | ColorBlobBackend | perceive=2 | 3 | 2.1s | - |
| track | drones=pilot | OnnxBackend | perceive=1 | 2 | 15.7s | - |
| track | drones=pilot | OnnxBackend | perceive=2 | 3 | 75.4s | - |
