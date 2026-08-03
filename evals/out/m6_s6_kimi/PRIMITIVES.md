# Primitive statistics (observational only — §13 item 7)

Per-primitive call count, latency p50 (tool-call duration), and stable error-code counts (ICD §9), grouped by model / detector / difficulty. Numbers describe; they never rewrite prompts or parameters.

| primitive | model | detector | difficulty | calls | dur_p50 | errors |
|-----------|-------|----------|------------|-------|---------|--------|
| detect | drones=kimi | OnnxBackend | perceive=1 | 1 | 0.1s | - |
| report | drones=kimi | OnnxBackend | perceive=1 | 1 | 0.0s | - |
| scan | drones=kimi | OnnxBackend | perceive=1 | 1 | 0.0s | - |
| take_off | drones=kimi | OnnxBackend | perceive=1 | 1 | 7.1s | - |
