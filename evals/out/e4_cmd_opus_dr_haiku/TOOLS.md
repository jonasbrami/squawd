# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w1_split_reach | _layer=commander,commander=opus,drones=haiku | 2 | dispatch×7, situation×3, ToolSearch×2 | 0 | 3.1s | 786 | $0.26 |
| w2_allocation | _layer=commander,commander=opus,drones=haiku | 2 | dispatch×4, ToolSearch×2, situation×2 | 0 | 3.7s | 614 | $0.25 |
| w3_crossing | _layer=commander,commander=opus,drones=haiku | 2 | dispatch×6, ToolSearch×2, situation×2, done×1 | 0 | 5.0s | 308 | $0.35 |
| w5_sync_mark | _layer=commander,commander=opus,drones=haiku | 2 | dispatch×6, ToolSearch×2, situation×2, done×1 | 0 | 6.8s | 936 | $0.36 |
