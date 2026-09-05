# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w1_split_reach | _layer=commander,commander=haiku,drones=haiku | 2 | dispatch×4, ToolSearch×2, situation×2 | 0 | 1.7s | 549 | $0.03 |
| w2_allocation | _layer=commander,commander=haiku,drones=haiku | 2 | situation×6, dispatch×4, ToolSearch×2, done×1 | 0 | 4.3s | 4818 | $0.10 |
| w3_crossing | _layer=commander,commander=haiku,drones=haiku | 2 | dispatch×4, ToolSearch×2, situation×2 | 0 | 3.5s | 626 | $0.04 |
| w5_sync_mark | _layer=commander,commander=haiku,drones=haiku | 2 | dispatch×4, Bash×2, ToolSearch×2, situation×2 | 0 | 2.1s | 238 | $0.05 |
