# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w1_split_reach | _layer=commander,commander=haiku,drones=opus | 2 | dispatch×5, ToolSearch×2, situation×2, done×1, Bash×1 | 0 | 2.4s | 293 | $0.05 |
| w2_allocation | _layer=commander,commander=haiku,drones=opus | 2 | dispatch×4, situation×2, ToolSearch×1 | 0 | 1.6s | 162 | $0.05 |
| w3_crossing | _layer=commander,commander=haiku,drones=opus | 2 | dispatch×8, ToolSearch×2, situation×2, done×1 | 0 | 2.2s | 790 | $0.09 |
| w5_sync_mark | _layer=commander,commander=haiku,drones=opus | 2 | dispatch×4, ToolSearch×2, situation×2, done×2 | 0 | 3.8s | 558 | $0.07 |
