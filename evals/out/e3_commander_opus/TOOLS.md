# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w1_split_reach | _layer=commander,commander=opus,drones=opus | 2 | dispatch×4, ToolSearch×2, situation×2, done×1 | 0 | 3.3s | 247 | $0.27 |
| w2_allocation | _layer=commander,commander=opus,drones=opus | 2 | dispatch×4, ToolSearch×2, situation×2, done×2 | 0 | 6.4s | 410 | $0.35 |
| w3_crossing | _layer=commander,commander=opus,drones=opus | 2 | dispatch×8, ToolSearch×2, situation×2, done×1 | 0 | 5.2s | 671 | $0.41 |
| w5_sync_mark | _layer=commander,commander=opus,drones=opus | 2 | dispatch×6, ToolSearch×2, situation×2, done×1, ScheduleWakeup×1 | 0 | 3.5s | 1308 | $0.40 |
