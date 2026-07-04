# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w1_split_reach | drones=opus | 4 | goto×8, take_off×4, ToolSearch×2 | 2 | 3.5s | 0 | $0.00 |
| w2_allocation | drones=opus | 3 | goto×10, report×6, take_off×4, ToolSearch×2, land×2 | 4 | 1.8s | 2828 | $0.46 |
| w3_crossing | drones=opus | 5 | goto×18, report×10, goto_all×9, take_off×8, ToolSearch×5 | 3 | 7.2s | 4505 | $1.20 |
| w4_double_intercept | drones=opus | 4 | scan×32, goto×17, goto_all×9, ToolSearch×8, take_off×8, set_speed×4, report×4, hover×2 | 3 | 3.5s | 0 | $0.00 |
| w5_sync_mark | drones=opus | 2 | goto×6, take_off×4, goto_all×4, hover×4, report×4, ToolSearch×3 | 2 | 4.7s | 3470 | $0.54 |
