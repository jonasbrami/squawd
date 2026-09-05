# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w1_split_reach | drones=haiku | 2 | goto×10, Bash×8, ToolSearch×3, goto_all×3, take_off×2 | 3 | 3.6s | 0 | $0.00 |
| w2_allocation | drones=haiku | 3 | goto×9, take_off×4, land×4, Bash×3, ToolSearch×2, report×2, goto_all×2 | 2 | 3.2s | 4293 | $0.06 |
| w3_crossing | drones=haiku | 5 | Bash×17, goto×10, take_off×9, ToolSearch×8, goto_all×5, scan×4 | 7 | 2.6s | 4923 | $0.06 |
| w4_double_intercept | drones=haiku | 4 | scan×22, Bash×19, goto×15, ToolSearch×12, take_off×6, goto_all×6, face×2, Agent×1, TaskCreate×1 | 4 | 2.9s | 0 | $0.00 |
| w5_sync_mark | drones=haiku | 2 | goto×10, goto_all×7, take_off×4, hover×4, ToolSearch×2, look×2, land×2, Bash×1 | 4 | 3.1s | 3906 | $0.11 |
