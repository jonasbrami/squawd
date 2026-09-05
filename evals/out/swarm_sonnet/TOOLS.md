# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w1_split_reach | drones=sonnet | 2 | goto_all×12, take_off×4, goto×4, ToolSearch×2 | 1 | 4.9s | 0 | $0.00 |
| w2_allocation | drones=sonnet | 2 | goto_all×9, goto×8, take_off×4, ToolSearch×2 | 2 | 6.4s | 4280 | $0.66 |
| w3_crossing | drones=sonnet | 4 | goto×26, goto_all×17, take_off×8, ToolSearch×4, scan×2, report×2 | 4 | 4.9s | 0 | $0.00 |
| w4_double_intercept | drones=sonnet | 4 | goto×27, scan×24, ToolSearch×11, take_off×8, face×2, goto_all×2, hover×2 | 5 | 3.5s | 0 | $0.00 |
| w5_sync_mark | drones=sonnet | 2 | goto_all×11, goto×10, take_off×4, hover×3, ToolSearch×2 | 4 | 5.0s | 4725 | $0.36 |
