# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c3_constrained_survey | drones=opus | 2 | goto×20, ToolSearch×2, take_off×2, scan×2, set_speed×2, hover×2, report×2 | 0 | 14.1s | 4674 | $0.40 |
| p6_tight_route | drones=opus | 2 | goto×8, ToolSearch×2, take_off×2, report×2 | 0 | 16.2s | 1162 | $0.36 |
| p7_knapsack | drones=opus | 2 | goto×10, ToolSearch×2, take_off×2, report×2 | 0 | 19.5s | 4494 | $0.64 |
