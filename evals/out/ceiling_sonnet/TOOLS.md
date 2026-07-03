# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c3_constrained_survey | drones=sonnet | 2 | goto×22, ToolSearch×2, take_off×2, scan×2, hover×2, land×2, set_speed×1, report×1 | 0 | 16.0s | 0 | $0.00 |
| p6_tight_route | drones=sonnet | 3 | goto×9, ToolSearch×2, take_off×2, report×2 | 1 | 15.8s | 1291 | $0.42 |
| p7_knapsack | drones=sonnet | 3 | goto×10, ToolSearch×2, take_off×2, land×2, report×2 | 0 | 19.4s | 11959 | $1.30 |
