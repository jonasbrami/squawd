# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c3_constrained_survey | drones=haiku | 2 | goto×25, ToolSearch×2, take_off×2, scan×1, hover×1, land×1, report×1 | 0 | 18.2s | 6474 | $0.09 |
| p6_tight_route | drones=haiku | 2 | ToolSearch×2, run_mission×2, report×2, take_off×1 | 0 | 4.4s | 6343 | $0.12 |
| p7_knapsack | drones=haiku | 3 | goto×12, ToolSearch×2, take_off×2, report×2, land×1 | 1 | 18.8s | 13742 | $0.27 |
