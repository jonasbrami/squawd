# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c3_constrained_survey | drones=pilot | 1 | goto×11, take_off×1, hover×1 | 0 | 17.8s | 0 | $0.00 |
| p6_tight_route | drones=pilot | 1 | goto×4, take_off×1, hover×1 | 0 | 13.6s | 0 | $0.00 |
| p7_knapsack | drones=pilot | 1 | goto×5, take_off×1, hover×1 | 0 | 19.6s | 0 | $0.00 |
