# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c2_lowsurvey | drones=sonnet | 1 | - | 0 | 0.0s | 0 | $0.00 |
| p1_route2 | drones=sonnet | 1 | - | 0 | 0.0s | 0 | $0.00 |
| p4_revisit | drones=sonnet | 1 | goto×7, take_off×3, scan×3, ToolSearch×1, look×1 | 1 | 5.1s | 0 | $0.00 |
| s5_midpoint | drones=sonnet | 1 | goto×5, ToolSearch×1, take_off×1, scan×1 | 1 | 8.8s | 0 | $0.00 |
