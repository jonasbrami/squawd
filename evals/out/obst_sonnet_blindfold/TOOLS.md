# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c4_obstacle_run | drones=sonnet | 1 | - | 0 | 0.0s | 0 | $0.00 |
| o1_detour | drones=sonnet | 2 | scan×9, goto×6, ToolSearch×2, take_off×2, face×1, look×1, fly×1 | 0 | 13.7s | 0 | $0.00 |
| o2_slalom | drones=sonnet | 1 | scan×5, fly×4, look×2, ToolSearch×1, take_off×1 | 0 | 8.8s | 0 | $0.00 |
| o3_inspect | drones=sonnet | 1 | - | 0 | 0.0s | 0 | $0.00 |
