# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c4_obstacle_run | drones=sonnet | 3 | scan×13, goto×10, take_off×5, fly×3, ToolSearch×2, look×1 | 1 | 7.5s | 0 | $0.00 |
| o1_detour | drones=sonnet | 5 | scan×10, goto×8, take_off×5, ToolSearch×2, look×1 | 1 | 4.2s | 0 | $0.00 |
| o2_slalom | drones=sonnet | 6 | scan×11, goto×9, look×3, ToolSearch×2, take_off×2, report×2, land×1 | 0 | 4.8s | 0 | $0.00 |
| o3_inspect | drones=sonnet | 4 | goto×12, scan×6, ToolSearch×3, take_off×2, orbit×1, land×1, report×1, set_speed×1 | 1 | 12.3s | 5713 | $0.44 |
