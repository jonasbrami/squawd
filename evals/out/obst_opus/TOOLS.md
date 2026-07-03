# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c4_obstacle_run | drones=opus | 2 | goto×7, ToolSearch×2, scan×2, take_off×2, hover×2, report×2 | 1 | 13.2s | 4982 | $0.67 |
| o1_detour | drones=opus | 2 | goto×9, scan×4, ToolSearch×3, take_off×3, report×1, set_speed×1 | 0 | 11.7s | 4063 | $0.29 |
| o2_slalom | drones=opus | 2 | goto×7, scan×6, look×6, ToolSearch×3, take_off×2, hover×2, report×2 | 0 | 5.1s | 8372 | $0.51 |
| o3_inspect | drones=opus | 2 | ToolSearch×4, goto×4, look×4, scan×3, take_off×2, orbit×2, report×2, set_speed×1, hover×1 | 0 | 5.6s | 4562 | $0.70 |
