# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c4_obstacle_run | drones=haiku | 2 | goto×8, ToolSearch×2, scan×2, take_off×2, hover×2, report×2 | 2 | 14.5s | 5221 | $0.12 |
| o1_detour | drones=haiku | 2 | goto×11, ToolSearch×4, scan×2, take_off×2, report×2 | 1 | 7.9s | 6228 | $0.14 |
| o2_slalom | drones=haiku | 2 | goto×11, ToolSearch×1, take_off×1, scan×1, look×1 | 6 | 3.8s | 0 | $0.00 |
| o3_inspect | drones=haiku | 2 | goto×18, ToolSearch×4, scan×3, take_off×2, look×2, orbit×1 | 1 | 16.4s | 0 | $0.00 |
