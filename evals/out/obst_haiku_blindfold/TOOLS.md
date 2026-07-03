# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c4_obstacle_run | drones=haiku | 2 | goto×8, ToolSearch×1, scan×1, take_off×1 | 0 | 28.2s | 0 | $0.00 |
| o1_detour | drones=haiku | 2 | fly×3, ToolSearch×1, take_off×1, scan×1, goto×1, report×1 | 0 | 7.2s | 2956 | $0.05 |
| o2_slalom | drones=haiku | 1 | goto×7, ToolSearch×1, scan×1, take_off×1, report×1 | 1 | 9.5s | 4332 | $0.09 |
