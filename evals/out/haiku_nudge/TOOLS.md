# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| am5_noflyzone | drones=haiku | 8 | goto×35, ToolSearch×9, take_off×8, look×8, report×8, scan×4, Bash×1 | 2 | 16.6s | 5048 | $0.48 |
| p1_route2 | drones=haiku | 8 | goto×18, ToolSearch×11, report×9, take_off×8, hover×7 | 1 | 13.4s | 2424 | $0.30 |
| s6_bearing | drones=haiku | 8 | goto×17, ToolSearch×9, take_off×8, hover×8, report×7, fly×1 | 1 | 13.6s | 3283 | $0.27 |
