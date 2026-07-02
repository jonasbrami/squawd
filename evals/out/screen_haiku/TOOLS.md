# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| am4_briefing | drones=haiku | 2 | goto×5, ToolSearch×2, take_off×2, hover×2, report×2 | 2 | 12.4s | 1406 | $0.06 |
| am5_noflyzone | drones=haiku | 11 | goto×28, ToolSearch×19, look×13, take_off×8, report×6, Bash×6, scan×3, hover×2 | 1 | 5.6s | 4672 | $0.29 |
| c1_recon_patrol | drones=haiku | 2 | goto×6, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 14.3s | 1942 | $0.07 |
| c2_lowsurvey | drones=haiku | 5 | run_mission×4, ToolSearch×2, report×2, scan×2, hover×1, land×1, take_off×1 | 0 | 19.6s | 3728 | $0.09 |
| p1_route2 | drones=haiku | 8 | goto×26, ToolSearch×9, take_off×8, report×7, hover×6 | 2 | 15.5s | 2276 | $0.21 |
| p2_route3 | drones=haiku | 2 | goto×8, ToolSearch×3, take_off×2, hover×2, report×2 | 1 | 15.1s | 2230 | $0.08 |
| p3_route4 | drones=haiku | 2 | goto×9, ToolSearch×2, take_off×2, report×2, Bash×2 | 0 | 16.0s | 2336 | $0.08 |
| p4_revisit | drones=haiku | 2 | goto×12, ToolSearch×2, take_off×2, hover×2, report×2 | 2 | 18.9s | 2380 | $0.09 |
| p5_tsp5 | drones=haiku | 2 | goto×14, ToolSearch×2, take_off×2, land×2, report×2 | 4 | 19.0s | 4664 | $0.11 |
| s5_midpoint | drones=haiku | 2 | goto×3, ToolSearch×2, take_off×2, hover×2 | 0 | 18.1s | 2672 | $0.06 |
| s6_bearing | drones=haiku | 8 | ToolSearch×16, goto×14, take_off×7, hover×7, report×7, fly×5 | 1 | 5.2s | 2820 | $0.26 |
| s7_triangle | drones=haiku | 2 | goto×7, ToolSearch×3, take_off×2, hover×2, report×2 | 0 | 23.8s | 4282 | $0.11 |
