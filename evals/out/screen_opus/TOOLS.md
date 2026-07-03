# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| am4_briefing | drones=opus | 2 | goto×3, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 14.9s | 844 | $0.27 |
| am5_noflyzone | drones=opus | 2 | goto×6, look×6, scan×4, ToolSearch×2, take_off×2, report×2 | 0 | 5.6s | 1748 | $0.47 |
| c1_recon_patrol | drones=opus | 2 | goto×6, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 14.7s | 926 | $0.31 |
| c2_lowsurvey | drones=opus | 2 | ToolSearch×2, scan×2, take_off×2, run_mission×2, hover×2, report×2 | 0 | 15.5s | 1667 | $0.35 |
| p1_route2 | drones=opus | 2 | ToolSearch×2, take_off×2, goto×2, hover×2, report×2, fly×2 | 0 | 16.2s | 740 | $0.28 |
| p2_route3 | drones=opus | 2 | goto×9, ToolSearch×3, take_off×3, set_speed×1, scan×1, hover×1, report×1 | 0 | 16.6s | 877 | $0.15 |
| p3_route4 | drones=opus | 2 | goto×8, ToolSearch×2, take_off×2, report×2 | 0 | 16.7s | 1004 | $0.31 |
| p4_revisit | drones=opus | 2 | goto×8, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 20.7s | 1114 | $0.35 |
| p5_tsp5 | drones=opus | 2 | goto×12, ToolSearch×2, take_off×2, report×2 | 0 | 23.1s | 2168 | $0.44 |
| s5_midpoint | drones=opus | 2 | ToolSearch×2, take_off×2, goto×2, hover×2, report×2 | 0 | 12.2s | 611 | $0.25 |
| s6_bearing | drones=opus | 2 | goto×4, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 19.2s | 1162 | $0.31 |
| s7_triangle | drones=opus | 2 | goto×6, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 24.0s | 2210 | $0.41 |
