# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| am4_briefing | drones=haiku | 2 | goto×5, ToolSearch×2, take_off×2, hover×2, report×2 | 2 | 12.4s | 1406 | $0.06 |
| am4_briefing | drones=opus | 2 | goto×3, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 14.9s | 844 | $0.27 |
| am4_briefing | drones=sonnet | 2 | goto×4, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 13.6s | 1114 | $0.35 |
| am5_noflyzone | drones=haiku | 2 | goto×9, Bash×5, look×3, ToolSearch×2, take_off×2, scan×1, report×1 | 1 | 8.1s | 4163 | $0.06 |
| am5_noflyzone | drones=opus | 2 | goto×6, look×6, scan×4, ToolSearch×2, take_off×2, report×2 | 0 | 5.6s | 1748 | $0.47 |
| am5_noflyzone | drones=sonnet | 2 | goto×8, look×5, scan×4, ToolSearch×2, take_off×2, report×2 | 1 | 3.4s | 2189 | $0.53 |
| c1_recon_patrol | drones=haiku | 2 | goto×6, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 14.3s | 1942 | $0.07 |
| c1_recon_patrol | drones=opus | 2 | goto×6, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 14.7s | 926 | $0.31 |
| c1_recon_patrol | drones=sonnet | 2 | goto×8, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 13.7s | 1080 | $0.39 |
| c2_lowsurvey | drones=haiku | 2 | run_mission×4, ToolSearch×2, report×2, scan×2, hover×1, land×1, take_off×1 | 0 | 19.6s | 3728 | $0.09 |
| c2_lowsurvey | drones=opus | 2 | ToolSearch×2, scan×2, take_off×2, run_mission×2, hover×2, report×2 | 0 | 15.5s | 1667 | $0.35 |
| c2_lowsurvey | drones=sonnet | 2 | ToolSearch×2, run_mission×2, land×2, report×2, goto×1, hover×1 | 0 | 17.3s | 2878 | $0.46 |
| p1_route2 | drones=haiku | 2 | goto×8, ToolSearch×2, take_off×2, hover×1, report×1 | 1 | 16.5s | 3029 | $0.04 |
| p1_route2 | drones=opus | 2 | ToolSearch×2, take_off×2, goto×2, hover×2, report×2, fly×2 | 0 | 16.2s | 740 | $0.28 |
| p1_route2 | drones=sonnet | 2 | goto×5, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 14.3s | 816 | $0.33 |
| p2_route3 | drones=haiku | 2 | goto×8, ToolSearch×3, take_off×2, hover×2, report×2 | 1 | 15.1s | 2230 | $0.08 |
| p2_route3 | drones=opus | 2 | goto×9, ToolSearch×3, take_off×3, set_speed×1, scan×1, hover×1, report×1 | 0 | 16.6s | 877 | $0.15 |
| p2_route3 | drones=sonnet | 2 | goto×9, ToolSearch×3, take_off×3, hover×2, report×2, scan×1 | 1 | 15.5s | 1328 | $0.47 |
| p3_route4 | drones=haiku | 2 | goto×9, ToolSearch×2, take_off×2, report×2, Bash×2 | 0 | 16.0s | 2336 | $0.08 |
| p3_route4 | drones=opus | 2 | goto×8, ToolSearch×2, take_off×2, report×2 | 0 | 16.7s | 1004 | $0.31 |
| p3_route4 | drones=sonnet | 2 | goto×9, ToolSearch×2, take_off×2, report×2 | 1 | 15.5s | 1100 | $0.38 |
| p4_revisit | drones=haiku | 2 | goto×12, ToolSearch×2, take_off×2, hover×2, report×2 | 2 | 18.9s | 2380 | $0.09 |
| p4_revisit | drones=opus | 2 | goto×8, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 20.7s | 1114 | $0.35 |
| p4_revisit | drones=sonnet | 2 | goto×11, take_off×4, ToolSearch×3, hover×2, report×2, scan×2, fly×1 | 0 | 19.8s | 1858 | $0.58 |
| p5_tsp5 | drones=haiku | 2 | goto×14, ToolSearch×2, take_off×2, land×2, report×2 | 4 | 19.0s | 4664 | $0.11 |
| p5_tsp5 | drones=opus | 2 | goto×12, ToolSearch×2, take_off×2, report×2 | 0 | 23.1s | 2168 | $0.44 |
| p5_tsp5 | drones=sonnet | 2 | goto×13, ToolSearch×2, take_off×2, land×2, report×2 | 1 | 21.7s | 3482 | $0.65 |
| s5_midpoint | drones=haiku | 2 | goto×3, ToolSearch×2, take_off×2, hover×2 | 0 | 18.1s | 2672 | $0.06 |
| s5_midpoint | drones=opus | 2 | ToolSearch×2, take_off×2, goto×2, hover×2, report×2 | 0 | 12.2s | 611 | $0.25 |
| s5_midpoint | drones=sonnet | 2 | goto×3, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 7.1s | 731 | $0.50 |
| s6_bearing | drones=haiku | 2 | ToolSearch×10, goto×3, take_off×1, hover×1, report×1 | 0 | 4.3s | 2606 | $0.04 |
| s6_bearing | drones=opus | 2 | goto×4, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 19.2s | 1162 | $0.31 |
| s6_bearing | drones=sonnet | 2 | goto×4, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 11.0s | 1100 | $0.34 |
| s7_triangle | drones=haiku | 2 | goto×7, ToolSearch×3, take_off×2, hover×2, report×2 | 0 | 23.8s | 4282 | $0.11 |
| s7_triangle | drones=opus | 2 | goto×6, ToolSearch×2, take_off×2, hover×2, report×2 | 0 | 24.0s | 2210 | $0.41 |
| s7_triangle | drones=sonnet | 2 | goto×8, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 18.6s | 2653 | $0.51 |
