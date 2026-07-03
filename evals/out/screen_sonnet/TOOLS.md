# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| am4_briefing | drones=sonnet | 3 | goto×4, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 13.6s | 1114 | $0.35 |
| am5_noflyzone | drones=sonnet | 2 | goto×8, look×5, scan×4, ToolSearch×2, take_off×2, report×2 | 1 | 3.4s | 2189 | $0.53 |
| c1_recon_patrol | drones=sonnet | 2 | goto×8, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 13.7s | 1080 | $0.39 |
| c2_lowsurvey | drones=sonnet | 2 | ToolSearch×2, run_mission×2, land×2, report×2, goto×1, hover×1 | 0 | 17.3s | 2878 | $0.46 |
| p1_route2 | drones=sonnet | 2 | goto×5, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 14.3s | 816 | $0.33 |
| p2_route3 | drones=sonnet | 7 | goto×9, ToolSearch×3, take_off×3, hover×2, report×2, scan×1 | 1 | 15.5s | 1328 | $0.47 |
| p3_route4 | drones=sonnet | 2 | goto×9, ToolSearch×2, take_off×2, report×2 | 1 | 15.5s | 1100 | $0.38 |
| p4_revisit | drones=sonnet | 2 | goto×11, take_off×4, ToolSearch×3, hover×2, report×2, scan×2, fly×1 | 0 | 19.8s | 1858 | $0.58 |
| p5_tsp5 | drones=sonnet | 2 | goto×13, ToolSearch×2, take_off×2, land×2, report×2 | 1 | 21.7s | 3482 | $0.65 |
| s5_midpoint | drones=sonnet | 7 | goto×3, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 7.1s | 731 | $0.50 |
| s6_bearing | drones=sonnet | 3 | goto×4, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 11.0s | 1100 | $0.34 |
| s7_triangle | drones=sonnet | 2 | goto×8, ToolSearch×2, take_off×2, hover×2, report×2 | 1 | 18.6s | 2653 | $0.51 |
