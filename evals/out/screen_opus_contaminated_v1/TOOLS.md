# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| am4_briefing | drones=opus | 2 | take_off×4, ToolSearch×3, goto×3, report×2, hover×1, scan×1, run_mission×1 | 1 | 6.7s | 1754 | $0.41 |
| am5_noflyzone | drones=opus | 1 | goto×4, ToolSearch×2, take_off×1, set_speed×1 | 0 | 11.0s | 0 | $0.00 |
| c1_recon_patrol | drones=opus | 1 | goto×5, ToolSearch×2, take_off×1, scan×1, set_speed×1 | 0 | 4.7s | 0 | $0.00 |
| c2_lowsurvey | drones=opus | 1 | - | 0 | 0.0s | 0 | $0.00 |
| p3_route4 | drones=opus | 1 | - | 0 | 0.0s | 0 | $0.00 |
| s6_bearing | drones=opus | 1 | take_off×3, goto×3, ToolSearch×1, scan×1 | 0 | 4.2s | 0 | $0.00 |
