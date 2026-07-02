# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| am4_briefing | drones=haiku | 1 | goto×2, ToolSearch×1, take_off×1 | 1 | 3.7s | 0 | $0.00 |
| c1_recon_patrol | drones=haiku | 1 | - | 0 | 0.0s | 0 | $0.00 |
| p1_route2 | drones=haiku | 1 | goto×3, ToolSearch×1, take_off×1 | 0 | 36.6s | 0 | $0.00 |
| p3_route4 | drones=haiku | 1 | goto×7, ToolSearch×1, take_off×1, report×1 | 1 | 16.3s | 2237 | $0.04 |
| p4_revisit | drones=haiku | 1 | goto×7, ToolSearch×1, take_off×1 | 0 | 26.4s | 0 | $0.00 |
| s5_midpoint | drones=haiku | 1 | - | 0 | 0.0s | 0 | $0.00 |
| s6_bearing | drones=haiku | 1 | - | 0 | 0.0s | 0 | $0.00 |
| s7_triangle | drones=haiku | 1 | - | 0 | 0.0s | 0 | $0.00 |
