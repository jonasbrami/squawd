# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d1_rendezvous | drones=opus | 2 | scan×7, goto×4, hover×4, ToolSearch×3, take_off×2, set_speed×1 | 0 | 13.6s | 0 | $0.00 |
| d2_shadow | drones=opus | 2 | scan×20, hover×14, ToolSearch×3, goto×3, take_off×2 | 1 | 6.1s | 0 | $0.00 |
| d3_timing_gate | drones=opus | 2 | scan×20, goto×7, ToolSearch×3, take_off×2, set_speed×2, hover×2, report×1 | 0 | 6.7s | 4545 | $0.43 |
| d4_estimate_intercept | drones=opus | 2 | scan×16, ToolSearch×4, hover×4, take_off×2, set_speed×2, fly×2 | 0 | 8.6s | 0 | $0.00 |
| d5_perimeter | drones=opus | 2 | scan×7, goto×7, ToolSearch×4, set_speed×2, take_off×2, report×2, face×1 | 0 | 9.1s | 5209 | $0.77 |
