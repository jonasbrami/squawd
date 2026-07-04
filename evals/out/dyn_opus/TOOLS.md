# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d1_rendezvous | drones=opus | 2 | scan×5, goto×3, ToolSearch×2, take_off×2, hover×2, set_speed×1 | 0 | 6.9s | 0 | $0.00 |
| d2_shadow | drones=opus | 2 | scan×22, hover×10, goto×4, ToolSearch×2, take_off×2, set_speed×1, fly×1 | 1 | 5.0s | 0 | $0.00 |
| d3_timing_gate | drones=opus | 2 | scan×11, goto×7, ToolSearch×3, set_speed×3, take_off×2, face×1, hover×1, report×1 | 1 | 7.3s | 5216 | $0.40 |
| d4_estimate_intercept | drones=opus | 2 | scan×9, ToolSearch×4, take_off×2, hover×2, goto×2, set_speed×1, fly×1, face×1 | 0 | 8.1s | 0 | $0.00 |
| d5_perimeter | drones=opus | 2 | goto×7, scan×6, ToolSearch×3, take_off×3, set_speed×2, face×1 | 1 | 4.1s | 0 | $0.00 |
