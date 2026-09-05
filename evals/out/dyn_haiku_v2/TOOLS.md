# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d1_rendezvous | drones=haiku | 2 | goto×7, scan×5, ToolSearch×4, fly×2, take_off×2, hover×2 | 3 | 6.9s | 0 | $0.00 |
| d2_shadow | drones=haiku | 2 | scan×27, fly×10, goto×5, hover×3, ToolSearch×2, look×2, face×2 | 2 | 3.2s | 0 | $0.00 |
| d3_timing_gate | drones=haiku | 2 | goto×20, scan×9, ToolSearch×4, hover×3, take_off×1, fly×1 | 4 | 6.7s | 0 | $0.00 |
| d4_estimate_intercept | drones=haiku | 2 | scan×12, fly×5, face×3, goto×3, ToolSearch×2, take_off×1, hover×1, set_speed×1, report×1 | 1 | 3.7s | 9148 | $0.10 |
| d5_perimeter | drones=haiku | 2 | scan×9, goto×8, face×4, ToolSearch×3, look×2, take_off×1, report×1 | 3 | 4.3s | 4213 | $0.06 |
