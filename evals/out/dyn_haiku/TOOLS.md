# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d1_rendezvous | drones=haiku | 2 | goto×8, ToolSearch×3, scan×3, take_off×1, hover×1 | 1 | 7.8s | 0 | $0.00 |
| d2_shadow | drones=haiku | 2 | scan×10, goto×8, fly×5, ToolSearch×3, take_off×3, run_mission×3, look×2, face×1, hover×1, set_speed×1 | 3 | 3.4s | 0 | $0.00 |
| d3_timing_gate | drones=haiku | 2 | scan×8, goto×8, ToolSearch×3, hover×3, fly×1, take_off×1 | 1 | 5.3s | 0 | $0.00 |
| d4_estimate_intercept | drones=haiku | 2 | scan×9, face×5, ToolSearch×2, look×1, take_off×1, hover×1, set_speed×1, goto×1, fly×1 | 0 | 2.4s | 0 | $0.00 |
| d5_perimeter | drones=haiku | 2 | scan×8, goto×8, ToolSearch×2, face×2, take_off×1, set_speed×1 | 4 | 3.1s | 0 | $0.00 |
