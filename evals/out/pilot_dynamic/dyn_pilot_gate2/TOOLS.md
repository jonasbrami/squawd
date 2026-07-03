# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d1_rendezvous | drones=pilot | 1 | take_off×1, set_speed×1, goto×1, hover×1 | 0 | 5.1s | 0 | $0.00 |
| d2_shadow | drones=pilot | 1 | goto×25, take_off×1, set_speed×1 | 22 | 2.5s | 0 | $0.00 |
| d2_shadow | drones=pilot_null | 1 | goto×25, take_off×1, set_speed×1 | 23 | 2.3s | 0 | $0.00 |
| d3_timing_gate | drones=pilot | 1 | hover×3, goto×2, take_off×1, set_speed×1 | 0 | 1.0s | 0 | $0.00 |
| d3_timing_gate | drones=pilot_null | 1 | take_off×1, set_speed×1, goto×1 | 0 | 2.6s | 0 | $0.00 |
| d4_estimate_intercept | drones=pilot | 1 | goto×3, take_off×1, set_speed×1 | 0 | 6.8s | 0 | $0.00 |
| d4_estimate_intercept | drones=pilot_null | 1 | goto×8, take_off×1, set_speed×1 | 4 | 5.1s | 0 | $0.00 |
| d5_perimeter | drones=pilot | 1 | goto×3, take_off×1, set_speed×1 | 2 | 4.6s | 0 | $0.00 |
| d5_perimeter | drones=pilot_null | 1 | goto×8, take_off×1, set_speed×1 | 4 | 5.1s | 0 | $0.00 |
