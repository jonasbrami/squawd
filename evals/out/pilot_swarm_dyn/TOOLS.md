# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w4_double_intercept | drones=pilot | 2 | goto×8, take_off×4, set_speed×4, goto_all×2, hover×2 | 0 | 9.0s | 0 | $0.00 |
| w4_double_intercept | drones=pilot_null | 2 | goto×16, take_off×2, set_speed×2 | 0 | 17.1s | 0 | $0.00 |
| w5_sync_mark | drones=pilot | 2 | take_off×4, goto_all×2, hover×2 | 0 | 13.6s | 0 | $0.00 |
| w5_sync_mark | drones=pilot_null | 2 | goto×4, take_off×2, set_speed×2, hover×2 | 0 | 9.3s | 0 | $0.00 |
