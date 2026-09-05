# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w4_double_intercept | drones=pilot | 2 | take_off×4, track_all×2 | 0 | 12.1s | 0 | $0.00 |
| w4_double_intercept | drones=pilot_null | 2 | goto×16, take_off×2, set_speed×2 | 0 | 17.1s | 0 | $0.00 |
