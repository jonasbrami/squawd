# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d2_shadow | drones=pilot | 2 | hover×4, take_off×2, set_speed×2, goto×2, run_mission×2 | 0 | 5.1s | 0 | $0.00 |
| d2_shadow | drones=pilot_null | 2 | goto×38, take_off×2, set_speed×2 | 0 | 12.3s | 0 | $0.00 |
