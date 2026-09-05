# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d2_shadow | drones=pilot | 1 | take_off×1, track×1 | 0 | 20.1s | 0 | $0.00 |
| d2_shadow | drones=pilot_null | 1 | goto×25, take_off×1, set_speed×1 | 0 | 9.3s | 0 | $0.00 |
