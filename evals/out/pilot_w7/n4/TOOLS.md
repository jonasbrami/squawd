# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w7_survey_n4 | drones=pilot | 2 | goto_all×12, take_off×8, set_speed×8 | 0 | 7.1s | 0 | $0.00 |
| w7_survey_n4 | drones=pilot_null | 2 | goto_all×24, take_off×4, set_speed×4 | 0 | 9.5s | 0 | $0.00 |
