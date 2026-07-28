# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w7_survey_n2 | drones=haiku | 2 | run_mission×10, goto×8, fly×5, take_off×4, ToolSearch×2, land×2, goto_all×1 | 5 | 2.6s | 13674 | $0.13 |
