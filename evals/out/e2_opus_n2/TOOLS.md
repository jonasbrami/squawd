# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w7_survey_n2 | drones=opus | 2 | take_off×4, run_mission×4, scan×4, land×4, ToolSearch×2, report×2 | 0 | 2.7s | 3016 | $0.51 |
