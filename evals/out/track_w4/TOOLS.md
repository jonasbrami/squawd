# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w4_double_intercept | drones=haiku | 2 | scan×6, goto×6, goto_all×3, ToolSearch×2, take_off×2, fly×2, track_all×2, track×2, Bash×1 | 4 | 3.0s | 3631 | $0.06 |
| w4_double_intercept | drones=opus | 2 | scan×8, take_off×4, goto_all×4, goto×4, track×4, ToolSearch×3, report×2 | 1 | 4.4s | 4622 | $0.96 |
| w4_double_intercept | drones=sonnet | 2 | scan×10, goto_all×10, goto×9, ToolSearch×5, take_off×4, track_all×2, track×2 | 4 | 5.2s | 0 | $0.00 |
