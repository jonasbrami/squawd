# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d4_estimate_intercept | drones=haiku | 2 | ToolSearch×5, scan×5, face×4, Bash×2, take_off×2, track×2, report×2, look×1 | 0 | 3.7s | 2542 | $0.09 |
| d4_estimate_intercept | drones=opus | 2 | scan×6, ToolSearch×4, take_off×2, track×2, report×2, set_speed×1 | 0 | 6.7s | 2391 | $0.46 |
| d4_estimate_intercept | drones=sonnet | 2 | ToolSearch×3, scan×3, take_off×2, track×2, report×2 | 0 | 9.5s | 1672 | $0.41 |
