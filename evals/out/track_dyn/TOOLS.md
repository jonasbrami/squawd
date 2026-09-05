# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d2_shadow | drones=haiku | 6 | ToolSearch×48, track×4, take_off×1, scan×1 | 0 | 2.8s | 0 | $0.00 |
| d2_shadow | drones=opus | 11 | ToolSearch×4, track×4, scan×2, take_off×2, report×2 | 0 | 11.2s | 0 | $0.64 |
| d2_shadow | drones=sonnet | 6 | goto×6, scan×5, take_off×3, track×3, report×3, ToolSearch×2 | 1 | 7.4s | 0 | $1.39 |
| d4_estimate_intercept | drones=haiku | 2 | ToolSearch×17, take_off×1, scan×1, track×1, report×1 | 0 | 2.9s | 1523 | $0.03 |
| d4_estimate_intercept | drones=opus | 11 | scan×11, ToolSearch×4, hover×3, take_off×2, fly×2, track×1, report×1 | 0 | 8.7s | 0 | $0.29 |
| d4_estimate_intercept | drones=sonnet | 2 | scan×6, ToolSearch×2, take_off×2, hover×2, track×2, report×2, fly×2 | 0 | 17.6s | 4510 | $0.71 |
