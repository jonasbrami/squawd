# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d1_rendezvous | drones=sonnet | 2 | scan×6, ToolSearch×3, goto×3, take_off×2, hover×2, set_speed×2 | 1 | 5.9s | 0 | $0.00 |
| d2_shadow | drones=sonnet | 2 | scan×18, goto×9, hover×8, take_off×3, ToolSearch×2 | 4 | 5.1s | 0 | $0.00 |
| d3_timing_gate | drones=sonnet | 2 | scan×13, hover×7, goto×5, ToolSearch×2, take_off×2, set_speed×1 | 0 | 4.4s | 0 | $0.00 |
| d4_estimate_intercept | drones=sonnet | 2 | scan×6, hover×3, ToolSearch×2, take_off×2, set_speed×2, fly×2, goto×1 | 0 | 7.9s | 0 | $0.00 |
| d5_perimeter | drones=sonnet | 2 | scan×6, goto×4, take_off×3, ToolSearch×2, set_speed×1, face×1, look×1 | 0 | 11.6s | 0 | $0.00 |
