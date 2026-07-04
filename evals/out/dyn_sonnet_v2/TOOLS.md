# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| d1_rendezvous | drones=sonnet | 2 | scan×8, goto×8, hover×4, ToolSearch×2, take_off×2, set_speed×2 | 2 | 6.9s | 0 | $0.00 |
| d2_shadow | drones=sonnet | 2 | scan×25, hover×15, goto×6, ToolSearch×4, take_off×3, set_speed×2, orbit×1 | 1 | 5.2s | 0 | $0.00 |
| d3_timing_gate | drones=sonnet | 2 | scan×17, hover×8, goto×4, ToolSearch×2, take_off×2, face×2, fly×2, set_speed×1 | 0 | 4.9s | 0 | $0.00 |
| d4_estimate_intercept | drones=sonnet | 2 | scan×15, hover×5, fly×5, ToolSearch×2, take_off×2, face×1 | 0 | 7.3s | 0 | $0.00 |
| d5_perimeter | drones=sonnet | 2 | scan×10, goto×9, fly×3, ToolSearch×2, take_off×2, set_speed×2, report×1 | 3 | 11.0s | 4801 | $0.42 |
