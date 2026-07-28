# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| p1_identify | drones=pilot | 3 | hover×26, take_off×3, goto×3, track×2 | 0 | 3.0s | 0 | $0.00 |
| p1_identify | drones=pilot_null | 3 | take_off×3, goto×3, hover×3 | 0 | 9.8s | 0 | $0.00 |
| p2_crossing | drones=pilot | 4 | hover×28, take_off×4, goto×4, track×3 | 0 | 3.0s | 0 | $0.00 |
| p2_crossing | drones=pilot_null | 3 | take_off×3, goto×3, hover×3 | 0 | 12.3s | 0 | $0.00 |
