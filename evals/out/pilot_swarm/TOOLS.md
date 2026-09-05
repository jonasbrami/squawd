# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w1_split_reach | drones=pilot | 2 | take_off×4, goto_all×2 | 0 | 9.1s | 0 | $0.00 |
| w2_allocation | drones=pilot | 2 | take_off×4, goto_all×4 | 0 | 12.6s | 0 | $0.00 |
| w2_allocation | drones=pilot_null | 2 | take_off×4, goto_all×4 | 0 | 12.1s | 0 | $0.00 |
| w3_crossing | drones=pilot | 2 | take_off×4, goto_all×4 | 0 | 11.6s | 0 | $0.00 |
| w3_crossing | drones=pilot_null | 2 | take_off×4, goto_all×4 | 0 | 12.6s | 0 | $0.00 |
