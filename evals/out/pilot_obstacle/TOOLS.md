# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| c4_obstacle_run | drones=pilot | 1 | goto×4, take_off×1, hover×1 | 0 | 6.6s | 0 | $0.00 |
| o1_detour | drones=pilot | 1 | goto×3, take_off×1, hover×1 | 0 | 5.8s | 0 | $0.00 |
| o2_slalom | drones=pilot | 1 | goto×4, take_off×1, hover×1 | 1 | 6.6s | 0 | $0.00 |
| o3_inspect | drones=pilot | 1 | goto×3, take_off×1, hover×1 | 0 | 5.6s | 0 | $0.00 |
