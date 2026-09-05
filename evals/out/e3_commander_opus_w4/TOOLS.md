# Tool usage by cell (from transcripts)

burst = max per-cell count of gotos issued <5s after the previous goto (each one overrode an in-flight move on pre-blocking-goto data); gap_p50 = median seconds between tool calls (patience).

| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |
|------|-----------|-------|----------|-------|---------|-------------|------|
| w4_double_intercept | _layer=commander,commander=opus,drones=opus | 2 | dispatch×6, ToolSearch×2, situation×2 | 0 | 4.3s | 2782 | $0.49 |
