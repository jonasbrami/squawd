# Pursuit-altitude experiment review — 2026-08-03

**VERDICT: DO NOT SHIP AS-IS.** The altitude diagnosis is directionally
correct, but the patch fixes only the nominal geometry and the repo’s actual
telemetry already demonstrates failure after reaching 5.5 m.

**Disposition — 2026-08-09:** the rejected 5.5 m altitude and range-table
changes were discarded. Finding 4's independent coast-latch defect was fixed
by holding the current vehicle position in both COASTING branches and pinned
with a regression test. The other recommendations remain follow-up design
work, not accepted implementation.

1. Critical — the supplied evidence does not match the claim. The checked-in timeline contains neither `vis_car_243` nor `vis_car_245`. It records `vis_car_3`, beginning pursuit at 7.1 m/23.4 m, reaching 5.5 m, then bottom-clipping at 18.2 m and becoming LOST ([timeline:26](../../evals/out/show_2026-08-03/repro_timeline.jsonl:26), [timeline:38](../../evals/out/show_2026-08-03/repro_timeline.jsonl:38), [timeline:79](../../evals/out/show_2026-08-03/repro_timeline.jsonl:79), [timeline:111](../../evals/out/show_2026-08-03/repro_timeline.jsonl:111)). Resolve provenance and rerun before claiming this fix closes the complaint.

2. High — use 6.0 m, not 5.5 m. The geometry function gives:

- 5.5 m: `R_min=17`, implicit lock ring 19 m, ≈19.7 m aim-point slant.
- 6.0 m: `R_min=18`, ring 20 m, ≈20.8 m slant.

Both fit the qualified 10–22 m band, but documented SITL altitude noise is ±1–2 m ([PROJECT-STATE.md:63](../../docs/PROJECT-STATE.md:63)); 5.5 m can physically fall below 4 m. Six metres has live precedent ([demo-scenarios:92](../../docs/benchmarks/demo-scenarios-2026-08-02.md:92)). Also, the claimed “3°” margin is optimistic: the law aims at target z=0.5 m ([projection.py:22](../../agents/perception/projection.py:22)); the bbox bottom is nearer ground level and has less pitch margin.

3. High — add a pre-pursuit descent phase. `track()` primes the final-altitude setpoint for only 0.25 s ([ops.py:668](../../agents/flight/ops.py:668)) and then immediately enables the direct pursuit lane ([ops.py:704](../../agents/flight/ops.py:704), [ops.py:828](../../agents/flight/ops.py:828)). Meanwhile `R_min` was calculated from the final altitude, not live altitude ([ops.py:638](../../agents/flight/ops.py:638)).

Inside `track()`, without changing its public contract, stage `hold_altitude` shadow/orbit by holding initial x/y, zero horizontal FF, streaming the target yaw and commanded altitude until `|H−Hcmd|≤0.5 m` for 0.5 s. Then enable pursuit. Above roughly 8 m, reject or explicitly stage before accepting a lock; a 23→6 m descent cannot reliably preserve a detector-qualified ground target.

4. High — the direct-lane coast hold is wrong. `_shp` is initialized at engagement start ([ops.py:700](../../agents/flight/ops.py:700)) but never updated by the direct lane; COASTING commands that stale point ([ops.py:797](../../agents/flight/ops.py:797)). That can produce the operator’s “immediately deviates” symptom. Latch current aircraft x/y on the first direct-lane coast tick instead. The R8 barrier also disappears once the bbox is stale ([ops.py:880](../../agents/flight/ops.py:880)), so it cannot recover an established clip.

5. Medium — standoff semantics are silently inconsistent. A requested 12 m becomes 17 m at 5.5 m ([ops.py:650](../../agents/flight/ops.py:650)); that safety clamp is correct, but the API still advertises 8–40 m ([cmd.py:38](../../agents/pilot/cmd.py:38), [server.py:72](../../agents/observatory/server.py:72)). Values above about 21 m horizontal exceed the v2 slant band. Expose the effective value and constrain the demo profile to approximately 18–21 m at 6 m.

6. Medium — orbit should not retain arbitrary cruise altitude. At 8 m, `alt=None` captures 8 m ([ops.py:565](../../agents/flight/ops.py:565)) and a requested 15 m orbit floors to 24 m, outside the detector band. Apply the same 6 m altitude/staging to operator orbit, or reject orbit until the shadow has settled.

7. Configuration/tests: the launcher does not forward `SQUAWD_PURSUIT_ALT_M` ([run_single_demo.sh:54](../../scripts/run_single_demo.sh:54)); `_pursuit_alt()` accepts `inf`, and the test explicitly blesses 7 m—already outside the intended band ([cmd.py:53](../../agents/pilot/cmd.py:53), [test_cmd_supervisor.py:155](../../tests/test_cmd_supervisor.py:155)). Validate finite, geometry-compatible values and add transient, coast-latch, standoff, and orbit tests. I independently ran the targeted file: 14 tests passed; they currently test dispatch, not flight behavior.
