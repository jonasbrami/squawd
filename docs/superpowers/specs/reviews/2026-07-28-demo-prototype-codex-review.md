## 1. VERDICT

Verdict: challenge as written. The component boundaries are sound—UI commands stay deterministic, contacts remain authoritative, and moving-target orbit belongs in the offboard controller—but the design is not implementation-ready. Click identity, command ownership, and orbit/fusion are treated as existing “fall-through” behavior when the repo shows they require explicit redesign.

## 2. CLAIM CHECK

Verdict: five load-bearing claims are wrong or stale.

- [contacts.py:971](/home/quenouille/drone/agents/vision/contacts.py:971): `designate()` does call `request_lock()`, but ONNX defaults to tracker `"none"`; [detector.py:171](/home/quenouille/drone/agents/vision/detector.py:171) then calls `create_tracker("none")`, which raises at [trackers/__init__.py:47](/home/quenouille/drone/agents/vision/trackers/__init__.py:47). The doc’s “dormant, not on prototype path” bug is directly on the click path.
- [index.html:312](/home/quenouille/drone/agents/observatory/static/index.html:312): `matchContact` does not spatially associate; it chooses the same-class contact with nearest confidence and explicitly assumes one mover. It is unsafe with three cars.
- [estop.py:18](/home/quenouille/drone/agents/pilot/estop.py:18): `ActiveToolRegistry` is only one unowned task slot. Its generation is not checked by `FlightOps`; a resumed LLM tool can overwrite the operator task. Separate estop/cmd supervisors would also race, so “estop still wins unchanged” is false.
- [ops.py:715](/home/quenouille/drone/agents/flight/ops.py:715): the vision-tracking shaped servo discards `control_ref`’s returned feedforward and rebuilds velocity from target velocity plus error. Adding tangential feedforward only in [track.py:81](/home/quenouille/drone/agents/flight/track.py:81) will not deliver the proposed law.
- [beam.py:59](/home/quenouille/drone/agents/vision/beam.py:59): fusion accepts only `mode=="shadow"` and ≤3 m/s. Orbit does not inherit ToF fusion.
- W2’s cited `onnx_bench.py` does not exist; the available benchmark is `evals/mask_iou_eval.py`.

## 3. WEAKEST POINTS

Verdict: these are the three likely practical failures, ranked.

1. COCO-on-Fuel domain gap: attractive meshes are not evidence that small rendered cars/people will detect. Cheapest mitigation: gate two cars plus one person through the exported ONNX at expected ranges before building the full world.
2. Arbitration: cloned supervisors plus a single registry cannot guarantee estop > operator > LLM. Cheapest mitigation: one serialized command arbiter with an estop latch and explicit control-owner lease.
3. Orbit controller: phase discontinuity, discarded tangent feedforward, 7 m bubble interaction, and shadow-only fusion will produce lag or loss. Cheapest mitigation: initialize phase from current relative position and test the complete `FlightOps` path, not only `control_ref`.

## 4. MISSING

Verdict: two omissions matter most.

- Exact click timing: WebCodecs output is asynchronous, but [index.html:264](/home/quenouille/drone/agents/observatory/static/index.html:264) stores the newest received AU stamp, not the displayed decoded frame’s identity. Carry `frame_seq` through decode, retain snapshots by sequence, and reject stale clicks.
- Detection admission semantics: displaying 80 COCO classes is fine; creating followable contacts for chairs, trees, and static cars is not. Define a dynamic-class allowlist, per-class thresholds/size gates, and how the non-COCO delivery robot is classified.

## 5. MILESTONE ORDER

Verdict: keep the broad order, but pull risk forward.

- Add W0: detector-on-rendered-assets, exact det↔contact identity, tracker-`none`, and arbitration spike.
- Split W1: validate two movers first; scale to 6–10 only after W0 detection acceptance.
- Pull orbit math, phase initialization, feedforward, keep-out, and fusion tests into W3; leave W4 for live tuning and UI operations.

## 6. §6 DECISIONS

Verdict: three proposals stand, with tighter constraints.

- Person movers: `mover_system`; deterministic pose/contact observability outweighs walk animation for this demo.
- Stand-off: `range_m` on `track`; define radial initialization explicitly rather than calling it merely ω=0 orbit.
- COCO floor: 0.25 for overlay; start tracking at 0.30 vehicles/0.20 persons with two-hit confirmation.
- `/pilot/cmd`: always preempt, but acquire an operator lease until Stop/Resume/timeout; estop remains latched above both operator and LLM.