SHIP-WITH-CHANGES — the host-sidecar direction is sound, but the plan should not be implemented as written. The raw-frame seam, asynchronous execution, frame freshness, GPU scheduling, and network security need correction first.

## Findings

1. **BLOCKER — the proposed frame source does not exist.** The plan says `VisionPipeline.latest()` supplies frames ([plan:126](/home/quenouille/.kimi-code/sessions/wd_drone_579721d6036f/session_b05e3ce4-f723-4164-8975-d72f674361a7/agents/main/plans/black-hawk-taskmaster-groot.md:126)), but `PerceptionSnapshot` contains dimensions and detections—not RGB bytes ([pipeline.py:18](/home/quenouille/drone/agents/vision/pipeline.py:18)). The raw frame lives in `InferenceResult.frame` ([types.py:38](/home/quenouille/drone/agents/vision/types.py:38)) or `GzCameras.snapshot()` ([camera.py:71](/home/quenouille/drone/agents/core/camera.py:71)).

   Do not add 691 KB of RGB to `PerceptionSnapshot` or `/pilot/detections`. Inject `frame_source=lambda: cameras.snapshot(0)` into the deep tools/slowlane during assembly at [run.py:102](/home/quenouille/drone/agents/pilot/run.py:102).

2. **BLOCKER — synchronous `urllib` would stall the pilot event loop.** MCP handlers are async, but the existing lightweight detector is called synchronously at [tools.py:263](/home/quenouille/drone/agents/flight/tools.py:263). An 8–10 second host HTTP call there would block `estop_supervisor` and `cmd_supervisor`, which share the same loop ([agent.py:66](/home/quenouille/drone/agents/pilot/agent.py:66)).

   Every deep call must use `await asyncio.to_thread(client.detect/segment, …)` or a real async client. Preserve cancellation semantics and test estop responsiveness against a deliberately hung sidecar. Eight- and ten-second operational timeouts are excessive after warm-up; target ≤1 second failure and reject work immediately when busy.

3. **HIGH — slow-lane boxes become invalid as the camera moves.** At 0.3 Hz, a box may be over three seconds old. The current overlay deliberately joins by camera sequence/stamp ([server.py:137](/home/quenouille/drone/agents/observatory/server.py:137)); the plan omits frame identity from deep responses ([plan:84](/home/quenouille/.kimi-code/sessions/wd_drone_579721d6036f/session_b05e3ce4-f723-4164-8975-d72f674361a7/agents/main/plans/black-hawk-taskmaster-groot.md:84)). Likewise, comparing a slow result to the *current* fast box would make `fp_suspect` geometrically meaningless.

   Responses and publications need `frame_seq`, `sim_stamp`, dimensions, and completion time. Compute overlap against fast detections from the exact submitted `InferenceResult`, define it as `intersection / fast_box_area`, and expire display/advisory state after ≤0.5 seconds. Never let stale advisories modify contacts.

4. **HIGH — GPU coexistence is not controlled.** The service needs one process, one model instance each, and one inference lock. `YOLOWorld.set_classes()` mutates shared embeddings/names, so concurrent vocabularies can race. Slow-lane requests must be skip-if-busy with no queue; on-demand requests get priority.

   The demo defaults to Intel rendering ([run_single_demo.sh:10](/home/quenouille/drone/scripts/run_single_demo.sh:10)), but NVIDIA rendering explicitly shares the GPU ([run_single_demo.sh:39](/home/quenouille/drone/scripts/run_single_demo.sh:39)). Disable slowlane by default when `RENDER_BACKEND=nvidia` or while armed/OFFBOARD until an A/B gate proves stable sim RTF, PX4 time synchronization, detector cadence, VRAM, and p95 latency. Sidecar failure or OOM must only clear deep annotations and return `UNAVAILABLE`; the 10 Hz detector remains untouched.

5. **HIGH — networking is plausible but brittle and exposed.** The container currently uses Docker’s default bridge because no `--network` is supplied ([run_single_demo.sh:64](/home/quenouille/drone/scripts/run_single_demo.sh:64)). `172.17.0.1` is common, not contractual; `--dns` does not establish it, and `-p 8000:8000` is unrelated container-to-host egress. Docker officially provides `--add-host host.docker.internal:host-gateway`, resolving to the configured default-bridge host address ([Docker dockerd documentation](https://docs.docker.com/reference/cli/dockerd/#configure-host-gateway-ip)).

   Add that host mapping and use `http://host.docker.internal:8100`. Do not bind blindly to `0.0.0.0`; bind to the discovered bridge gateway address. Also require a generated bearer token, exact frame-size validation, bounded request bodies/prompts, and single-request concurrency. Binding all interfaces exposes unauthenticated GPU work to the LAN.

6. **MEDIUM — SAM2 claim is true, but the selected API is unnecessarily stateful.** Ultralytics 8.4.103 contains its own SAM2 builder and exports `SAM2DynamicInteractivePredictor`; no external `sam2` package is needed. Point/box prompts work, but the dynamic predictor requires `obj_ids` plus `update_memory=True` on a fresh state. It is intended for sequential memory-backed tracking, as current [official documentation](https://docs.ultralytics.com/models/sam-2/#dynamic-interactive-segment-and-track) confirms.

   For stateless one-shot `/segment`, use the documented public `SAM("sam2.1_t.pt").predict(points=…, labels=…)` or `bboxes=…` API. Use the dynamic predictor only if M4 later adds cross-frame SAM tracking.

7. **MEDIUM — YOLO-World workflow is correct but underspecified.** `yolov8s-worldv2.pt`, `YOLOWorld`, and `set_classes([...])` are supported in current [Ultralytics documentation](https://docs.ultralytics.com/models/yolo-world/#set-prompts). Installed 8.4.103 also auto-promotes `YOLO("…-worldv2.pt")` to `YOLOWorld`.

   Pin `ultralytics==8.4.103`, serialize `set_classes()+predict`, canonicalize/cache the last vocabulary, and cap prompt count/length. Do not imply YOLO-World supplies masks; it supplies boxes, while SAM supplies unlabeled masks.

8. **MEDIUM — RLE reuse needs an explicit spatial contract.** `rle_encode` itself is suitable: row-major Boolean data, leading-zero run first ([types.py:72](/home/quenouille/drone/agents/vision/types.py:72)). But `Detection.mask` is currently a **box-local** RLE ([backends.py:68](/home/quenouille/drone/agents/vision/backends.py:68)), serialized with box-derived dimensions ([pipeline.py:39](/home/quenouille/drone/agents/vision/pipeline.py:39)). A SAM full-frame mask cannot be called “the same contract” without dimensions and origin.

   Crop SAM masks to their tight `xyxy`, encode only that crop, and return `{xyxy, mask:{rle,w,h}, centroid, area_px, score}`. Pin round-trip and empty/malformed-mask tests.

9. **MEDIUM — semantic and range claims overreach.** SAM segments but does not identify. YOLO-World answers “is one of these prompted concepts present?”, not open-ended scene captioning. Projecting a facade centroid onto the ground via [projection.py:53](/home/quenouille/drone/agents/perception/projection.py:53) yields a point behind the building, not reliable object range.

   `look` should report label, confidence, bbox, bearing, and at most an explicitly named `ground_intersection` from a visible bottom-center. `pinpoint` should report an unlabeled mask/bearing unless paired with a YOLO-World box. Deep outputs must remain advisory and must not become flight targets without map/fast-contact confirmation.

10. **Scope — M3 is bloated; M4 acceptance is too anecdotal.** M3 combines slowlane, overlays, FP correlation, and interactive identification ([plan:168](/home/quenouille/.kimi-code/sessions/wd_drone_579721d6036f/session_b05e3ce4-f723-4164-8975-d72f674361a7/agents/main/plans/black-hawk-taskmaster-groot.md:168)). Two screenshots/chat scenarios cannot validate open-vocabulary quality or flight coexistence.

## Concrete change requests

1. Make M1a the fake-testable service/schema/auth layer; M1b provisions pinned official weights and runs real point, box, vocabulary, color-order, latency, and VRAM tests.
2. Remove all copying from `~/perception-lab` at [plan:97](/home/quenouille/.kimi-code/sessions/wd_drone_579721d6036f/session_b05e3ce4-f723-4164-8975-d72f674361a7/agents/main/plans/black-hawk-taskmaster-groot.md:97). Download pinned official URLs and verify hard-coded expected SHA-256 values before writing manifests.
3. Make M2 only on-demand `look` and pixel/box `pinpoint`, with injected raw-frame source, nonblocking MCP calls, freshness fields, and graceful failure.
4. Gate M3 on measured GPU coexistence. Add only the skip-if-busy slowlane and advisory topic first; defer identify-click UI unless it remains necessary.
5. Add integration tests for real container→host resolution, bearer rejection, payload limits, concurrent vocab isolation, sidecar kill/restart, annotation expiry, hung-call estop latency, and exact-frame FP overlap.
6. M4 must include a recorded pursuit-aspect set with per-concept recall/FP numbers, SAM point/box mask IoU, p50/p95 end-to-end latency, fast-lane 10 Hz regression, and PX4/Gazebo load telemetry—not merely two successful conversations.