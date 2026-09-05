# Review — ICD v3 perception-lab integration layer

**Scope:** §0.5, §1 (`Detection.tid`), §6.2–6.4, §6.8, §14 of `2026-07-19-interface-specification.md`, verified against `~/perception-lab` source and design spec §6.1/M2.

**Overall:** the architectural idea is right — image-space association swappable, world-space fusion fixed, everything optional behind extras, stdlib default. But the v3 layer was written from memory of the lab, not from its code. The lab's orchestration file (`inference.py`) — the one place that actually solves mode-switching, lock ordering, and tracker reset — is absent from the §14 mapping, and every problem it solves resurfaces in the ICD unsolved. Two contracts are unimplementable as written; several vendoring claims don't match the donor source.

---

## Blockers

**B1. The default registry tracker (`iou-gate`) cannot implement the protocol it's registered under.**
Evidence: ICD §6.8 defines `TargetTracker.update(frame, dets) -> (cx, cy, conf)` — a pure image-space contract (faithful to lab `trackers/base.py:25-27`). The same section's table registers `iou-gate` (DEFAULT) as "world-space NN gate on projected ground points." A world-space gate needs `attitude_at(sim_stamp)`, the camera model, and the track's predicted world position — none of which cross the `(frame, dets)` signature. The one entry every M2 build must instantiate is the one entry that can't be built. Worse, §6.4 already says "hits come from `tracker.update()` when locked, **else the default world-space gate**" — so if `iou-gate` were implementable, it would be a second copy of the fallback that VisionContacts contains anyway.
Fix: remove `iou-gate` from the registry. Make the registry tracker `Optional`: `VISION_TRACKER` unset/`none` (default) ⇒ no registry tracker, designated contact uses VisionContacts' built-in world-space gate like every other contact. The registry then only ever holds genuinely image-space trackers, and the association/fusion split becomes real instead of aspirational.

**B2. The Detector↔tracker seam has no API, and one legal reading contains a lock-time track-ID race.**
Evidence: §6.3 describes the track-mode switch only in a comment — Detector's signature list has no `set_track_mode`/`set_association` method, and no way to obtain the tracker YAML: the ported `TargetTracker` protocol (§6.8) has no `tracker_yaml()`; in the lab it's a duck-typed extra on `DnnAssociationTracker` (`trackers/dnn.py:16-17`) consumed by the inference thread (`inference.py:70-71`). Timing is also ambiguous: §6.3 says the switch happens "at tracker (re)selection," but §6.4 creates the tracker inside `designate()`. If track mode engages at designation, `DnnAssociationTracker.lock()` runs against the latest `InferenceResult`, which was produced in **predict** mode — every `Detection.tid` is `-1` (§1 default, matching `models.py:83`) — so the tracker adopts `_tid = -1` and `update()` (`dnn.py:27-28`) never matches a real track again. The lab never hits this because its loop runs track mode continuously from the moment a DNN tracker is selected, frames before any click (`inference.py:70-74`).
Fix: specify (a) `Detector.set_association(needs_track_ids: bool, tracker_yaml: str | None)` called once at assembly from `VISION_TRACKER` — track mode is on for the whole process when a DNN tracker is selected, exactly like the lab; (b) `tracker_yaml` as a protocol attribute (or registry metadata); (c) `lock()` MUST return None when the nearest detection has `tid == -1` (stale/pre-switch frame).

---

## Major

**M1. The `perception-sam` extra doesn't satisfy the code it unlocks.**
§0.5 says `perception-sam` = the `sam2` package. The vendored implementation imports `from ultralytics.models.sam import SAM2DynamicInteractivePredictor` (`sam.py:8`) plus `cv2` and `numpy` (`sam.py:6-7`) — it never imports `sam2`. Installing `perception-sam` alone is an ImportError; the availability guard would then *remove* sam2 from the registry, silently, on a machine where the user believes they installed it. Also `model="sam2_t.pt"` is hardcoded CWD-relative (`sam.py:44`) — the drone's weights dir differs.
Fix: either declare `perception-sam` ⊇ `perception-dnn` (it rides ultralytics) or re-vendor against the actual `facebookresearch/sam2` package; parameterize the checkpoint path from `weights_dir`.

**M2. `FollowTarget` vendored "unchanged" violates three of the ICD's own laws.**
The upstream tests exist (`tests/test_follow.py` — the "unit-tested upstream" claim is true), but the class: (a) uses wall-clock `time.time()` (`follow.py:44,60`) where §0.3/§6.5 mandate sim time; (b) owns its own loss constants — `coast_max=20` misses ≈ **4 s at the 5 Hz detector** and `lost_ttl=3.0` wall-seconds — against `TrackerConfig.coast_s=1.0`/`lost_s=2.0`, re-creating the two-owners-of-`lost_s` problem the v2 review explicitly killed (Fable-minor-3): the EKF would drop the track at 2 s while FollowTarget still reports COAST until ~4 s; (c) applies 0.5-EMA smoothing to hits before anything downstream (`follow.py:48-49`) — pre-filtered measurements entering `CvEkf` have correlated noise and lag, breaking §6.5's covariance contract; (d) computes bearing as `dx/frame_w * hfov` with a **75° webcam default** (`follow.py:13,65`) — the linear approximation design §3.3/§6.3 explicitly replaced with full pinhole.
Fix: don't vendor it whole. Take the four-state skeleton with injected time and thresholds derived from `TrackerConfig` (frames↔seconds via `dt_nominal_s`), and delete EMA/bearing/command/trail — or state in §6.8 exactly which fields are dead and that hits pass to fusion raw.

**M3. Tracker hit → EKF measurement contract is missing, and hits are centers, not footpoints.**
All three vendored trackers return patch/mask/box **centers** (`dnn.py:28`, `template.py:44`, `sam.py:74`). Design §6.3's projection contract intersects the ground plane at the box **bottom-center** — projecting a center systematically overestimates range for ground contacts (the ray exits above the footpoint). Nothing in §6.4/§6.5/§6.8 says which `CvEkf` measurement model consumes a hit (projected `update_xy`? `update_bearing`?), what sigma a hit carries, or what `ContactView.conf` becomes when the tracker's conf is the hardcoded `0.0` of template/SAM hits — as written, the `detect` grammar would print `conf 0.00` on the operator's own designated target.
Fix: one paragraph in §6.5: registry hits are converted to bearing/elevation and consumed as `update_bearing` with `sigma_bearing_deg` (range only via ToF), OR footpoint-corrected using the tracker's mask/box bottom before `update_xy`; `ContactView.conf` holds the last *detection* confidence, tracker conf is ignored.

**M4. Tracker compute runs on the wrong thread.**
§6.4 has VisionContacts (asyncio-loop-confined, §0.2c) drive `tracker.update()`. CSRT is tens of ms; SAM2 the lab itself warns is "seconds per frame" on CPU (`inference.py:49-50`) — and `create_tracker` defaults to `device="cpu"` in the ICD (§6.8) vs `"cuda"` upstream (`trackers/__init__.py:10`). Running that inside the pipeline task blocks the event loop — the exact starvation class v2 created `VisionPipeline` to prevent. The lab runs trackers on its inference thread (`inference.py:82`).
Fix: drive `lock()`/`update()` on the Detector thread and carry the designated hit (and mask) inside `InferenceResult`; this also structurally fixes B2's lock-frame race. VisionContacts consumes the hit, never the tracker.

**M5. `UltralyticsBackend` lacks `reset()`, so per-call `tracker_yaml` is a mirage.**
Ultralytics instantiates the tracker on the first `track(persist=True)` call and reuses it thereafter — a different `tracker_yaml` passed later to `infer_tracked()` is silently ignored. The lab knows this precisely: `YoloRunner.reset()`'s docstring ("forces fresh trackers after a tracker-config change," `models.py:51-53`) and `set_tracker` → `runner.reset()` (`inference.py:31`). The ICD ports "model cache" but not `reset`; `VisionContacts.reset()` (evals per-cell clean slate, §6.4) has no path to flush tracker ID state between cells.
Fix: make `tracker_yaml` a set-once property paired with `UltralyticsBackend.reset()`; wire eval `soft_reset` and (optionally) `clear_designation` to it.

**M6. Backend-without-track-mode is unspecified.**
`needs_track_ids` gates on the *tracker* extra, but the capability lives in the *backend*: `VISION_TRACKER=botsort` + `VISION_BACKEND=blob|onnx` has no `infer_tracked` to call. §0.5's availability guards can't catch it (perception-dnn may be installed while the blob backend is active).
Fix: add `supports_track: bool` to `DetectorBackend` (§6.1); `available_trackers()` intersects installed extras with the active backend's capability; mismatch at boot ⇒ fall back to default + the §6.2-style legible log line.

**M7. `conf` propagated into track mode breaks ByteTrack-family association.**
`infer_tracked(frame, conf, tracker_yaml)` implies the Detector's `conf=0.45` is passed to `model.track()`. Ultralytics track mode deliberately runs at low conf (~0.1) so the trackers' own thresholds (`track_high_thresh=0.25`, `track_low_thresh=0.1`) perform two-stage association — pre-filtering at 0.45 amputates ByteTrack's entire low-score recovery stage. The lab passes no conf in track mode (`models.py:69-70`).
Fix: in track mode, run the model at conf≈0.1 and apply the Detector's `conf` as a post-filter for contact birth only, preserving tid continuity underneath.

---

## Minor

**m1. `lock()` return type mistyped.** ICD: `tuple[str, float, float, int, float]`; template and SAM return `tid=None` (`template.py:36`, `sam.py:64`). Normalize to `-1` in the port or type it `int | None`.

**m2. Lock-by-name is unresolvable inside an image-space tracker.** The tracker has no contact-name↔detection mapping; only VisionContacts does. Specify that VisionContacts resolves `name` → the designated contact's last image position and calls `lock(xy=...)`; keep `name` on `designate()`, off the tracker protocol.

**m3. §14 mapping omits `inference.py` and `overlay.py`.** `inference.py` is the file whose logic v3 is actually absorbing (mode switch, lock-request queue, tracker hot-swap + runner reset, thread ownership) — leaving it unmapped is how B2/M4/M5 got lost. Add a row: `inference.py` → absorbed by Detector thread + VisionContacts, enumerating which behavior lands where; `overlay.py` → not taken.

**m4. The OBB claim is false through the ported extraction path.** `extract_detections` reads only `res.boxes` (`models.py:78-85`); OBB models populate `res.obb` and leave `boxes` empty — `yolo26*-obb.pt` yields zero detections in the lab code as-is, yet "OBB-DOTA" is claimed in §6.2's docstring, §14, and design §6.1. Add OBB extraction (xyxyxyxy→AABB) or drop the claim (the M2 "immediate value" story survives on visdrone alone).

**m5. Frame conversion and helper homelessness.** `Frame.rgb` is RGB888 bytes; ultralytics treats bare ndarrays as BGR, and cv2 template trackers/`cv2.resize` need ndarrays too — the bytes→array and channel-order conversion is nowhere specified (wrong order silently degrades detection). Also `nearest_detection`/`Detection.dist` (`models.py:19-24,89-95`), used by all three vendored trackers, have no ported home and ICD `Detection` (xyxy, cx/cy properties) doesn't provide them.

**m6. `mask()` plumbing asserted but not carried by any signature.** §6.8's table says sam2's "mask silhouette for the beam association," but `BeamAssociator.associate` (§6.6) takes masks only via `Detection.mask`; tracker masks are ndarray bools (`sam.py:19-22`), and the RLE encoding step/owner is unspecified.

**m7. Unavailable/unknown `VISION_TRACKER` failure path unspecified.** Lab `create_tracker` raises `ValueError` (`trackers/__init__.py:17`). §6.2 gives backends "skipped with a legible log line"; trackers get no equivalent. Specify: validate at assembly (not first `designate()`), fall back to default, log.

**m8. Design-spec staleness (should join ICD §12's list).** Design §6.2 still says "Rejected: ByteTrack/BoT-SORT (vendor-only, heavy deps)" while §6.1 now adopts them as extras. Design §6.4 defers SAM-class behind a 3-condition trigger (incl. "licensing settled" and explicitly avoiding AGPL-ultralytics paths for FastSAM) while the ICD ships an env-selectable SAM2 tracker that runs *through* ultralytics. Both need reconciliation notes.

## Nit

**n1.** `iou-gate` is labeled "stdlib, **vendored**" in §6.8's table — it doesn't exist in the lab; it's new code. And it's a distance gate, not IoU.
**n2.** `discover_weights` is a module function upstream (`models.py:27-32`), staticmethod in the ICD — fine, but the "model cache" bullet is vestigial for a single-model backend (`__init__(…, model_name)` vs the lab's multi-model `model_names`).
**n3.** `create_tracker(name, device="cpu")` flips the upstream `"cuda"` default; combined with the lab's own SAM2-on-CPU warning, a cpu-default sam2 selection deserves the same legible warning path the lab has (`inference.py:49-50`).
**n4.** Design M2 places the vendored lifecycle at `agents/vision/follow.py`; ICD §6.8 never gives it a path. Align.

---

## Go / no-go for M2

**NO-GO for the v3 integration layer as specified; GO for M2 on the pre-v3 baseline.**

The baseline M2 path (blob/ONNX backend + built-in world-space gate, rangefinder, projection) is untouched by most of this — *except* that B1 puts the broken registry contract on the **default** path (`iou-gate` is the DEFAULT entry), so an implementing agent following §6.8 literally hits an unimplementable protocol on day one. Fixing B1 (registry tracker becomes `Optional`, default none) plus B2 (an actual Detector API for the mode switch, lab-style always-on track mode, tid≠-1 guard) makes the layer buildable; M1–M7 are each a paragraph of spec work and should land in the same v3.1 pass, since every one of them produces *silent* misbehavior (dead second-stage association, wall-clock lifecycles, range bias, event-loop stalls) rather than crashes — exactly the class of spaghetti this ICD exists to prevent. The split itself — swappable image-space association, fixed world-space EKF, one designated contact, availability-guarded extras — is sound and verified consistent with design §6.1/M2's intent; it's the contracts under it that need to catch up to the donor code they cite.

Sources: [Ultralytics track mode docs](https://docs.ultralytics.com/modes/track) · [botsort.yaml defaults](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/trackers/botsort.yaml) · [Re-ID and new tracker backends (ocsort/deepocsort/fasttrack/tracktrack)](https://github.com/ultralytics/ultralytics/issues/24846)
