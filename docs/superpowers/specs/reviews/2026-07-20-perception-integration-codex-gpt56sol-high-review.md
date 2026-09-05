## Verdict

**NO-GO for M2 with the v3 integration as specified.** The baseline M2 detector work remains viable, but the tracker integration requires contracts that do not yet exist. The critical failures are the unwired detector track-mode transition, the impossible `iou-gate` abstraction, and an association result too weak to feed projection, beam association, and fusion safely.

## Blockers

1. **Detector track mode has no control path**

   **Lenses:** Fidelity, implementability.

   The ICD says the detector switches from `infer()` to `infer_tracked()` when the selected tracker has `needs_track_ids=True`, but:

   - `Detector` receives no tracker and exposes no `set_tracker()`/`configure_tracking()` method ([ICD §6.3](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:643)).
   - `VisionContacts` creates the tracker only during `designate()` ([ICD §6.4](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:688)).
   - `TargetTracker` does not expose the donor’s required `tracker_yaml()` method ([ICD §6.8](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:785); [donor dnn.py](/home/quenouille/perception-lab/perception_lab/trackers/dnn.py:16)).
   - The donor explicitly resets the cached model when tracker configuration changes, then passes the tracker YAML to every inference ([inference.py](/home/quenouille/perception-lab/perception_lab/inference.py:27), [inference.py](/home/quenouille/perception-lab/perception_lab/inference.py:69)). No equivalent reset/reconfiguration contract exists in v3.

   This also creates an ordering deadlock: a DNN tracker cannot lock until it sees tracked detections with valid `tid`s, but v3 appears to lock the tracker during designation, before the detector has switched modes.

   **Fix:** Introduce an explicit thread-safe contract such as:

   ```python
   class TrackingMode:
       needs_track_ids: bool
       tracker_yaml: str | None

   Detector.configure_tracking(mode: TrackingMode) -> int  # returns change generation
   UltralyticsBackend.reset_tracking() -> None
   ```

   Select/configure the tracker before inference, wait for an `InferenceResult` from the new generation, then call `lock()`. Validate at construction that a `needs_track_ids` tracker is paired with a track-capable backend. An explicitly selected incompatible pair must fail with a typed configuration error or sensing-degraded boot—not silently fall back.

2. **`iou-gate` cannot implement the advertised `TargetTracker` protocol**

   **Lenses:** Spaghetti risk, implementability.

   Section 6.8 defines association as image-space and gives trackers only `Frame` and `Detection`s, but calls `iou-gate` a “world-space NN gate on projected ground points” ([ICD §6.8](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:774), [tracker table](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:807)). That gate requires vehicle pose, support plane, EKF predictions, covariance, and metric thresholds—none are in `TargetTracker`.

   It is also not IoU: the existing algorithm is nearest-neighbour gating in world metres ([ICD §6.5](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:726)). Putting it in the image-association registry collapses the association/fusion split v3 claims to enforce.

   Consequently, the default cannot compose coherently in `VisionContacts.update()`: the same world gate would be both the general multi-contact association mechanism and a designated-contact `TargetTracker`.

   **Fix:** Keep world NN/NIS gating wholly inside `VisionContacts`/fusion. Use `VISION_TRACKER=world-nn` or `none` to mean “no designated image tracker,” outside the `TargetTracker` registry. If a default registry tracker is required, implement a real image-space box-IoU tracker and name it accordingly.

3. **The association payload cannot drive projection or beam association**

   **Lenses:** Fidelity, spaghetti risk, implementability.

   The proposed tracker returns only `(cx, cy, conf)` ([ICD §6.8](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:792)). That discards:

   - box geometry needed for bottom-centre/footpoint projection ([design §6.3](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1226));
   - detection identity/index needed to reserve a designated measurement;
   - class, `tid`, and mask;
   - the tracker mask needed by `BeamAssociator`.

   `BeamAssociator` accepts `detections` and a `designated_index`, but not `tracker.mask()` ([ICD §6.6](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:743)). Template or SAM hits may have no corresponding detection index at all.

   The adapted lock-by-name contract is also unimplementable as written. `Detection` contains no contact name ([ICD §1](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:183)); donor trackers lock by pixel point ([base.py](/home/quenouille/perception-lab/perception_lab/trackers/base.py:21)).

   Finally, ICD `Frame` is RGB bytes, while donor template/SAM implementations require an ndarray with `.shape` and pass it to OpenCV/Ultralytics ([ICD Frame](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:129), [template.py](/home/quenouille/perception-lab/perception_lab/trackers/template.py:24), [sam.py](/home/quenouille/perception-lab/perception_lab/trackers/sam.py:47)). RGB/BGR conversion is unspecified.

   **Fix:** Replace the tuple with a first-class `AssociationHit`, for example:

   ```python
   class AssociationHit:
       detection_index: int | None
       xyxy: tuple[float, float, float, float] | None
       aim_px: tuple[float, float]
       conf: float
       tid: int | None
       mask: bytes | None
   ```

   Define whether `aim_px` is centroid, mask footpoint, or box bottom-centre. Pass an explicit pixel seed/detection index into `lock()`; resolve contact name to that seed inside `VisionContacts`. Specify one RGB→tracker-array conversion, shape, dtype, and channel order. Extend beam association to consume the designated `AssociationHit` directly.

4. **M2 has no legal pipeline/contact object**

   **Lens:** Implementability.

   The milestone table keeps `GzPoses` through M2 and introduces `VisionContacts` at M3a ([ICD §0.6](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:106)). But `VisionPipeline` requires `VisionContacts` and calls `contacts.update()` ([ICD §6.7](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:752)), while M2 must publish `/pilot/detections` ([design M2](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1372)). `GzPoses` cannot satisfy that contract.

   **Fix:** Either:

   - move a minimal `VisionContacts` implementation into M2; or
   - define a `DetectionConsumer`/`ContactSnapshotProvider` protocol and allow `contacts=None`, producing raw-detection snapshots with empty contacts until M3a.

## Major findings

5. **The vendored `FollowTarget` lifecycle conflicts with the ICD’s health and time model**

   **Lenses:** Fidelity, spaghetti risk.

   Donor behavior is:

   - `coast_max=20` misses, meaning more than four seconds at the ICD’s 5 Hz detector;
   - `lost_ttl=3.0`;
   - wall-clock `time.time()`;
   - 75° HFOV;
   - 0.5 EMA smoothing of image positions ([follow.py](/home/quenouille/perception-lab/perception_lab/follow.py:12), [follow.py](/home/quenouille/perception-lab/perception_lab/follow.py:41)).

   The ICD requires one `lost_s=2.0` owner using sim time ([ICD §6.5](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:709)); the camera HFOV is 69° ([design projection](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:449)). Mapping donor `COAST/LOST` directly to contact health therefore creates a second, incompatible loss clock. `LOST` also expires to `IDLE`, which has no specified contact-health mapping.

   If FollowTarget’s EMA output is fed into the EKF, fusion receives lagged, correlated pseudo-measurements and its NIS/noise assumptions become invalid.

   **Fix:** Specify the adapted `FollowTarget` API explicitly. Inject sim time and calibrated HFOV; derive coast/loss deadlines from `TrackerConfig`; keep LOST semantics persistent until the contact owner removes/rebinds it. Feed raw association measurements to the EKF; reserve EMA output for display or pointing only.

6. **Designated and non-designated association can double-consume detections**

   **Lens:** Spaghetti risk.

   Section 6.8 says one designated contact uses the registry tracker and all others use the default world gate ([ICD §6.8](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:820)). It never says that the designated detection is removed from the multi-contact candidate pool. One detection can therefore update the designated track and a second track in the same frame.

   “Tracker hit when locked, else default gate” is also ambiguous ([ICD §6.4](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:690)): does one missed image hit immediately fall back to nearest-world contact, defeating COAST/LOST and inviting an ID switch, or only after lifecycle loss?

   **Fix:** Define deterministic ordering:

   1. designated association first;
   2. reserve its detection index/measurement;
   3. world-gate remaining detections to other contacts;
   4. on designated miss, coast without fallback;
   5. reacquire only through an explicit gated/reconfirmation transition.

7. **Optional extras do not match donor imports or availability behavior**

   **Lenses:** Fidelity, implementability.

   The donor registry imports every implementation eagerly, so missing OpenCV or Ultralytics breaks registry import; it has no `available_trackers()` ([trackers/__init__.py](/home/quenouille/perception-lab/perception_lab/trackers/__init__.py:1)).

   More seriously, `perception-sam = sam2` does not match the donated tracker. The donor imports:

   - `ultralytics.models.sam.SAM2DynamicInteractivePredictor`;
   - `cv2`;
   - `numpy` ([sam.py](/home/quenouille/perception-lab/perception_lab/trackers/sam.py:6)).

   Official Ultralytics documentation likewise installs and imports this predictor through `ultralytics`, not standalone `sam2` ([Ultralytics SAM2 documentation](https://github.com/ultralytics/ultralytics/blob/main/docs/en/models/sam-2.md)).

   **Fix:** Decide which implementation is intended:

   - faithful donor port: `perception-sam` must include Ultralytics/Torch and its OpenCV requirements; or
   - standalone Meta `sam2`: rewrite the adapter and stop calling it a port of `sam.py`.

   Registry factories must lazy-import their implementation. `available_trackers()` should test both dependency availability and backend compatibility without importing optional modules at package import time.

8. **Environment selection and failure behavior are incomplete**

   **Lens:** Implementability.

   Only `VISION_BACKEND` and `VISION_TRACKER` are named. M2 implementers still have to invent:

   - weights directory and model-selection variables;
   - device/half settings and accepted device grammar;
   - tracker YAML/config override;
   - invalid-value behavior;
   - explicit-selection versus automatic-fallback behavior;
   - missing model/weights behavior;
   - missing tracker extra during `designate()`;
   - a DNN tracker paired with blob/ONNX;
   - whether `clear_designation()` disables track mode and resets IDs.

   “Skipped with a legible log line” does not define what object gets constructed afterward ([ICD §6.2](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:639)). `designate()` returns `None`, so it cannot report an unavailable tracker to `TrackSession`.

   **Fix:** Add a validated `VisionConfig` and a compatibility matrix. Explicit user selections should fail closed into sensing-degraded/`NOT_READY`; only an `auto` value should fall back. Make designation failure typed and observable.

9. **“Any `.pt`, including OBB” and segmentation support are overstated**

   **Lens:** Fidelity.

   The donor extractor reads only `res.boxes` and never extracts masks ([models.py](/home/quenouille/perception-lab/perception_lab/models.py:76)). Ultralytics OBB results live in `result.obb`, not `result.boxes` ([official OBB result contract](https://docs.ultralytics.com/tasks/obb)). Therefore a direct port does not support OBB-DOTA as claimed and does not populate the ICD’s RLE `Detection.mask` for segmentation weights.

   The donor also never passes its `conf` argument to `model.predict()` or `model.track()` ([models.py](/home/quenouille/perception-lab/perception_lab/models.py:60)). V3’s confidence contract will differ unless implemented deliberately.

   **Fix:** Either narrow the claim to detect/segment box output, or specify extraction branches for boxes, masks, and `result.obb`, including conversion of OBB to the agreed axis-aligned or polygon schema. Pass `conf=conf` explicitly and define RLE encoding tests.

10. **`tid` typing and validity are inconsistent**

   **Lenses:** Fidelity, implementability.

   ICD `Detection.tid=-1` matches the donor. But ICD `TargetTracker.lock()` says its returned `tid` is an `int`, while donor template and SAM trackers return `None` ([template.py](/home/quenouille/perception-lab/perception_lab/trackers/template.py:36), [sam.py](/home/quenouille/perception-lab/perception_lab/trackers/sam.py:64)).

   Donor DNN lock also accepts `tid=-1`; it then follows the first future detection with `tid=-1`, which is not an identity ([dnn.py](/home/quenouille/perception-lab/perception_lab/trackers/dnn.py:19)).

   **Fix:** Use `tid: int | None` internally and normalize backend `-1` to `None`. DNN `lock()` must reject missing/negative IDs. Specify that IDs reset on mode/model/tracker changes and are not contact IDs.

11. **Registry fidelity is described too strongly**

   **Lens:** Fidelity.

   Exact deltas from the donor are:

   - donor has no `available_trackers()`;
   - taxonomy is stored in `config.py`, not solely in the registry ([config.py](/home/quenouille/perception-lab/perception_lab/config.py:10));
   - donor default is `botsort`, not `iou-gate` ([config.py](/home/quenouille/perception-lab/perception_lab/config.py:13));
   - donor `create_tracker()` defaults to CUDA, ICD defaults to CPU;
   - `iou-gate` is new, not donated;
   - lock changes from positional pixel coordinates to name/point keywords;
   - donor tracker masks are ndarrays, not RLE bytes;
   - donor `FollowTarget` exposes `clear()/lock()/step()/snapshot()`, none of whose signatures appear in the ICD.

   These adaptations can be sensible, but section 14 should call them a redesign derived from the lab—not a faithful protocol/registry port.

   **Fix:** Add an explicit adaptation table and pin the provenance header to donor commit `26e9431a193f4cf4f051d086d23ac0133dd305a6`.

12. **M2 scope and the companion design do not consistently place SAM/tracker functionality**

   **Lens:** Implementability.

   The design still declares SAM-class segmentation deferred, while v3 exposes a SAM2 tracker extra and calls it immediate integration. M2 creates the registry and FollowTarget, but designation and `VisionContacts` do not arrive until later, and the M2 tests/gate contain no tracker, `tid`, mode-switch, or extra-availability checks ([design M2](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1359)).

   The claim that VisDrone/OBB weights provide immediate M2 value is also conditional: the donor contains no `.pt` weights, and v3 specifies neither provisioning nor model-selection environment variables.

   **Fix:** State whether M2 delivers dormant adapters or an exercised capability. If adapters land at M2, add contract tests for all selection/failure paths and provide an explicit weight installation/provenance procedure. Otherwise move designated trackers to M3a/M3b.

## Minor findings

- “Stdlib end-to-end (`iou-gate` + blob/ONNX)” is false: ONNX Runtime and the EKF use NumPy ([ICD §14](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:1097)). Say “baseline dependencies only; no optional extras.”

- The companion design’s rejection of ByteTrack/BoT-SORT should be qualified as rejection for the fixed world-fusion/default path; v3 now accepts them as optional designated image association. Otherwise §§6.1 and 6.2 read as contradictory.

- Template and SAM trackers emit confidence `0.0` and potentially synthetic classes `"template"`/`"sam2"`. The ICD must state that designation preserves the existing contact’s name/class and define whether tracker confidence participates in measurement gating.

## M2 release condition

M2 can proceed with this integration only after:

1. defining detector tracking-mode configuration/reset/generation;
2. removing world NN gating from the image tracker protocol;
3. replacing tuple hits with a projection- and beam-capable association DTO;
4. defining the M2 pipeline without `VisionContacts`;
5. reconciling FollowTarget with sim-time health/loss;
6. defining deduplication and designated reacquisition;
7. correcting extras and all environment/failure contracts;
8. adding tracker/mode/`tid`/availability tests to the M2 gate.

Until those are incorporated, implementers would have to invent safety-relevant contracts, so the appropriate decision is **NO-GO**.