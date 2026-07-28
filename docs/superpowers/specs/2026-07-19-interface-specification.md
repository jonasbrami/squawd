# Interface Specification (ICD) — Single-Drone Rebuild

- **Date:** 2026-07-19 (v3.1 — perception-lab integration, revised after dual review)
- **Status:** REVIEWED (v2) + perception-lab integration (v3.1, dual-reviewed)
- **Companion to:** [2026-07-18-single-drone-rebuild-design.md](2026-07-18-single-drone-rebuild-design.md)
  (v4.2 — the *what/why*; this document is the *contract*). Where they disagree,
  the design spec wins on rationale, this ICD wins on signatures.
- **Codebase basis:** `feat/dynamic-scenarios` @ `7622618`; target branch
  `rebuild-single-drone`.
- **Audience:** implementing agents. Implement exactly these signatures, states,
  and dependency rules and the modules cannot tangle.

---

## 0. Global rules (the anti-spaghetti law)

### 0.1 Layering and allowed imports (normative)

The matrix below is the **normative adjacency list** consumed verbatim by
`tests/test_import_rules.py` (AST scan over `agents/**`). It governs `agents.*`,
ROS (`rclpy`, `std_msgs`, `px4_msgs`), and `gz.*` edges — the spaghetti-relevant
ones. Third-party libraries (starlette, uvicorn, av, websockets, numpy,
onnxruntime, PIL, mavsdk, claude_agent_sdk) are governed by requirements pinning
instead, and allowed anywhere their row doesn't forbid them.

```
core → world → perception → vision → flight → pilot
                  ↑                     ↑
          observatory (core only)   evals (test-side consumer)
```

| Package | MAY import | MUST NOT import |
|---|---|---|
| `agents/core` | stdlib, gz.*, rclpy, px4_msgs, std_msgs, PIL | any `agents.*` |
| `agents/world` | stdlib | `agents.*`, ROS, gz |
| `agents/perception` | stdlib | `agents.*`, ROS, gz, numpy |
| `agents/vision` | `agents.core.*`, `agents.world`, `agents.perception`, numpy, onnxruntime, PIL | `agents.flight`, `agents.pilot`, ROS, gz |
| `agents/flight` | `agents.core.*`, `agents.world`, `agents.perception`, mavsdk, claude_agent_sdk | `agents.vision`, `agents.pilot`, ROS, gz |
| `agents/pilot` | anything below | — (assembler + loop; no domain logic) |
| `agents/observatory` | `agents.core.*`, std_msgs, px4_msgs | `agents.vision`, `agents.flight`, `agents.pilot` |
| `evals` | `agents.*` (except pilot) | — |

Three deliberate decouplings make cycles impossible:

1. **`flight` never imports `vision`.** `ContactProvider`/`TargetDesignator` are
   `typing.Protocol`s defined in `flight` (§5.1); all shared DTOs live one layer
   down in `agents/core/contact.py` (§1) so neither side imports the other.
2. **`observatory` never imports `vision`/`flight`.** Frames via `core.GzCameras`;
   fusion state via the `/pilot/detections` topic only.
3. **`vision` never imports ROS/gz.** Frames and range samples arrive as `core`
   objects; `Detection`/backend types live in `vision/types.py` (§6.1), so
   `detector.py` ↔ `backends.py` cannot cycle.

### 0.2 Concurrency model

| Thread/loop | Owns | Rule |
|---|---|---|
| gz callback threads (`core.GzCameras`, `core.GzPoses`, `core.GzRangeProvider`) | latest-value stores | NEVER awaits; writes immutable snapshots under a short lock; any reduction (ray-bundle reduce, impairment) is bounded, pure, and side-effect-free |
| rclpy spin thread (`core.RosBridge`) | `LatestStore`, TopicLog appends | same rule |
| Detector thread (`vision.Detector`) | inference state | the only thread that runs the model; input `Frame` snapshots in, `InferenceResult` out under a lock |
| **`VisionPipeline` asyncio task (pilot process)** | detector→contacts ticking + atomic `PerceptionSnapshot` assembly | independent task started by `PilotAgent.run()` — NEVER awaited inside agent turns or tool calls (a 120 s `track` must not starve it); polls `detector.wait_next(after_seq)` at ≥ detector hz |
| asyncio loop (pilot process) | agent turn, FlightOps, estop arbiter | all awaits live here |
| VideoHub task (observatory process) | H.264 encode | encode-only |

Rules: (a) no `await` in gz/rclpy callbacks; (b) cross-thread handoff ONLY via
immutable snapshots or `LatestStore.get`; (c) `VisionContacts` is
asyncio-loop-confined except `poses()/sim_time()/velocities()/observation()`
which return deep copies under a lock (readers: the pipeline's snapshot
assembler, eval sampler).

### 0.3 Units, frames, time

- World frame: ENU metres; headings radians (0=N, +CW toward E) in code; degrees
  only at tool/prompt boundaries (documented per signature). Bearings: degrees,
  relative to boresight, `+` = right. Image coords: px, origin top-left.
- Sim time (float seconds, gz stamps) for all measurement joins; monotonic for
  rates/timeouts. `RangeSample.sample_time` sim; `receive_time` monotonic.
- Angle interpolation is ALWAYS shortest-angle (never linear across ±π).
- Tool result codes (§9) are machine-readable prefixes: `CODE: human text`.

### 0.4 Naming

Tools: snake_case, 13 final (§5.5; **12 at M1** — `detect` lands at M2, §0.6).
Contact IDs: `vis_{cls}_{k}` (opaque). Topics: `/pilot/*` (§8.1). Result codes:
SNAKE_UPPER. Files: one responsibility per file; no `utils.py`/`helpers.py`/
`common.py`.

### 0.5 Dependencies added

Baseline (always installed): `onnxruntime` + `numpy` (pinned; numpy confined to
`agents/vision/`). No torch, no ultralytics, no OpenCV in the baseline.

Optional extras (availability-guarded in the registries, §6.2/§6.8 — an extra's
absence removes its entries, never breaks import; registry factories lazy-import):

| Extra | Packages | Unlocks |
|---|---|---|
| `perception-dnn` | `torch`, `ultralytics`, `lap` | `UltralyticsBackend` (yolo26 family, yolo11-seg, visdrone fine-tunes) + DNN association trackers (botsort, bytetrack, ocsort…) |
| `perception-cv` | `opencv-contrib-python` | template trackers (csrt, kcf, mosse — class-less lock) |
| `perception-sam` | = `perception-dnn` + `perception-cv` (the donor's SAM tracker rides ultralytics' `SAM2DynamicInteractivePredictor` + cv2, NOT the standalone `sam2` package — Fable-M1/Codex-M7) + the `sam2_t.pt` checkpoint in the weights dir | SAM2 mask tracker |

License note: ultralytics is AGPL-3.0 — consistent with design §6.1 (open-source
research repo); SAM2 weights Apache-2.0; visdrone community weights carry their
own licenses (documented in `models/README.md` at M2).

**`VisionConfig`** (validated at assembly — Codex-M8):

```python
@dataclass(frozen=True)
class VisionConfig:
    backend: str = "auto"        # blob | onnx | ultralytics | auto
    weights_dir: str = "models/"
    model: str | None = None     # .pt/.onnx filename; required for ultralytics|onnx
    device: str = "cpu"          # cpu | cuda
    half: bool = False           # fp16 (cuda only)
    tracker: str = "none"        # none | botsort | bytetrack | ... | auto
    tracker_yaml: str | None = None   # override for DNN family
    @classmethod
    def from_env(cls) -> "VisionConfig"      # VISION_* vars, validated
```

Rules: explicit selections FAIL CLOSED (`VisionConfigError` → sensing-degraded
boot with a legible `/pilot/chat` health line); only `auto` values fall back
(with a log). Weights are provisioned explicitly (copy/symlink into
`weights_dir` + manifest entry — the lab's own weights are host symlinks and
are NOT assumed present).

### 0.6 Milestone compatibility of the interfaces (Codex-B1)

Final constructors are capability-aware — M-phase objects are Optional:

| Milestone | tools | FlightOps contacts | detector | rangefinder |
|---|---|---|---|---|
| M1 | **12** (13-current − `look`) | `GzPoses` (evals only; interactive: none) | `None` | `None` |
| M2 | 13 (+`detect`) | `GzPoses` still | ColorBlob→Onnx | GzRangeProvider |
| M3a | 13 | `VisionContacts` replaces `GzPoses` | ✓ | ✓ |
| M3b | 13 | + designation/acquisition/ToF fusion | ✓ | ✓ |

`PilotAgent`/`make_pilot_options`/`VisionPipeline` therefore take
`detector: Detector | None` etc. (§5.5, §7). `detect` is NOT stubbed into M1 —
the M1 gate asserts 12 tools. **`VisionPipeline` also accepts
`contacts: VisionContacts | None`**: at M2 (pre-M3a) it publishes raw-detection
snapshots (empty `contacts` array, `beam`/`track` IDLE) so `/pilot/detections`
and the overlay work before fusion exists; `detect` at M2 formats raw dets +
projection (Codex-B4). Registry trackers land at M2 as WIRED-but-dormant
adapters (exercised from M3a; their selection/failure paths are contract-tested
at M2).

---

## 1. Data schemas (the shared nouns)

Neutral DTOs live in **`agents/core/contact.py`** (bottom layer; both `vision`
and `flight` import it). Detector-internal types live in `agents/vision/types.py`
(§6.1). All `@dataclass(frozen=True)` unless noted.

```python
# agents/core/camera.py
@dataclass(frozen=True)
class Frame:
    seq: int                     # strictly increasing per drone, ≥1
    sim_stamp: float
    width: int
    height: int
    rgb: bytes                   # RGB888

# agents/core/contact.py
ContactHealth = str   # "MEASURED" | "COASTING" | "ACQUIRING" | "LOST"
RangeSource   = str   # "tof" | "geom" | "bearing"
PositionSource = str  # "measured" | "predicted" | "none"

@dataclass(frozen=True)
class ContactView:               # the full read model of ONE tracked contact
    name: str                    # "vis_{cls}_{k}"
    cls: str
    conf: float
    e: float | None              # None while bearing-only-newborn (never locked)
    n: float | None
    z: float | None              # None while bearing-only-newborn; predicted
                                 # value allowed only with position_src="predicted"
    position_src: PositionSource
    ve: float
    vn: float
    bearing_deg: float | None    # always present (camera measurement)
    elevation_deg: float | None
    range_m: float | None
    range_src: RangeSource
    range_conf: float
    health: ContactHealth
    age_s: float                 # sim-time seconds since last measurement
    foot_px: tuple | None = None  # last accepted det footpoint (u, v) — the
                                 # acquisition SM's image-servo aim (§3.10)
    bbox_xyxy: tuple | None = None  # last accepted det bbox — the vertical-
                                 # centre reference (erosion-robust, §3.10)

@dataclass(frozen=True)
class TargetLockEvent:
    contact_id: str
    sim_stamp: float
    tool: str                    # "track" | "goto"

# agents/core/rangefinder.py
@dataclass(frozen=True)
class RangeSample:
    sample_time: float
    receive_time: float
    range_m: float | None        # None = no valid return (NOT free space)
    min_m: float
    max_m: float
    fov_rad: float
    quality: float               # 0..1
    status: str                  # VALID|LOW_SIGNAL|SATURATED|OUT_OF_RANGE|STALE|EDGE_MIX
    seq: int

# agents/vision/types.py
@dataclass(frozen=True)
class Detection:
    cls: str                     # backend label; ColorBlobBackend pins "target"
    conf: float
    xyxy: tuple[float, float, float, float]
    mask: bytes | None = None    # RLE (counts: list[int] varint, row-major);
                                 # codec defined in vision/types.py docstring
    tid: int | None = None       # backend track id when run in track mode;
                                 # backend's -1 is NORMALIZED to None (Codex-M10);
                                 # never a contact id; resets on mode/model change
    @property
    def cx(self) -> float: ...
    @property
    def cy(self) -> float: ...

@dataclass(frozen=True)
class InferenceResult:
    frame: Frame
    detections: list[Detection]
    completed_monotonic: float
    generation: int = 0                         # configure_tracking() generation
    designated_hit: "AssociationHit | None" = None   # §6.8, Detector-thread-produced

@dataclass(frozen=True)
class AssociationHit:              # §6.8 — one designated-target image-space hit
    detection_index: int | None   # index into the frame's dets; None for
                                  # template/SAM hits with no matching detection
    xyxy: tuple[float, float, float, float] | None   # box when known
    aim_px: tuple[float, float]   # FOOTPOINT (mask/box bottom-center) when
                                  # derivable, else patch centroid — stated per impl
    conf: float                   # tracker-side confidence (display only;
                                  # NEVER fed to the EKF or printed as det conf)
    tid: int | None               # backend track id; NOT a contact id
    mask: bytes | None            # RLE (§1 codec) when the impl provides one

@dataclass(frozen=True)
class TrackingMode:                # §6.3 — Detector.configure_tracking() input
    needs_track_ids: bool
    tracker_yaml: str | None = None

@dataclass(frozen=True)
class BeamAssociation:
    status: str                  # ASSOCIATED | AMBIGUOUS | EDGE | OFF_TARGET
                                 # | OUT_OF_ENVELOPE | NO_SAMPLE
    detection_index: int | None
    residual_m: float | None     # sample minus predicted range
    footprint_px: float
    reason: str

# agents/vision/pipeline.py — the ONE atomic wire/authority object (schema v1)
@dataclass(frozen=True)
class PerceptionSnapshot:
    schema_version: int          # = 1
    frame_seq: int
    sim_stamp: float
    dets: list[Detection]
    contacts: list[ContactView]
    detector: dict               # {"healthy": bool, "latency_ms": float}
    beam: dict                   # {"status": LOCKED|SEARCHING|NO_RETURN|EDGE_MIX|
                                 #  OUT_OF_ENVELOPE|IDLE, "target": str|None,
                                 #  "range_m": float|None}
    track: dict                  # {"state": IDLE|DESIGNATED|ACQUIRING|RANGE_LOCKED|
                                 #  WORLD_TRACKED|COASTING|LOST,
                                 #  "target": str|None, "gap_m": float|None}
```

JSON wire form of `PerceptionSnapshot` (published on `/pilot/detections`, §8.1):
same fields; `null` for unavailable numbers (never NaN/inf); floats rounded to 2
decimals (`sim_stamp` at 2dp is still unique per frame: 100 ms frame gap > 10 ms
granularity).

---

## 2. `agents/core` — data buses [PORT+]

Import reality (corrects ICD v1): `core/store.py` and `core/geo.py` are sim-free
pure Python; `core/bus.py` imports rclpy at module scope and `core/camera.py`
imports gz at module scope TODAY — porting keeps that (unit tests import them
only in the sim container; refactoring to lazy imports is optional, not mandated).
`core/gzposes.py` keeps its lazy-import constructor pattern.

### 2.1 `core/store.py` [PORT unchanged]

```python
class LatestStore:
    def set(self, key: str, value: Any) -> None
    def get(self, key: str) -> Any | None
class TopicLog:
    def __init__(self, bridge: "RosBridge", topic: str, msg_type: Any, qos: Any) -> None
    def append(self, text: str) -> None
    def all(self) -> list[str]
    def since(self, n: int) -> tuple[list[str], int]
```

### 2.2 `core/bus.py` [PORT + 2 QoS profiles]

```python
PX4_QOS: QoSProfile      # BEST_EFFORT/VOLATILE/KEEP_LAST(5)
CHAT_QOS: QoSProfile     # RELIABLE/TRANSIENT_LOCAL/KEEP_LAST(100) — chat only
CMD_QOS: QoSProfile      # RELIABLE/VOLATILE/KEEP_LAST(10) — commands, NO replay
STATE_QOS: QoSProfile    # RELIABLE/TRANSIENT_LOCAL/KEEP_LAST(1) — NEW: latched
                         # LATEST for /pilot/detections (no stale replay burst)

class RosBridge:   # (node_name default unchanged: "swarm_bridge" → kept verbatim)
    def __init__(self, node_name: str = "swarm_bridge") -> None
    def subscribe(self, topic, msg_type, qos=PX4_QOS, callback=None) -> None
    def publisher(self, topic, msg_type, qos=CHAT_QOS)
    def publish(self, topic, msg_type, msg, qos=CHAT_QOS) -> None
    def latest(self, topic): ...
    def start(self) -> None
    def shutdown(self) -> None
def publish_str(bridge, topic, text) -> None
```

### 2.3 `core/camera.py` [PORT+]

```python
CAM_TOPIC = ("/world/{world}/model/{model}_{i}/link/OakD-Lite/base_link"
             "/sensor/IMX214/image")          # model="x500_depth" | "x500_depth_range"
CAM_MODEL = os.environ.get("CAM_MODEL", "x500_depth")

class GzCameras:
    def __init__(self, n: int, world: str | None = None) -> None
    def seq(self, i: int) -> int
    def has(self, i: int) -> bool
    def raw(self, i: int) -> tuple[int, int, bytes] | None
    def jpeg(self, i: int, quality: int = 55, max_px: int | None = None) -> bytes | None
    def snapshot(self, i: int) -> Frame | None     # THE consumer API (C1)
```

- `snapshot()` is the only call new code makes (detector, VideoHub, accuracy
  tooling); `raw()/jpeg()` survive for the observatory `/frame` fallback.
- **`jpeg_b64()` is DELETED** (its only consumer was `look`, removed in v4.2);
  `tests/test_video*.py` migrate to `jpeg()`.
- Invariants: `seq` strictly increases per `i`; `snapshot` under one lock hold.

### 2.4 `core/gzposes.py` [PORT, eval-only + one method]

```python
class GzPoses:
    ANCHOR_TOPIC = "/movers/anchor"
    def __init__(self, world: str, names: list[str]) -> None
    def poses(self) -> dict[str, tuple[float, float, float]]
    def sim_time(self) -> float
    def velocities(self) -> dict[str, tuple[float, float]]   # NEW (O3): always {}
    def anchor(self) -> None
```

Fully satisfies `ContactProvider` (§5.1). It has no `ranges()`/`health()`/
`observation()` — callers use `getattr` defaults (`{}`, `"MEASURED"`, `None`).

### 2.5 `core/rangefinder.py` [NEW]

```python
RANGE_TOPIC = "/world/{world}/model/{model}_0/link/range_link/sensor/lidar/scan"
               # verified live at M2 (2026-07-21): gz derives the topic from the
               # sensor NAME ("lidar"), not its type (gpu_lidar)

class RangeProvider(Protocol):
    def latest(self) -> RangeSample | None: ...
    def robust_at(self, sim_stamp: float, *, window_s: float = 0.12,
                  sync_tolerance_s: float = 0.05) -> RangeSample | None: ...
        # Hampel/median-of-residuals about a linear fit over samples with
        # sample_time <= sim_stamp, newest-first, window_s deep; returns the
        # selected sample with its ORIGINAL sample_time; None when nothing is
        # within sync_tolerance_s of sim_stamp (staleness honesty).

class GzRangeProvider:
    def __init__(self, topic: str, *, bundle: int = 9,
                 impair: "ImpairmentModel | None" = None) -> None
        # gz callback: min-reduce bundle, flag EDGE_MIX on high intra-bundle
        # spread, stamp, store. Bounded, pure, side-effect-free (§0.2).
    def latest(self) -> RangeSample | None
    def robust_at(self, sim_stamp: float, **kw) -> RangeSample | None

class ImpairmentModel(Protocol):
    def apply(self, hits_m: list[float | None], ideal_range_m: float | None
              ) -> tuple[float | None, float, str]:
        """(bundle hits, RAW IDEAL sensor range of the true ray — never GzPoses
        oracle truth) -> (range_m|None, quality, status). Injects distance-scaled
        noise, reflectivity max, dropouts, latency. STALE is stamped by
        latest()/robust_at() at READ time from sample age, not here."""
```

### 2.6 `core/contact.py` [NEW]

The DTO module of §1 (`ContactView`, `ContactHealth`, `RangeSource`,
`PositionSource`, `TargetLockEvent`). Pure stdlib, no logic.

### 2.7 `core/telemetry.py` [NEW — the W1 feeder, Fable-MAJOR-3 / Codex-B10]

```python
class Px4StateRecorder:
    """Subscribes PX4 telemetry on `bridge` and feeds a duck-typed sink (World).
    Lives in core so World stays ROS-free; msg objects arrive duck-typed."""
    def __init__(self, bridge: "RosBridge", sink, i: int = 0) -> None
        # subscribes /px4_{i}/fmu/out/vehicle_local_position and
        # /px4_{i}/fmu/out/vehicle_attitude.
        # Clock alignment: PX4 µs-since-boot → sim-time offset captured ONCE at
        # first message pair (documented; reset on anchor). Before alignment,
        # sink.note_* is not called (pose_at() returns None — honest).
        # NED→ENU conversion is SHARED with World.drone_state (same helper,
        # not duplicated). Quaternion → roll/pitch/yaw here; shortest-angle
        # interpolation is the sink's job (§0.3).
    def start(self) -> None
```

### 2.8 `core/geo.py`, `core/singleton.py` [PORT unchanged]

---

## 3. `agents/world` — world model [PORT+]

Pure Python, no ROS/gz imports; telemetry arrives via `Px4StateRecorder` (§2.7).

```python
class World:
    def __init__(self, path: str | None = None) -> None
    @property
    def buildings(self) -> list[dict]
    @property
    def movers(self) -> list[dict]
    @property
    def spawn_x(self) -> float
    @property
    def spawn_spacing(self) -> float
    def drone_state(self, bridge, i: int)                    # latest (legacy)
    def world_xy(self, bridge, i: int)
    def resolve_xy(self, name: str, bridge, n_drones: int)
    # W1 buffers — fed ONLY by Px4StateRecorder
    def note_pose(self, t: float, e: float, n: float, alt: float, heading: float) -> None
    def note_attitude(self, t: float, roll: float, pitch: float, yaw: float) -> None
    def pose_at(self, t: float) -> tuple[float, float, float, float] | None
    def attitude_at(self, t: float) -> tuple[float, float, float] | None
```

Invariants: ~4 s append-only deques under a lock; `*_at` interpolates
(shortest-angle for heading/attitude), never extrapolates (None outside
coverage).

---

## 4. `agents/perception` — pure trig/text [PORT + projection]

As ICD v1 (§4.1/§4.2 verbatim), plus: `scan_text` mover tuples are
`(name, e, n, z | None)` and render `alt unk` when z is None — where z-None
comes from `ContactView.range_src == "bearing"` (§6.3's `all_views()`, NOT from
a held float). `erode_box()` and `footprint_in_region()` as in v1.

---

## 5. `agents/flight` — ops, controller, protocols, tools

### 5.1 `flight/contacts.py` [NEW — the decoupling seam, Protocols ONLY]

```python
@runtime_checkable
class ContactProvider(Protocol):
    """Minimum read contract. GzPoses and VisionContacts both satisfy it.
    Extended reads (ranges/health/observation) are OPTIONAL and consumed via
    getattr with defaults ({}, 'MEASURED', None)."""
    def poses(self) -> dict[str, tuple[float, float, float]]: ...
    def sim_time(self) -> float: ...
    def velocities(self) -> dict[str, tuple[float, float]]: ...

class TargetDesignator(Protocol):
    """Implemented by VisionContacts (the SOLE owner of range sampling, beam
    association, and fusion). FlightOps calls these; GzPoses does not implement
    (getattr guard → designation is a no-op with ground truth)."""
    def designate(self, name: str, *, support_z: float | None = None,
                  context: "TrackingContext | None" = None) -> None: ...
    def clear_designation(self) -> None: ...

@dataclass(frozen=True)
class TrackingContext:
    mode: str                    # "shadow" | "intercept"
    commanded_speed: float
    own_alt: float
    task_ceiling_m: float | None

# Optional extended read (getattr-guarded):
#   def observation(self, name: str) -> ContactView | None
#   def all_views(self) -> list[ContactView]
#   def ranges(self) -> dict[str, tuple[float, str, float]]
#   def health(self, name: str) -> ContactHealth
```

### 5.2 `flight/envelope.py` [NEW — §13 item 3, full semantics]

```python
@dataclass(frozen=True)
class Envelope:
    max_alt_m: float = 80.0          # soft tool-layer ceiling
    max_speed_mps: float = 12.0
    geofence_radius_m: float = 300.0 # hard PX4 layer, derived at connect()
    geofence_alt_m: float = 80.0     # (distinct roles, not duplication:
                                     #  max_* = pre-checks; geofence_* = PX4 GF_*)
    center_e: float = 0.0            # launch/home (world frame), set at connect()
    center_n: float = 0.0

class EnvelopeViolation(ValueError):
    def __init__(self, code: str, text: str) -> None: ...

def check_takeoff(env: Envelope, alt: float) -> None
def check_goto(env: Envelope, e: float, n: float, alt: float,
               task_ceiling_m: float | None = None) -> None
def check_orbit(env: Envelope, e: float, n: float, radius: float, alt: float) -> None
    # validates the PERIMETER (center ± radius), not just the center
def check_fly_endpoint(env: Envelope, e: float, n: float, alt: float) -> None
def check_speed(env: Envelope, speed: float) -> None
    # all raise EnvelopeViolation("INVALID_PARAM", "<legible>") — never silent-clamp
    # at the tool boundary. Controller-internal clamps (clamp_ref_alt, O6 alt
    # bias) are documented and REPORTED in result text, not exceptions.
```

`run_mission` admission (Codex-B13): arbitrary authored code CANNOT be
statically envelope-checked — the envelope governs the 12 fixed tools; inside
`run_mission`, PX4's own geofence is the only hard bound. This is stated
plainly in the tool description and the doc, not papered over. `connect()`
derives `GF_MAX_HOR_DIST/GF_MAX_VER_DIST/GF_ACTION` FROM the Envelope instance
(today `drone.py:55-57` hardcodes 300/80) so the two layers cannot diverge;
PX4 param-set failure ⇒ degraded (log + continue), matching today's behavior.

### 5.3 `flight/track.py` [PORT + `feed_direct`]

As ICD v1 (constants, `TargetEstimator`, `intercept_t_go`, `control_ref`,
`clamp_ref_alt`, `TrackLog`), plus `TargetEstimator.feed_direct(ve, vn)` (O3).

### 5.4 `flight/ops.py` [PORT+ — O1..O6 + emergency surface]

```python
class FlightOps:
    def __init__(self, drone, world, bridge, i: int, n: int,
                 contacts: ContactProvider | None = None,
                 envelope: Envelope | None = None) -> None
        # NOTE: NO rangefinder param (v1 removed it) — VisionContacts is the
        # SOLE ToF owner (Codex-B4). FlightOps meets range only through
        # contacts' fused output.

    async def take_off(self, altitude=10.0) -> str
    async def fly(self, north=0.0, east=0.0, up=0.0, wait=True) -> str
    async def goto(self, target="", east=None, north=None, up=None,
                   heading="travel", wait=True) -> str
    async def orbit(self, target="", east=None, north=None, radius=12.0,
                    speed=3.0, direction="cw", alt=None) -> str
    async def hover(self, seconds=0.0) -> str            # capped at 120 s
    async def set_speed(self, speed=5.0) -> str
    async def face(self, target="") -> str
    async def land(self) -> str
    def scan(self) -> str
    async def run_mission(self, code: str, timeout=None) -> tuple[bool, str]
    async def track(self, target="", mode="shadow", alt=12.0, duration_s=60.0,
                    within_m=15.0, speed=12.0, standoff_east=0.0,
                    standoff_north=0.0, support_z: float | None = None) -> str
    # emergency surface (Codex-B3): public, idempotent, safe under cancellation
    async def emergency_hold(self) -> str
    async def emergency_land(self) -> str
```

Behavioral contracts:

- **Errors:** typed — `InvalidParamError` (bad input), `NotReadyError`
  (preconditions), `BlockedError` (arrival timeout — `_await_arrival` RAISES it
  now instead of a success-suffix), `EnvelopeViolation` (extends
  `InvalidParamError`). No stringly-typed prefixes (v1 hack deleted).
- **O1:** mover reads via `contacts` (+ getattr-extended reads).
- **O2 (loss):** while the contact is absent: keep streaming the last reference
  (the setpoint stream never stops mid-call). Loss decision is `health()`-first
  when the provider offers it (LOST state), else an absence timer
  (`lost_s=2.0 s`) — one owner of the constant: `VisionContacts`; the timer is
  only the fallback for health-less providers (Fable-minor-3). On loss: break,
  offboard stop in `finally`, return `LOST: ...` (degraded completion,
  `is_error=False`).
- **O3:** velocity dispatch via `contacts.velocities()` (`feed_direct` vs legacy
  finite-difference).
- **O4:** `_resolve_xy` accepts contact names WITH a position (measured or
  predicted); newborn bearing-only contacts (position None) resolve via bearing
  for `face` only (yaw needs no range) and are rejected for `goto`/`orbit` with
  `NOT_READY: contact <id> is bearing-only — track it to acquire range first`.
- **O5 (`face`):** blocks until |err| ≤ 5° held for 3 consecutive telemetry
  samples, or 5 s — returns the settled heading and, on timeout, the residual
  error. 5° is the CAMERA-facing criterion only; ToF beam-lock is the tighter
  acquisition criterion owned by the TrackSession (O6).
- **O6 (TrackSession — the sole owner of the mission-level acquisition SM):**
  `track` on a bearing-only contact: `designate(name, support_z, context)` →
  ACQUIRING (yaw to bearing, alt bias ±3 m via `clamp_ref_alt` + task ceiling)
  → beam-lock = 3 consecutive ASSOCIATED samples within 1.0 s → RANGE_LOCKED →
  WORLD_TRACKED. Retries: 3, backoff 2/4/8 s, then `BLOCKED: could not lock beam
  on <id>`. Fusion is envelope-gated (shadow, ≤3 m/s, |Δz|≤3 m, beam on target)
  or opportunistic — never required.
- **Cancellation:** `emergency_*` are idempotent, shielded, and safe to call
  while a tool is in flight (they coordinate through the arbiter, §7.1).

### 5.5 `flight/tools.py` [REWRITE — inverted construction, Fable-B1/Codex-B3]

```python
def make_pilot_options(ops: FlightOps, *,
                       detect_text: Callable[[str | None], str] | None = None,
                       report: Callable[[str], None],
                       env: dict | None = None, model: str | None = None,
                       cli_path: str | None = None) -> ClaudeAgentOptions
```

- The assembler (pilot/run.py, §7.2) builds THE ONE `FlightOps` and passes it
  here AND to `PilotAgent` — estop and tools share the instance (no closure
  archaeology, no second instance). `detect_text` is composed in the pilot layer
  (which may import vision) — `flight` never sees `Detector` (Fable-MAJOR-6);
  at M1 (`detect_text=None`) the `detect` tool is not registered (12 tools, §0.6).
- Registers 13 MCP tools (12 at M1): `take_off, fly, goto, orbit, hover,
  set_speed, face, land, report, scan, detect, run_mission, track`.
- `ClaudeAgentOptions(mcp_servers={"pilot": server},
  allowed_tools=[mcp__pilot__*], tools=[], setting_sources=[], env, model,
  cli_path, system_prompt=<design §4.1>)`.
- Tool JSON schemas: full objects with explicit `required` arrays +
  `additionalProperties: false` (Codex-M1-missing-contracts), even though
  shorthand optionality works (design §10 adjudication 2 — belt and braces).
- Wrappers map typed errors via the §9 table, IN ORDER: `EnvelopeViolation/
  InvalidParamError → INVALID_PARAM`; `NotReadyError → NOT_READY`;
  `BlockedError → BLOCKED`; `asyncio.TimeoutError → TIMEOUT`;
  `asyncio.CancelledError → ESTOPPED` (it is BaseException — caught
  explicitly, never by `except Exception`); `Exception → INTERNAL` (logged with
  traceback server-side).
- `detect` result grammar (Fable-MAJOR-5 — the most LL-visible contract):

```
2 detections (frame #412, 0.2s old):
vis_target_0 rover conf 0.91 ahead-right ~34m ToF q0.9 (at E52 N18) | vis_box_1 box conf 0.66 left (bearing only)
```

  One line header + pipe-joined entries: `id cls conf bearing-word [range src
  (world)]`; `bearing only` when no range; stale annotation when >0.5 s old;
  empty ⇒ `"nothing detected (frame #412, 0.1s old)"`; degraded ⇒
  `"NOT_READY: sensing degraded (detector down)"`. Class labels: blob backend
  pins `"target"` (IDs `vis_target_k`, matching design §2.5's example).

---

## 6. `agents/vision` — the local perception pipeline [NEW]

numpy/onnxruntime confined here. No flight/pilot/ROS/gz imports.

### 6.1 `vision/types.py` [NEW]

`Detection`, `InferenceResult`, `BeamAssociation`, `DetectorBackend` (Protocol),
`BackendError` — per §1. No imports beyond stdlib + `agents.core.contact` (Frame
comes via `agents.core.camera` — allowed, §0.1).

### 6.2 `vision/backends.py`

```python
class DetectorBackend(Protocol):
    supports_track: bool               # can run a tid-producing track mode?
    def infer(self, frame: Frame, conf: float) -> list[Detection]: ...

class ColorBlobBackend:      # interim, PIL-only, pins cls="target", real HSV mask
    supports_track = False
    def __init__(self, hsv_lo: tuple, hsv_hi: tuple, min_area_px: int = 40) -> None
    def infer(self, frame: Frame, conf: float) -> list[Detection]

class OnnxBackend:
    supports_track = False
    def __init__(self, model_path: str, manifest_path: str, device: str = "cpu") -> None
        # manifest: {"sha256": str, "source": str, "trained_at": str,
        #            "output": {"layout": "1x84x8400"|"seg-v1", "nms": "embedded"|"external"}}
        # verifies SHA-256 before load; mismatch raises BackendError
    def infer(self, frame: Frame, conf: float) -> list[Detection]

class UltralyticsBackend:    # OPTIONAL extra `perception-dnn` — derived from
    """ultralytics-compatible .pt (yolo26 detect/seg family, yolo11-seg,
    visdrone fine-tunes; OBB-DOTA only via the obb branch below).
    Derived from perception-lab's YoloRunner (weights discovery, model cache)."""
    supports_track = True
    def __init__(self, weights_dir: str, model_name: str,
                 device: str = "cpu", half: bool = False) -> None
    @staticmethod
    def discover_weights(weights_dir: str) -> list[str]     # *.pt, sam* excluded
    def infer(self, frame: Frame, conf: float) -> list[Detection]
        # model.predict(conf=conf); extraction branches on the Results object:
        # res.boxes -> Detection; res.masks -> RLE mask (seg models);
        # res.obb -> polygon->AABB (else the model yields ZERO dets — Fable-m4)
    def infer_tracked(self, frame: Frame, conf: float,
                      tracker_yaml: str) -> list[Detection]
        # model.track(persist=True, tracker=tracker_yaml, conf=TRACK_CONF) with
        # TRACK_CONF = 0.1 (Fable-M7: two-stage association needs the low-score
        # stage); the Detector's `conf` applies as a POST-filter for contact
        # birth only — tid continuity underneath is never disturbed
    def reset_tracking(self) -> None
        # drops the cached model so a NEW tracker_yaml takes effect (the lab's
        # YoloRunner.reset() — without it the first tracker persists silently);
        # wired to configure_tracking() changes and eval soft_reset

def frame_to_array(frame: Frame):      # -> numpy uint8 ndarray (H, W, 3) BGR
    """THE one conversion site (Fable-m5): Frame.rgb is RGB888 bytes;
    ultralytics/OpenCV consume BGR ndarrays. Everything cv-ish goes through
    here — no ad-hoc channel juggling anywhere else."""
```

Backend selection via `VisionConfig` (§0.5): explicit selections fail closed
(`VisionConfigError` → sensing-degraded boot); `auto` falls back through
ultralytics → onnx → blob with a legible log line.

### 6.3 `vision/detector.py`

```python
class Detector:
    """Owns the model session + ONE daemon thread. Lifecycle states:
    INIT -> RUNNING -> DEGRADED (3 consecutive frame failures | stale camera
    >2 s) -> STOPPED. start() raises BackendError (pilot → degraded boot)."""
    def __init__(self, cameras: "GzCameras", backend: DetectorBackend, *,
                 i: int = 0, hz: float = 5.0, conf: float = 0.45) -> None
    def start(self) -> None
    def stop(self, timeout: float = 2.0) -> None      # joins the thread, deadline
    def detections(self) -> InferenceResult | None    # newest completed (lock)
    def wait_next(self, after_seq: int, timeout: float) -> InferenceResult | None
        # NEW (Codex-B12): post-face freshness — callers wait for an inference
        # NEWER than after_seq (e.g. captured pre-face)
    def latency_ms(self) -> float
    def healthy(self) -> bool                          # RUNNING and fresh

    # --- tracking mode + designated pursuit (§6.8; Fable-B2, Codex-B1) ---
    def configure_tracking(self, mode: "TrackingMode") -> int:
        """Set ONCE at assembly (thread-safe), before start() or between
        inferences — never mid-frame. mode = TrackingMode(needs_track_ids,
        tracker_yaml). When needs_track_ids is True the thread runs
        backend.infer_tracked() for EVERY frame (always-on, exactly like the
        lab — so every frame carries valid tids long before any lock request).
        Calls backend.reset_tracking() on change. Returns a generation
        counter; wait_next() results carry it, and a lock request is only
        honored against results of the CURRENT generation."""
    def request_lock(self, seed_xy: tuple[float, float] | None = None,
                     seed_index: int | None = None) -> None
        """The designation slot (mirrors the lab's LockRequests): thread-safe
        one-shot; the Detector thread creates/locks the registry tracker
        against the NEXT inference of the current generation and starts
        emitting InferenceResult.designated_hit."""
    def clear_lock(self) -> None
```

`InferenceResult` gains (§1): `generation: int` and
`designated_hit: AssociationHit | None`. Validation (Codex-B1/M8): at
assembly, a tracker with `needs_track_ids=True` paired with a backend where
`supports_track` is False is a TYPED `VisionConfigError` for explicit
selections (fail closed → sensing-degraded boot) and a fall-back-to-`none`
plus legible log for `auto`.
```

### 6.4 `vision/contacts.py`

```python
class VisionContacts:                # ContactProvider + TargetDesignator
    """SOLE owner of: range sampling (RangeProvider), beam association
    (BeamAssociator), per-track CV-EKF fusion, track health. asyncio-loop-
    confined except the deep-copy readers (§0.2c)."""
    def __init__(self, world: World, rangefinder: RangeProvider | None = None,
                 i: int = 0, config: "TrackerConfig | None" = None) -> None
    # ticking — called ONLY by VisionPipeline with an atomic InferenceResult
    def update(self, result: InferenceResult) -> None
    # ContactProvider
    def poses(self) -> dict[str, tuple[float, float, float]]
        # contacts WITH a numeric position (measured or predicted) — matches
        # "trackability"; newborn bearing-only contacts are NOT in poses()
    def sim_time(self) -> float
    def velocities(self) -> dict[str, tuple[float, float]]
    # extended reads
    def ranges(self) -> dict[str, tuple[float, str, float]]
    def health(self, name: str) -> ContactHealth
    def observation(self, name: str) -> ContactView | None
    def all_views(self) -> list[ContactView]
    # TargetDesignator
    def designate(self, name: str, *, support_z: float | None = None,
                  context: TrackingContext | None = None) -> None
        # also drives the registry tracker (§6.8): create_tracker(VISION_TRACKER)
        # .lock(frame, dets, name=name); the designated contact's hits then come
        # from tracker.update() when locked, else the default world-space gate
    def clear_designation(self) -> None
    # lifecycle
    def reset(self) -> None                            # evals: per-cell clean slate
```

### 6.5 `vision/tracker.py` [NEW — the fusion contract, Codex-B7]

```python
@dataclass(frozen=True)
class TrackerConfig:
    dt_nominal_s: float = 0.2            # 5 Hz detector
    v_max_mps: float = 12.0
    gate_m: float = 5.0                  # max(2 * v_max * dt, 3 * sigma_pred)
    nis_max: float = 9.21                # chi2(2 dof, 99%) innovation gate
    confirm_hits: int = 2                # two-hit confirmation on source change
    birth_hits: int = 2                  # consecutive gated hits to open a track
    coast_s: float = 1.0                 # sim-time
    lost_s: float = 2.0                  # sim-time — THE single lost constant
    rebind_window_s: float = 2.0         # name-rebind gate after drop
    sigma_geom_m: float = 2.0            # support-plane measurement noise
    sigma_tof_m: float = 0.15            # ToF measurement noise (in-envelope)
    sigma_bearing_deg: float = 1.5
    accel_max_mps2: float = 4.0          # process model clamp

class CvEkf:
    """Constant-velocity EKF, state (e, n, ve, vn), numpy(4x4). Measurement
    models: XY (geom), RANGE (tof, bearing known), BEARING (held range)."""
    def predict(self, dt: float) -> None
    def update_xy(self, e: float, n: float, sigma_m: float) -> float      # -> NIS
    def update_range(self, rng: float, bearing_deg: float, sigma_m: float) -> float
    def update_bearing(self, bearing_deg: float, sigma_deg: float) -> float
```

Rules (all numeric defaults are TEST VECTORS, not prose): NN-gating on
projected ground points at `gate_m`; NIS ≤ `nis_max` or the measurement is
rejected; source changes require `confirm_hits` before the new source's
covariance applies; a newborn bearing-only track publishes `e=n=z=None`
(`position_src="none"`); a previously-locked track slipping to bearing-only
keeps a predicted position marked `position_src="predicted"`; name-rebind
inside `rebind_window_s`; tracks older than `lost_s` drop and report LOST.
`age_s`/coast/loss use SIM time.

### 6.6 `vision/beam.py`

```python
class BeamAssociator:
    def __init__(self, *, hfov_deg: float = HFOV_DEG,
                 cam_to_beam_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None
        # frames: camera = x-right/y-down/z-forward; beam co-boresighted with a
        # documented rigid offset (sim constant; calibration on hardware)
    def associate(self, frame: Frame, detections: list[Detection],
                  sample: RangeSample, attitude: tuple[float, float, float],
                  designated_index: int | None, predicted_range: float | None,
                  predicted_sigma: float | None) -> BeamAssociation
        # footprint disc (beam half-angle at sample.range_m) must lie inside
        # exactly ONE mask | 22%-eroded box; then |residual| <= 3*predicted_sigma
        # (consistency gate); statuses per BeamAssociation (§1)
```

### 6.7 `vision/pipeline.py` [NEW — Codex open-point 4 / Fable-B2 home]

```python
class VisionPipeline:
    """Owns the detector→contacts ticking and the atomic PerceptionSnapshot.
    Detector stays a pure inference thread (reusable by perceive_eval
    fixtures); PilotAgent starts/stops the pipeline and relays snapshots."""
    def __init__(self, detector: Detector | None, contacts: VisionContacts) -> None
    async def run(self) -> None      # independent asyncio task: wait_next() →
                                     # contacts.update(result) → assemble ONE
                                     # PerceptionSnapshot → publish (§8.1)
    def latest(self) -> PerceptionSnapshot | None
    def stop(self) -> None
```

Snapshot assembly is atomic wrt the detector thread: one `InferenceResult`
(read under the detector's lock) + one `contacts` deep-read (under its lock) →
one frozen snapshot. The UI and evals never re-join independently-read state
(Codex-B6).

### 6.8 `vision/trackers/` — swappable designated-target trackers (derived from perception-lab)

The **designated** contact (the ToF/acquisition target) may get a pluggable
pursuit tracker, derived from `~/perception-lab` @ `26e9431`
(redesign, not a verbatim port — adaptation table in §14). Two roles stay
strictly separated:

- **association** (this section, OPTIONAL and swappable): "where is the
  designated target in THIS image" — image-space hits.
- **fusion** (§6.5 `CvEkf`, not swappable): "where is it in the WORLD" —
  projects hits and filters them with source covariances. The **default for
  EVERY contact, designated included, is VisionContacts' built-in world-space
  NN/NIS gate** — that gate is NOT a registry tracker (v3.1: it can never
  satisfy the image-space signature; it needs attitude/camera/EKF state).

**Default is no registry tracker** (`VISION_TRACKER=none`): the designated
contact uses the same world-space gate as every other contact. The registry
only ever holds genuinely image-space trackers.

```python
# vision/trackers/base.py
# AssociationHit / TrackingMode live in vision/types.py (§1) — trackers import
# them from there (no cycle: trackers -> types, never the reverse).

class TargetTracker(Protocol):
    name: str
    needs_track_ids: bool         # True ⇒ Detector must run track mode (§6.3)
    tracker_yaml: str | None      # ultralytics tracker config name, DNN family only
    def lock(self, frame: Frame, dets: list[Detection], *,
             seed_xy: tuple[float, float] | None = None,
             seed_index: int | None = None) -> AssociationHit | None:
        """DNN impls MUST return None when the seed detection's tid is None
        (stale/pre-switch frame — the lock-time race, Fable-B2)."""
    def update(self, frame: Frame, dets: list[Detection]) -> AssociationHit | None
    def mask(self) -> bytes | None: ...
    def reset(self) -> None: ...

# vision/trackers/__init__.py — lazy-importing registry
def create_tracker(name: str, device: str = "cpu") -> TargetTracker   # unknown name → ValueError (validated at assembly)
def available_trackers(backend: DetectorBackend) -> list[str]
    # entries whose extras are installed AND backend.supports_track covers
    # their needs_track_ids — computed WITHOUT importing optional modules
```

| Name | Family | Extra | needs_track_ids |
|---|---|---|---|
| `botsort`/`bytetrack`/`ocsort`/`deepocsort`/`tracktrack`/`fasttrack` | DNN association (ultralytics built-ins) | `perception-dnn` | True |
| `csrt`/`kcf`/`mosse` | OpenCV template (class-less lock, any point) | `perception-cv` | False |
| `sam2` | mask-memory (ultralytics `SAM2DynamicInteractivePredictor`) | `perception-sam` | False |

**Ownership and threads (Fable-M4, Codex-B1):** the tracker object lives on the
**Detector thread** (never the asyncio loop — CSRT is tens of ms, SAM2 seconds
on CPU). `VisionContacts.designate()` resolves the contact NAME → a seed
(last image position or detection index) and posts a lock request to the
pipeline's designation slot (mirroring the lab's `LockRequests`); the Detector
thread creates/locks the tracker against the NEXT inference of the current
tracking generation and carries the `AssociationHit` (and RLE mask) inside
`InferenceResult.designated_hit`. VisionContacts consumes hits, never the
tracker.

**Deterministic consumption order (Codex-M6):** (1) designated association
first; (2) its `detection_index` is RESERVED — removed from the multi-contact
candidate pool (no double-consumption, no ID switch); (3) the world gate
associates the remaining detections to other contacts; (4) on a designated
miss the contact COASTs on the EKF — NO silent fallback to the world gate
(fallback is what manufactures ID switches); (5) reacquisition only through
the explicit gated re-confirmation of §6.5.

**Hit semantics (Fable-M3):** `aim_px` is the footpoint (mask/box
bottom-center) when the impl can derive one, else the patch centroid — the
impl documents which, and VisionContacts projects with the matching projection
variant (center-projected hits are marked `position_src` lower-confidence and
used as `update_bearing` unless a footpoint exists). `ContactView.conf` always
holds the last DETECTION confidence; tracker `conf` (0.0 from template/SAM
impls) is display-only. Tracker-provided classes (`"template"`/`"sam2"`) never
rename the contact — designation preserves name/class.

**Lifecycle (`vision/follow.py`, adapted — not vendored whole, Fable-M2 /
Codex-M5):** the lab's four-state skeleton (IDLE/TRACKING/COAST/LOST) with:
sim time injected (never `time.time()`), HFOV from `projection.HFOV_DEG` (69°,
not the lab's 75° webcam default), deadlines derived from `TrackerConfig`
(`coast_s`/`lost_s` ÷ `dt_nominal_s` — ONE owner of the lost constants), hits
passed to the EKF RAW (the lab's 0.5-EMA output is display/pointing only —
pre-filtered measurements would corrupt the EKF's covariance contract), and
LOST persistent until the contact owner drops/rebinds (the lab's auto-expire
to IDLE is not mapped). TRACKING→MEASURED, COAST→COASTING, LOST→LOST.

---

## 7. `agents/pilot` — the single agent [REWRITE]

### 7.1 `pilot/agent.py`

```python
class PilotAgent:
    def __init__(self, system: "mavsdk.System", ops: FlightOps,
                 pipeline: VisionPipeline, bridge: RosBridge,
                 env: dict | None = None, model: str | None = None,
                 cli_path: str | None = None) -> None
    async def connect(self) -> None        # MAVSDK connect + GF_* from Envelope
    async def run(self) -> None
```

- THE one `System` (`mavsdk.System(mavsdk_server_address="127.0.0.1",
  port=50051)`) and THE one `FlightOps` are built by the assembler (§7.2) and
  injected here AND into `make_pilot_options` — estop and tools share both
  (Fable-B1/Codex-B3).
- Inbox `TopicLog("/pilot/user_input", CMD_QOS)`, cursor at log end on start.
- Degraded boot: `pipeline`'s detector failure ⇒ health published; flight tools
  fully available; `detect` (when registered) returns `NOT_READY`.
- **Estop arbiter** (Codex-B3): an `ActiveToolRegistry` records the currently
  running tool task. On `/pilot/estop` (CMD_QOS): cancel the tool task (NOT the
  agent turn — Fable-MAJOR-1 option A: the turn survives to receive the code),
  await its cleanup with `asyncio.shield`, then `ops.emergency_hold()` (or
  `emergency_land()`), all under an operation generation counter so a cancelled
  controller can never resume streaming stale setpoints. The tool wrapper
  catches `CancelledError` and returns `ESTOPPED: operator halted <tool>` into
  the live turn. Confirmation to `/pilot/chat`. wait=false overlaps are
  registered the same way (a later estop cancels them too).

### 7.2 `pilot/run.py` — assembly order (Fable-MAJOR-4/Codex-B10 corrected)

```
bridge = RosBridge()
world = World()
recorder = Px4StateRecorder(bridge, sink=world, i=0)          # W1 feeder
cameras = GzCameras(1)
rangefinder = GzRangeProvider(RANGE_TOPIC.format(...)) | None # M2+
detector = Detector(cameras, backend) | None                  # M2+
contacts = VisionContacts(world, rangefinder)                 # M3a; M1/M2: GzPoses(evals)
pipeline = VisionPipeline(detector, contacts)
system = mavsdk.System(mavsdk_server_address="127.0.0.1", port=50051)
envelope = Envelope(center_e=spawn_e, center_n=spawn_n)
ops = FlightOps(system, world, bridge, 0, 1, contacts, envelope)
agent = PilotAgent(system, ops, pipeline, bridge,
                   env=agent_env("pilot"), model=..., cli_path=...)
options = make_pilot_options(ops, detect_text=pipeline_detect_text_or_None,
                             report=agent.report, env=..., model=..., cli_path=...)
bridge.start(); recorder.start(); await agent.connect()
await gather(agent.run(), pipeline.run(), estop_task)
```

`acquire_singleton_lock()` at entry (as `agents/swarm/run.py:73` today).
`agent_env()` per design §5.2 (`cli_path=shutil.which("claude")` REQUIRED on the
Kimi tier, R5).

---

## 8. `agents/observatory` — cockpit [REWRITE-lite]

### 8.1 Topics

| Topic | Type | QoS | Producer → consumer |
|---|---|---|---|
| `/px4_0/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4_QOS | uXRCE → Px4StateRecorder |
| `/px4_0/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | PX4_QOS | uXRCE → Px4StateRecorder |
| `/px4_0/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | PX4_QOS | uXRCE → cockpit `/state` (flight mode) |
| `/px4_0/fmu/out/battery_status` | `px4_msgs/BatteryStatus` | PX4_QOS | uXRCE → cockpit `/state` |
| `/pilot/user_input` | `std_msgs/String` | CMD_QOS | cockpit → pilot |
| `/pilot/estop` | `std_msgs/String` | CMD_QOS | cockpit → estop arbiter |
| `/pilot/chat` | `std_msgs/String` | CHAT_QOS | pilot/cockpit → feed |
| `/pilot/detections` | `std_msgs/String` (JSON, PerceptionSnapshot v1) | **STATE_QOS** | pipeline → cockpit / evals |

### 8.2 Server endpoints

```python
class CockpitServer:
    def __init__(self, bridge: RosBridge, cameras: GzCameras) -> None
    async def run(self, host: str = "0.0.0.0", port: int = 8000) -> None
# GET  /state          -> pose, attitude, flight mode, battery, detector/beam health
# WS   /ws_cam         -> per-frame: text {"seq": int, "sim_stamp": float,
#                         "codec": "avc1..."} announce once, then binary
#                         [seq:u32, stamp:f64, ...H.264 AU] per access unit
# WS   /ws_detections  -> /pilot/detections relay (verbatim)
# POST /command {text} -> /pilot/user_input
# POST /estop {action} -> /pilot/estop ("hold"|"land")
# GET  /chat?since=n   -> TopicLog lines
```

### 8.3 `video.py` [PORT+ — duck-type corrected]

`VideoHub` consumes **`snapshot()` exclusively** (not `seq()`+`raw()` — v1's
"unchanged pump" was wrong): one `Frame` per encode, and every emitted access
unit carries the frame's `seq`+`sim_stamp` (§8.2 framing) so the overlay can
match exactly.

### 8.4 Static UI

Per design §3.7 (cockpit POV + HUD table, 0.5 s staleness guard, degraded
banners), fed by §8.2's stamped video + verbatim snapshots.

---

## 9. Error taxonomy (v2 — producible paths for every code)

Typed failures in `flight/errors.py`:

```python
class ToolFailure(Exception):
    def __init__(self, code: str, text: str) -> None: ...
class InvalidParamError(ToolFailure): ...   # code INVALID_PARAM
class NotReadyError(ToolFailure): ...       # code NOT_READY
class BlockedError(ToolFailure): ...        # code BLOCKED
# EnvelopeViolation(InvalidParamError) — §5.2
```

| Code | Produced by | is_error |
|---|---|---|
| `INVALID_PARAM` | `InvalidParamError` / `EnvelopeViolation` | True |
| `NOT_READY` | `NotReadyError` (no fix, no contacts, detector down, bearing-only goto) | True |
| `BLOCKED` | `BlockedError` (`_await_arrival` timeout — it RAISES now; offboard refused; beam-lock retries exhausted) | True |
| `LOST` | track O2 — a RETURN value, degraded completion | False |
| `TIMEOUT` | `asyncio.TimeoutError` (run_mission) | True |
| `ESTOPPED` | wrapper catching `asyncio.CancelledError` (BaseException — caught explicitly) after the arbiter cancels the tool task | True |
| `INTERNAL` | any other `Exception` (logged with traceback server-side) | True |

Exact SDK result dicts: success = `{"content": [{"type": "text", "text": str}]}`;
failure = same + `"is_error": True`; LOST uses the success shape (degraded).
`run_mission` keeps its `(bool, str)` return (its internal protocol) and the
wrapper maps `True` to the fitting code (`TIMEOUT` or `INTERNAL`).

---

## 10. `evals` — harness port [PORT-lite + NEW]

```python
@dataclass
class Deps:
    world: World
    bridge: RosBridge
    cameras: GzCameras
    oracle_truth: GzPoses | None = None          # sampler + oracle ONLY
    flight_contacts: ContactProvider | None = None
    detector: Detector | None = None

# evals/perceive_eval.py — fixtures, NOT a live threaded Detector (Codex note):
def accuracy_report(frames: list[Frame], truths: list[dict],
                    backend: DetectorBackend) -> dict
    """Runs the BACKEND synchronously over recorded frames joined to truth by
    sim_stamp (tolerance 50 ms): precision/recall per class, contact error
    p50/p95 by range, ID-switch rate, fragmentation. Pure + offline."""

def note_target_lock(trace: "Trace", contacts: ContactProvider,
                     truth: GzPoses) -> None
    """Called SYNCHRONOUSLY at Trace.observe time (≤ one mover-tick association
    error, stated): first vis_* target of track/goto -> TargetLockEvent +
    truth association at that moment -> run_meta['target_lock']."""
```

`VisionContacts.reset()` runs inside `soft_reset` per cell; everything else
(`_drive`, `Trace`, CHECKS, `Sampler`, `report.py`) ports unchanged.

---

## 11. Testing strategy — few, sharp, contract-bound

Philosophy: **every test maps to a contract; anything that can't fail is deleted.**
Not one test per function — one test per *promise the ICD makes*. Three defense
lines: (1) contract tests per submodule (sim-free, fast); (2) ONE architecture
gate (`tests/test_import_rules.py`, the AST scan consuming §0.1's adjacency list
verbatim — any forbidden import fails); (3) reality gates (one sim smoke per
milestone + the oracle eval harness grading camera-fed vs truth-fed).

Efficiency rules: one contract → one test, named after it; no implementation-
detail tests; shared fixture factories (one synthetic-frame builder, one
`FakeContacts`, one recorded-truth dir); duplication pruning (two tests failing
for the same reason ⇒ one dies); tests live at `tests/test_<module>_<contract>.py`;
sim-gated tests marked `@pytest.mark.sim`, run at milestone gates, not in the
fast loop.

### Per-module seams (what proves what)

| Module | Fake/fixture | Covers |
|---|---|---|
| `core/*` | fake gz callbacks; FakeBridge; scripted msgs | stores, Frame atomicity, QoS tables, recorder clock alignment + shortest-angle |
| `world` | dict-only stubs | interpolation edges, resolve |
| `perception` | pure values | all trig incl. horizon-None, erosion, footprint |
| `vision` | `FakeBackend` (scripted Detections), synthetic RGB, `FakeRangeProvider` (scripted samples incl. robust_at windows), TrackerConfig test vectors | seq-dedupe, lifecycle/degrade, EKF birth/rebind/NIS, bearing-only, beam gates, snapshot atomicity |
| `flight` | `FakeContacts` (poses/velocities/health/designation), kinematic `FakeOps` (§13 item 5), envelope fixtures | O1–O6, LOST path, velocity dispatch, face-wait + residual, emergency_* idempotence, envelope reject codes per primitive |
| `pilot` | FakeOps + fake SDK client holding a turn open | loop, **estop arbiter (cancel→cleanup→shielded hold→ESTOPPED into the live turn)**, fusion-tick starvation (turn held open while pipeline keeps publishing), degraded boot |
| `observatory` | recorded snapshots + frames | overlay match/staleness, endpoints, WS framing |
| `evals` | recorded WorldTrack + frames + truth | accuracy_report, target-lock, §4.3 envelope render test |
| architecture | AST scan of `agents/**` | §0.1 adjacency list verbatim — the anti-spaghetti gate |

### Per-milestone load-bearing set (the whole suite, no more)

- **M0:** (spike scripts, not pytest) the S0 asserts of design §5.6.
- **M1 (7):** import-rules scan · exposed tool list == 12 (+ full JSON schemas) ·
  envelope rejects out-of-range goto/speed/orbit-perimeter, legible code, no
  motion · error mapping ValueError→INVALID_PARAM, CancelledError→ESTOPPED ·
  estop arbiter sequence (cancel→cleanup→shielded hold, generation counter
  blocks stale setpoints) · §4.3 envelope render (exact text) · doctor FAIL ⇒
  launch refused.
- **M2 (8):** Frame snapshot atomicity under hammering · projection (attitude
  rotation, support-plane None, footpoint) · detector seq-dedupe + degrade →
  NOT_READY · `VisionConfig` fail-closed explicit vs auto fallback · extraction
  branches (boxes/mask/obb) + `frame_to_array` channel order · rangefinder shim
  (edge-mix flag, dropouts, robust_at lag bound) · raw-snapshot JSON schema
  (contacts empty) · sim: airborne `detect` bearing through a 12 m/s transit.
- **M2.5 (2):** manifest SHA-256 verification (mismatch ⇒ BackendError) ·
  accuracy gate vs blob baseline on fixtures.
- **M3a (7):** EKF birth/rebind/NIS-gate/bearing-only · contacts-goes-silent ⇒
  structured LOST, offboard stopped · velocity dispatch (feed_direct vs
  finite-diff) · face-wait residual on timeout · **pipeline starvation** (turn
  held open, pipeline still publishes) · `scan` renders `alt unk` for
  bearing-only · sim: d2_shadow camera-fed 2/2 + truth-fed control 2/2.
- **M3b (5):** beam association gates (inside / outside / two masks / consistency
  reject) · acquisition SM transitions (bearing-only → ACQUIRING → lock →
  track; beam-slip → COASTING, no LOST-cycle) · envelope gating (no fusion at
  chase pitch) · deterministic consumption order (designated reserved, no
  double-consume, no fallback) · sim: airborne acquisition + in-envelope slant
  error <0.5 m p50.
- **M4 (3):** overlay sim-stamp match + >0.5 s staleness drop · degraded banner
  on killed detector · estop button holds mid-track.
- **M5 (4):** `accuracy_report` fixtures (precision/recall + ID-switch) ·
  TargetLockEvent → `identified_target` oracle path · per-cell
  `VisionContacts.reset()` (no leak across anchored repeats) · strategy A/B
  infra (activation only on measured lift).
- **M6 (3):** seam emits typed events (Text/ToolCall/ToolResult/Result) ·
  `cli_path` honored on the Kimi tier · spike: in-sim text-only tool chain on
  `kimi`.

That is **41 tests + 3 spike scripts** for the entire rebuild — each one
capable of failing, each one named after a contract in this ICD.

---

## 12. Design-spec notes (ICD v2 overrides flagged for the design's next rev)

- Design §2.3 mermaid still says "14 tools" → 13 (v4.2); M1 says 12 (§0.6).
- Design §3.6 estop says "cancels the in-flight agent turn + active tool task" —
  superseded by §7.1's arbiter (tool task only; the turn receives ESTOPPED).
- Design M1 "13 tools minus detect" / gate "== the 13 MCP tools" → **12** (the
  current 13 minus `look`; `detect` arrives M2).
- Design §3.1 camera topic is parameterized by model name (M2's
  `x500_depth_range` changes it) — `CAM_MODEL` env, §2.3.

---

## 13. Review changelog (v1 → v2)

Raw reviews: [Fable](reviews/2026-07-19-icd-fable-high-review.md) ·
[GPT-5.6-sol](reviews/2026-07-19-icd-codex-gpt56sol-high-review.md).
Verdicts: Fable GO (conditioned on B1+MAJOR-1) / Codex NO-GO (5 blockers) —
all resolved as follows.

### Blockers (both reviews) — resolved

| Finding | v2 fix |
|---|---|
| FlightOps buried in factory closures; estop can't reach it; envelope/rangefinder unwired (Fable-B1, Codex-B3) | Construction inverted: assembler builds THE ONE System/FlightOps/Envelope, injects into PilotAgent AND make_pilot_options; `emergency_hold/land` public; ActiveToolRegistry arbiter with generation counter + shielded cleanup (§5.5, §7) |
| Fusion tick starves during a 120 s track (Fable-B2) | `VisionPipeline` — an explicit independent asyncio task, never awaited inside turns; starvation test in §11 (§6.7, §0.2) |
| M1 contradicts ICD (12 vs 13 tools; future classes required) (Codex-B1) | §0.6 milestone compatibility table; capability-aware constructors; M1 = 12 tools, `detect` not stubbed |
| ContactView in flight but constructed by vision (Codex-B2) | DTOs moved to `agents/core/contact.py` (§1, §2.6) |
| Error taxonomy has unproducible codes (Fable-MAJOR-1, Codex-B9) | `flight/errors.py` typed failures; `_await_arrival` raises `BlockedError`; CancelledError→ESTOPPED into the live turn (arbiter cancels tool task, not the turn); INTERNAL added; exact result dicts (§9, §5.5) |
| Acquisition SM two owners; states not on the wire (Fable-MAJOR-2, Codex-B4/B6) | TrackSession (ops) owns mission SM; VisionContacts owns measurement health only; PerceptionSnapshot carries `track.state` + `beam.status` (§1, §5.4-O6, §6.7) |
| ContactProvider can't express bearing-only / no designate / two ToF owners (Codex-B4) | `ContactView` optional coords + bearing/elevation + position_src; `TargetDesignator` protocol; VisionContacts = SOLE rangefinder owner (FlightOps' param removed); TrackingContext (§5.1, §6.4) |
| robust() can't do the frame↔sample join (Codex-B5) | `robust_at(sim_stamp, sync_tolerance_s=0.05)` (§2.5) |
| W1 feeder unowned; clock/quat conversions homeless (Fable-MAJOR-3, Codex-B10) | `core/telemetry.py Px4StateRecorder` (§2.7) |
| Detector passed into flight breaks the seam (Fable-MAJOR-6) | `detect_text` closure composed in pilot; flight never sees Detector (§5.5) |
| BeamAssociator signature unimplementable (Fable-MAJOR-7, Codex-B8) | full signature + `BeamAssociation` result schema (§6.6) |
| Rangefinder missing from assembly; CAM_TOPIC breaks on composite model (Fable-MAJOR-4) | §7.2 order + `RANGE_TOPIC` + `CAM_MODEL` parameterization (§2.3, §2.5) |
| DetectionView undefined; detect grammar unspecified (Fable-MAJOR-5, Codex-B6) | verbatim grammar in §5.5; blob pins `cls="target"` |
| CV-EKF/acquisition "requirements not contracts" (Codex-B7) | `TrackerConfig` numeric test vectors + `CvEkf` measurement models + transition rules (§6.5) |
| core "sim-free" claim false for bus/camera (Codex-B11) | claim corrected (§2 header); optional lazy-import refactor noted, not mandated |
| Detector lifecycle/freshness (Codex-B12) | `InferenceResult`, `wait_next(after_seq)`, lifecycle states, `stop(timeout)` joins (§6.3) |
| Envelope semantics incomplete (Fable-minor-9, Codex-B13) | center=launch/home, per-primitive checks incl. orbit perimeter + fly endpoint, GF_* derived from Envelope at connect, run_mission admission stated (§5.2) |

### Majors/minors folded in

`/pilot/detections` on new `STATE_QOS` (depth-1 latched) · `vehicle_status`
topic added · VideoHub consumes `snapshot()` exclusively + per-access-unit
stamps + exact WS framing (§8.2/§8.3) · `jpeg_b64` deletion declared (test
migration noted) · `lost_s` single owner (VisionContacts; timer is fallback
only) · `face` 5°/5 s kept as CAMERA criterion + 3-sample hold + residual on
timeout; ToF beam-lock is the separate tighter acquisition criterion · import
matrix made normative adjacency list; observatory third-party deps moved to
requirements pinning · `accuracy_report` made fixture-based (backend over
recorded frames, not a live Detector) · `note_target_lock` synchronous with
the ≤1-tick caveat stated · `hover` 120 s cap documented · RosBridge node
name kept verbatim (`swarm_bridge`) · design-spec staleness notes in §12.

### Open points (ICD v1 §12) — resolved per both reviewers

1. Single snapshot topic, kept (versioned `PerceptionSnapshot`, STATE_QOS
   depth-1; debug split later if bandwidth demands).
2. Bearing-only coords: `e/n/z = None` + `position_src="none"` for newborns;
   `predicted` for previously-locked (Codex's refinement, subsuming Fable's
   "held float" while keeping `poses()` numeric-only for control consumers).
3. Envelope keeps geofence values; PX4 `GF_*` derived from it at connect;
   center = launch/home.
4. Fusion ticking: `VisionPipeline` in `vision/` owns it (Detector stays pure;
   pilot starts/stops + relays) — the reviewers' preferred home over raw pilot
   ownership.
5. `face` 5°/5 s provisional for camera only; re-measure timeout on the M1
   bench before M3b freezes acquisition timing on top.

---

## 14. v3/v3.1 — perception-lab integration (2026-07-20, owner-directed; v3.1 after dual review)

`~/perception-lab` @ `26e9431a193f4cf4f051d086d23ac0133dd305a6` (a sibling R&D
sandbox: webcam + model/tracker comparison web UI) donates its perception
machinery — **not** its web UI. The relationship is a **redesign derived from
the lab, not a verbatim port** (v3.1, per Codex-M11; adaptation table below).
v3 raw reviews: [Fable](reviews/2026-07-20-perception-integration-fable-high-review.md)
(NO-GO) · [GPT-5.6-sol](reviews/2026-07-20-perception-integration-codex-gpt56sol-high-review.md)
(NO-GO) — all findings resolved in the v3.1 sections above.

| Lab module | Becomes | Adaptation |
|---|---|---|
| `models.py` (`YoloRunner`) | `vision/backends.py UltralyticsBackend` (§6.2) | single-model backend (no multi-model cache); extraction branches added (masks, `res.obb`) — the donor reads `res.boxes` only; `conf=conf` passed explicitly; `reset_tracking()` added (donor's `reset()`); `TRACK_CONF=0.1` in track mode with Detector conf as post-birth filter |
| `trackers/base.py` (`TargetTracker`) | `vision/trackers/base.py` (§6.8) | tuple hit → first-class `AssociationHit` (index, box, footpoint `aim_px`, conf, tid, RLE mask); lock takes `seed_xy`/`seed_index` (contact-name resolution lives in VisionContacts, not the tracker); `tid` `int\|None`, −1 normalized; DNN lock must reject `tid=None` seeds |
| `trackers/__init__.py` (registry) | `vision/trackers/__init__.py` (§6.8) | lazy-importing factories; `available_trackers(backend)` intersects extras with `supports_track`; default is `none` (donor's default is `botsort`; `iou-gate` REMOVED from the registry — the world gate is not an image tracker) |
| `trackers/dnn.py`, `template.py`, `sam.py` | optional registry entries (§6.8) | `perception-dnn` / `perception-cv` / `perception-sam` extras (sam extra corrected to match the donor's ultralytics-SAM2 + cv2 imports) |
| `follow.py` (`FollowTarget`) | `vision/follow.py` — ADAPTED skeleton (§6.8) | sim time injected (never wall clock); HFOV 69°; deadlines from `TrackerConfig` (one owner of lost constants); EMA output is display-only — hits reach the EKF raw; LOST persistent (no auto-IDLE) |
| `inference.py` (the orchestrator) | absorbed by `Detector` thread + `VisionPipeline` (§6.3, §6.7) | mode switch → `configure_tracking()` always-on generation; `LockRequests` → `request_lock()` designation slot; `runner.reset()` → `reset_tracking()`; tracker hot-swap → assembly-time `VisionConfig`, never mid-flight |
| `state.py` (`LockRequests` etc.) | designation slot inside `InferenceResult` flow (§6.3) | no shared-state module is taken |
| `bench_models.py` | M2.5 model-selection pattern | fps tables feed backend choice |
| `webui.py`, `static/`, `api.py`, `capture.py`, `overlay.py`, `__main__.py` | **not taken** | the clickable UI, pursuit overlay, and FastAPI/uvicorn deps stay in the lab |

Layering rule (§6.8): **association is OPTIONAL and swappable** (registry
image-space trackers for the ONE designated contact, on the Detector thread);
**fusion is not** (world-space CV-EKF; every contact defaults to the built-in
world gate). Baseline runs with `tracker=none` and no extras.

Immediate value: visdrone aerial weights give real detection classes (person,
car, van, truck — UAV-footage fine-tunes) at M2, before the custom mover model
of M2.5 exists (OBB-DOTA works only through the added `res.obb` branch);
template/SAM2 trackers make designated pursuit robust through detector
dropouts; SAM2-as-tracker is the concrete shape of the deferred segmentation
trigger (design §6.4 — noted there as tracker-scoped, not full-image SAM).

### v3.1 triage (both reviews)

Fable: B1 registry→Optional+default none ✓ · B2 `configure_tracking` always-on
generation + `request_lock` + tid-None lock guard ✓ · M1 sam extra = dnn+cv ✓ ·
M2 follow.py adapted not vendored ✓ · M3 `AssociationHit` footpoint `aim_px` +
conf provenance ✓ · M4 trackers on Detector thread, hit in `InferenceResult` ✓ ·
M5 `reset_tracking()` ✓ · M6 `supports_track` + availability intersect ✓ ·
M7 `TRACK_CONF=0.1` + post-birth filter ✓ · m1 `tid int|None` ✓ · m2 name→seed
in VisionContacts ✓ · m3 `inference.py` mapped ✓ · m4 obb branch ✓ · m5
`frame_to_array` ✓ · m6 mask RLE carried in hit ✓ · m7 `VisionConfig` validated
at assembly ✓ · m8 design §6.2/§6.4 reconciled ✓ · n1–n4 wording ✓.

Codex: B1 tracking-mode generation + assembly validation ✓ · B2 world gate out
of the registry ✓ · B3 `AssociationHit` DTO + beam consumes it ✓ · B4
`VisionPipeline(contacts=None)` raw snapshots at M2 ✓ · M5 follow.py reconciled
✓ · M6 deterministic consumption order + no-fallback coast ✓ · M7 extras fixed +
lazy registry ✓ · M8 `VisionConfig` + fail-closed explicit ✓ · M9 extraction
branches + conf ✓ · M10 tid normalization ✓ · M11 derivation framing + commit
pin ✓ · M12 adapters dormant at M2, contract tests added to the gate ✓ ·
minors: "baseline deps" phrasing ✓, ByteTrack rejection qualified in design §6.2 ✓.
