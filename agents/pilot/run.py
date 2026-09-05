"""Single-drone pilot entrypoint (ICD §7.2 assembly).

Assembly: bridge → world → cameras → recorder → perception(detector+pipeline)
→ deep client → deep tools(sidecar) + slowlane(gated) → system → envelope →
ops → agent → bridge.start() → connect() → run().
At M2 the camera side is wired: GzCameras → Detector (VisionConfig-selected
backend, blob default) → VisionPipeline (raw snapshots, contacts=None) →
/pilot/detections + the `detect` tool; Px4StateRecorder feeds the W1 buffers
(pose/attitude at any sim-time) for the projection path. Deep-perception M2:
the host-GPU sidecar's `look`/`pinpoint` tools are env-gated (DEEP_TOKEN /
DEEP_PERCEPTION_URL) and answer UNAVAILABLE when the sidecar is down — the
pilot never blocks on them. M3: the gated slowlane (agents/vision/slowlane.py)
samples the same frame source at ~0.3 Hz and publishes annotations + the
fp_suspect advisory on /pilot/slowlane for the cockpit.
"""
import asyncio
import json
import os

from mavsdk import System

from agents.core.bus import RosBridge
from agents.core.camera import GzCameras
from agents.core.gzposes import GzPoses
from agents.core.singleton import acquire_singleton_lock
from agents.core.telemetry import Px4StateRecorder
from agents.world import World
from agents.flight.backend import agent_env
from agents.flight.envelope import Envelope
from agents.flight.ops import FlightOps
from agents.pilot.agent import PilotAgent
from agents.pilot.detect_text import make_detect_text


def build_perception(bridge, cameras, world=None, sim_clock=None):
    """Detector + VisionContacts + pipeline per VisionConfig (M2/M3a). Any
    backend/config failure -> (None, None, None): the caller boots
    SENSING-DEGRADED (flight tools unaffected), never crashes the pilot."""
    from agents.vision.backends import (ColorBlobBackend, OnnxBackend,
                                        UltralyticsBackend)
    from agents.vision.config import VisionConfig, VisionConfigError
    from agents.vision.contacts import VisionContacts
    from agents.vision.detector import Detector
    from agents.vision.pipeline import VisionPipeline
    try:
        cfg = VisionConfig.from_env()
    except VisionConfigError as e:
        print(f"perception config error: {e} — sensing degraded", flush=True)
        return None, None, None
    try:
        if cfg.backend == "ultralytics" or (
                cfg.backend == "auto" and cfg.model and
                UltralyticsBackend.available):
            backend = UltralyticsBackend(cfg.weights_dir, cfg.model,
                                         cfg.device, cfg.half)
        elif cfg.backend == "onnx" or (cfg.backend == "auto" and cfg.model):
            base = cfg.weights_dir.rstrip("/") + "/" + cfg.model
            backend = OnnxBackend(base, base.rsplit(".", 1)[0] + ".json")
        else:
            backend = ColorBlobBackend()
        detector = Detector(cameras, backend, i=0, hz=10.0, conf=cfg.conf)
        # conf 0.25 default for production tracking: the blob's far-range
        # scores (0.35–0.45) would starve the CV-EKF at the 0.45 default;
        # measurement quality lives in the contacts' NN/NIS gates (design
        # §6.8). VISION_CONF overrides (W2 §4: same 0.25 floor for COCO).
        detector.start()
        rangefinder = None
        if world is not None:
            try:
                from agents.core.rangefinder import (GzRangeProvider,
                                                     RANGE_TOPIC,
                                                     SimImpairment)
                rf = GzRangeProvider(
                    RANGE_TOPIC.format(
                        world=os.environ.get("GZ_WORLD", "dynamic")),
                    impair=SimImpairment())
                rf.connect()
                rangefinder = rf
            except Exception as e:
                print(f"rangefinder boot failed: {e} — ToF degraded",
                      flush=True)
        # W3 codex §1/§2: the shipped coco-* models assemble the tracker with
        # the vehicle superclass keys + 5 s grace (cfg.tracker_config());
        # None keeps the mover's contractual TrackerConfig defaults.
        contacts = VisionContacts(world, rangefinder=rangefinder,
                                  config=cfg.tracker_config(),
                                  admit_classes=cfg.admit_classes) \
            if world is not None else None
        if contacts is not None:
            contacts.attach_detector(detector)      # designate() lock seam
        return detector, VisionPipeline(detector, contacts=contacts,
                                        bridge=bridge), contacts
    except Exception as e:
        print(f"perception boot failed: {e} — sensing degraded", flush=True)
        return None, None, None


def build_deep_client():
    """The M2/M3 shared DeepClient (deep-perception plan §3): env-configured
    (DEEP_PERCEPTION_URL, default http://host.docker.internal:8100;
    DEEP_TOKEN). A missing token or an unreachable sidecar at boot logs ONE
    line; the tools/slowlane still get a client that answers UNAVAILABLE —
    the pilot never blocks on the sidecar (bounded ~2 s health probe only).
    With a token present the real client is kept even when the probe fails,
    so consumers self-heal when the sidecar comes up."""
    from agents.perception.deep_client import DeepClient, DeepResult, UNAVAILABLE

    if os.environ.get("DEEP_TOKEN"):
        client = DeepClient()          # picks up both env vars itself
        try:
            health = client.health()
        except Exception as e:
            health, detail = None, str(e)
        else:
            detail = health.detail
        if health is not None and health.ok:
            models = ",".join(health.data.get("models_loaded", []))
            print(f"deep perception online ({models})", flush=True)
        else:
            print(f"deep perception unreachable at boot ({detail}) — "
                  "look/pinpoint answer UNAVAILABLE until it recovers",
                  flush=True)
        return client

    print("deep perception disabled: DEEP_TOKEN not set — "
          "look/pinpoint answer UNAVAILABLE", flush=True)

    class _OfflineClient:
        """No token configured: every call answers UNAVAILABLE (the tools
        stay bound per plan M2 §3, never raise)."""

        def detect(self, *a, **k):
            return DeepResult(UNAVAILABLE, detail="DEEP_TOKEN not set")

        def segment(self, *a, **k):
            return DeepResult(UNAVAILABLE, detail="DEEP_TOKEN not set")

    return _OfflineClient()


def build_deep_tools(world, bridge, pipeline, cameras, client):
    """M2 deep-perception tools (deep-perception plan §4): look/pinpoint over
    the injected shared client. The pinpoint mask publisher rides the pilot's
    /pilot/deep channel (detections-adjacent, String JSON on STATE_QOS; the
    cockpit frame_seq join lands in M3)."""
    from agents.pilot.deep_tools import make_deep_tools

    def publish_mask(payload):
        from std_msgs.msg import String
        from agents.core.bus import STATE_QOS
        m = String()
        m.data = json.dumps(payload)
        bridge.publish("/pilot/deep", String, m, STATE_QOS)

    return make_deep_tools(world, bridge, pipeline,
                           lambda: cameras.snapshot(0), client,
                           mask_publisher=publish_mask)


def build_slowlane(bridge, cameras, detector, client, armed_ref):
    """M3 slow-lane annotator (deep-perception plan §5): the gated 0.3 Hz
    sidecar sampler. armed_ref is a zero-arg callable returning the live
    armed state (main() feeds it from vehicle_status); the gate also reads
    DEEP_SLOWLANE / RENDER_BACKEND from env at each tick. Its state rides
    /pilot/slowlane (String JSON on STATE_QOS, the /pilot/deep precedent —
    the observatory consumes topics only, ICD §0.1)."""
    from agents.vision.slowlane import SlowLane, gate_decision

    def publish_state(payload):
        from std_msgs.msg import String
        from agents.core.bus import STATE_QOS
        m = String()
        m.data = json.dumps(payload)
        bridge.publish("/pilot/slowlane", String, m, STATE_QOS)

    lane = SlowLane(
        lambda: cameras.snapshot(0), client,
        detector=detector, publisher=publish_state,
        gate=lambda: gate_decision(os.environ.get("DEEP_SLOWLANE"),
                                   os.environ.get("RENDER_BACKEND", "intel"),
                                   bool(armed_ref())))
    return lane


async def main() -> None:
    bridge = RosBridge()
    world = World()
    cameras = GzCameras(1)
    # physics-rate clock for the recorder (M2 lesson: the 10 Hz camera stamp
    # drops attitude_at() off the buffer edge); truth poses are NOT tracked —
    # GzPoses stays out of the flight path (O1 demotion), this is only a clock.
    clock = GzPoses(os.environ.get("GZ_WORLD", "dynamic"), [])
    recorder = Px4StateRecorder(bridge, world, i=0,
                                sim_time_ref=clock.sim_time)
    detector, pipeline, contacts = build_perception(bridge, cameras, world,
                                                    clock.sim_time)
    deep_client = build_deep_client()
    deep_tools = build_deep_tools(world, bridge, pipeline, cameras,
                                  deep_client)
    # M3 slowlane gate input: the live armed state off vehicle_status (the
    # recorder's subscribe-with-callback pattern; arming_state 2 == ARMED,
    # PX4-Autopilot/msg/VehicleStatus.msg).
    flight = {"armed": False}

    def _on_status(m) -> None:
        flight["armed"] = getattr(m, "arming_state", None) == 2

    slowlane = build_slowlane(bridge, cameras, detector, deep_client,
                              lambda: flight["armed"])
    system = System(mavsdk_server_address="127.0.0.1", port=50051)
    envelope = Envelope()
    ops = FlightOps(system, world, bridge, contacts=contacts,
                    envelope=envelope)
    backend = os.environ.get("SQUAWD_BACKEND", "claude")
    agent = PilotAgent(
        system, ops, bridge,
        backend=backend,
        env=agent_env("pilot", backend),
        model=os.environ.get("SQUAWD_MODEL") or None,
        codex_effort=os.environ.get("SQUAWD_CODEX_EFFORT") or None,
        detect_text=(make_detect_text(world, bridge, pipeline)
                     if pipeline is not None else None),
        deep_tools=deep_tools)
    bridge.start()
    recorder.start()
    from px4_msgs.msg import VehicleStatus      # lazy: ROS at runtime
    bridge.subscribe("/px4_0/fmu/out/vehicle_status", VehicleStatus,
                     callback=_on_status)
    await agent.connect()
    if pipeline is not None:
        pipeline.start()
    slowlane.start()
    print("pilot online: drone_0 — waiting for commands on /pilot/user_input.",
          flush=True)
    await agent.run()
    slowlane.stop()


if __name__ == "__main__":
    _lock = acquire_singleton_lock()
    asyncio.run(main())
