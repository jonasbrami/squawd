"""Single-drone pilot entrypoint (ICD §7.2 assembly).

Assembly: bridge → world → cameras → recorder → perception(detector+pipeline)
→ system → envelope → ops → agent → bridge.start() → connect() → run().
At M2 the camera side is wired: GzCameras → Detector (VisionConfig-selected
backend, blob default) → VisionPipeline (raw snapshots, contacts=None) →
/pilot/detections + the `detect` tool; Px4StateRecorder feeds the W1 buffers
(pose/attitude at any sim-time) for the projection path.
"""
import asyncio
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
                UltralyticsBackend.supports_track):
            backend = UltralyticsBackend(cfg.weights_dir, cfg.model,
                                         cfg.device, cfg.half)
        elif cfg.backend == "onnx" or (cfg.backend == "auto" and cfg.model):
            base = cfg.weights_dir.rstrip("/") + "/" + cfg.model
            backend = OnnxBackend(base, base.rsplit(".", 1)[0] + ".json")
        else:
            backend = ColorBlobBackend()
        detector = Detector(cameras, backend, i=0, hz=10.0, conf=0.25)
        # conf 0.25 for production tracking: the blob's far-range scores
        # (0.35–0.45) would starve the CV-EKF at the 0.45 default; measurement
        # quality lives in the contacts' NN/NIS gates (design §6.8)
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
        contacts = VisionContacts(world, rangefinder=rangefinder) \
            if world is not None else None
        if contacts is not None:
            contacts.attach_detector(detector)      # designate() lock seam
        return detector, VisionPipeline(detector, contacts=contacts,
                                        bridge=bridge), contacts
    except Exception as e:
        print(f"perception boot failed: {e} — sensing degraded", flush=True)
        return None, None, None


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
    system = System(mavsdk_server_address="127.0.0.1", port=50051)
    envelope = Envelope()
    ops = FlightOps(system, world, bridge, 0, 1, contacts=contacts,
                    envelope=envelope)
    agent = PilotAgent(
        system, ops, bridge,
        env=agent_env("pilot"),
        model=os.environ.get("SQUAWD_MODEL") or None,
        detect_text=(make_detect_text(world, bridge, pipeline)
                     if pipeline is not None else None))
    bridge.start()
    recorder.start()
    await agent.connect()
    if pipeline is not None:
        pipeline.start()
    print("pilot online: drone_0 — waiting for commands on /pilot/user_input.",
          flush=True)
    await agent.run()


if __name__ == "__main__":
    _lock = acquire_singleton_lock()
    asyncio.run(main())
