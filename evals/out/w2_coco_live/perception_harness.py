"""W2 validation harness (design 2026-07-28 §4, milestone W2).

Boots the PRODUCTION perception assembly (agents.pilot.run.build_perception —
the same function the pilot entrypoint uses) without the LLM agent, which
plays no role in perception: VisionConfig.from_env -> OnnxBackend (sha256
manifest verify + manifest class table) -> Detector -> VisionContacts
(admission allowlist) -> VisionPipeline -> /pilot/detections. The cockpit
observatory server then relays it to /state + /ws_detections.

Run INSIDE the container:
  source /opt/ros/jazzy/setup.bash && source /opt/px4_ws/install/setup.bash
  PYTHONPATH=/workspace uv run --no-project --with onnxruntime \
      python evals/out/w2_coco_live/perception_harness.py
"""
import asyncio
import os

from agents.core.bus import RosBridge
from agents.core.camera import GzCameras
from agents.core.gzposes import GzPoses
from agents.core.telemetry import Px4StateRecorder
from agents.world import World
from agents.pilot.run import build_perception


async def main() -> None:
    bridge = RosBridge()
    world = World()
    cameras = GzCameras(1)
    clock = GzPoses(os.environ.get("GZ_WORLD", "demo"), [])
    recorder = Px4StateRecorder(bridge, world, i=0, sim_time_ref=clock.sim_time)
    detector, pipeline, contacts = build_perception(bridge, cameras, world,
                                                    clock.sim_time)
    if detector is None or pipeline is None:
        raise SystemExit("perception boot failed — see stdout above")
    bridge.start()
    recorder.start()
    pipeline.start()
    print("w2 harness: perception online "
          f"(model={os.environ.get('VISION_MODEL')})", flush=True)
    while True:
        await asyncio.sleep(3600)


if __name__ == "__main__":
    asyncio.run(main())
