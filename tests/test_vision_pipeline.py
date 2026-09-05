"""VisionPipeline raw-snapshot contract (Codex-B4): atomic schema-v1
snapshots with empty contacts at M2, beam/track IDLE, detector health fields,
JSON wire shape."""
import asyncio
import base64
import json

from agents.core.contact import Frame
from agents.vision.pipeline import PerceptionSnapshot, VisionPipeline
from agents.vision.types import (Detection, InferenceResult, rle_decode,
                                 rle_encode)


class FakeDetector:
    def __init__(self):
        self._n = 0

    def wait_next(self, after_seq, timeout):
        self._n += 1
        if self._n == 1:
            return InferenceResult(Frame(7, 42.5, 64, 48, bytes(64 * 48 * 3)),
                                   [Detection("target", 0.9, (1, 2, 3, 4))],
                                   1.0, None)
        return None

    def healthy(self):
        return True

    def latency_ms(self):
        return 12.34


def test_raw_snapshot_schema_and_json():
    pipe = VisionPipeline(FakeDetector(), contacts=None, bridge=None)

    async def main():
        task = asyncio.create_task(pipe.run())
        for _ in range(100):
            if pipe.latest() is not None:
                break
            await asyncio.sleep(0.01)
        pipe.stop()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(main())
    snap = pipe.latest()
    assert snap is not None
    assert snap.schema_version == 1 and snap.frame_seq == 7
    assert snap.sim_stamp == 42.5
    assert snap.contacts == []
    assert snap.beam["status"] == "IDLE" and snap.track["state"] == "IDLE"
    wire = json.loads(snap.to_json())
    assert wire["schema_version"] == 1 and wire["seq"] == 7
    assert wire["dets"] == [{"cls": "target", "conf": 0.9,
                             "xyxy": [1.0, 2.0, 3.0, 4.0]}]
    assert wire["contacts"] == []
    assert wire["detector"] == {"healthy": True, "latency_ms": 12.3}
    assert wire["beam"]["status"] == "IDLE" and wire["track"]["state"] == "IDLE"


def test_pipeline_idles_without_detector():
    pipe = VisionPipeline(None, contacts=None, bridge=None)

    async def main():
        task = asyncio.create_task(pipe.run())
        await asyncio.sleep(0.1)
        pipe.stop()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(main())
    assert pipe.latest() is None


def test_det_json_carries_mask_when_present():
    """W2 (design §4): a det's box-region RLE mask rides the wire — base64
    varints + decode dims — and round-trips. Maskless dets keep the pre-W2
    shape (pinned by the schema test above)."""
    rows = [[True, False, True], [False, True, False]]
    mask = rle_encode(rows)
    snap = PerceptionSnapshot(
        schema_version=1, frame_seq=1, sim_stamp=2.0, frame_w=64, frame_h=48,
        completed_monotonic=0.0,
        dets=[Detection("car", 0.7, (1.0, 2.0, 4.0, 4.0), mask)],
        contacts=[], detector={"healthy": True, "latency_ms": 1.0})
    wire = json.loads(snap.to_json())
    m = wire["dets"][0]["mask"]
    assert m["w"] == 3 and m["h"] == 2       # the encoder's box formula
    assert rle_decode(base64.b64decode(m["rle"]), m["w"], m["h"]) == rows
