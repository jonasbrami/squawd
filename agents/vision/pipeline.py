"""vision/pipeline.py — detector → snapshot ticking (ICD §6.7).

Owns the detector→(contacts) ticking and the atomic PerceptionSnapshot.
Detector stays a pure inference thread; PilotAgent starts/stops the pipeline
and relays snapshots. At M2 (pre-M3a) `contacts` is None: the pipeline
publishes RAW-detection snapshots (empty contacts array, beam/track IDLE) so
/pilot/detections and the overlay work before fusion exists (Codex-B4).
"""
import asyncio
import base64
import json
from dataclasses import dataclass, field

from agents.vision.detector import Detector
from agents.vision.types import InferenceResult  # noqa: F401


@dataclass(frozen=True)
class PerceptionSnapshot:
    """The ONE atomic wire/authority object (schema v1, ICD §1)."""
    schema_version: int
    frame_seq: int
    sim_stamp: float
    frame_w: int
    frame_h: int
    completed_monotonic: float
    dets: list                  # Detection
    contacts: list              # ContactView (empty at M2)
    detector: dict              # {"healthy": bool, "latency_ms": float}
    beam: dict = field(default_factory=lambda: {
        "status": "IDLE", "target": None, "range_m": None})
    track: dict = field(default_factory=lambda: {
        "state": "IDLE", "target": None, "gap_m": None})

    def to_json(self) -> str:
        def det_json(d):
            j = {"cls": d.cls, "conf": round(d.conf, 2),
                 "xyxy": [round(v, 1) for v in d.xyxy]}
            if d.mask is not None:
                # W2 (design §4): the box-region RLE mask rides the wire for
                # UI drawing — base64 varints + the decode dims, computed with
                # the encoder's own box formula. Omitted when the backend has
                # no mask (the pre-W2 wire shape, byte-compatible).
                j["mask"] = {
                    "rle": base64.b64encode(d.mask).decode("ascii"),
                    "w": max(1, int(d.xyxy[2]) - int(d.xyxy[0])),
                    "h": max(1, int(d.xyxy[3]) - int(d.xyxy[1]))}
            return j
        return json.dumps({
            "schema_version": self.schema_version,
            "sim_stamp": round(self.sim_stamp, 2),
            "seq": self.frame_seq,
            "frame_w": self.frame_w,
            "frame_h": self.frame_h,
            "dets": [det_json(d) for d in self.dets],
            "contacts": self.contacts,
            "detector": {"healthy": self.detector["healthy"],
                         "latency_ms": round(self.detector["latency_ms"], 1)},
            "beam": self.beam,
            "track": self.track,
        })


class VisionPipeline:
    """Independent asyncio task: wait_next → (contacts.update at M3a) →
    assemble ONE PerceptionSnapshot → publish (STATE_QOS)."""

    def __init__(self, detector: Detector | None, contacts=None,
                 bridge=None) -> None:
        self._detector = detector
        self._contacts = contacts
        self._bridge = bridge
        self._latest: PerceptionSnapshot | None = None
        self._task: asyncio.Task | None = None
        self._stop = False

    def latest(self) -> PerceptionSnapshot | None:
        return self._latest

    def start(self) -> None:
        self._task = asyncio.create_task(self.run())

    def stop(self) -> None:
        self._stop = True
        if self._task is not None:
            self._task.cancel()

    async def run(self) -> None:
        last_seq = 0
        while not self._stop:
            if self._detector is None:
                await asyncio.sleep(0.5)
                continue
            # wait_next BLOCKS on a threading.Condition up to `timeout` — it
            # must run off-loop, or every quiet 0.5 s stalls the whole agent
            # (the Fable-B2 starvation the pipeline-starvation test guards).
            res = await asyncio.to_thread(self._detector.wait_next,
                                          after_seq=last_seq, timeout=0.5)
            if res is None:
                await asyncio.sleep(0.05)
                continue
            last_seq = res.frame.seq
            if self._contacts is not None:
                self._contacts.update(res)
                views = [v.__dict__ for v in self._contacts.all_views()]
                beam = self._contacts.beam_view()
                track = self._contacts.track_view()
            else:
                views = []
                beam = {"status": "IDLE", "target": None, "range_m": None}
                track = {"state": "IDLE", "target": None, "gap_m": None}
            self._latest = PerceptionSnapshot(
                schema_version=1,
                frame_seq=res.frame.seq,
                sim_stamp=res.frame.sim_stamp,
                frame_w=res.frame.width,
                frame_h=res.frame.height,
                completed_monotonic=res.completed_monotonic,
                dets=list(res.detections),
                contacts=views,
                detector={"healthy": self._detector.healthy(),
                          "latency_ms": self._detector.latency_ms()},
                beam=beam,
                track=track,
            )
            if self._bridge is not None:
                self._publish(self._latest.to_json())

    def _publish(self, text: str) -> None:
        from std_msgs.msg import String            # lazy: ROS at runtime
        from agents.core.bus import STATE_QOS
        m = String()
        m.data = text
        self._bridge.publish("/pilot/detections", String, m, STATE_QOS)
