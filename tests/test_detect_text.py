"""detect grammar (ICD §5.5): header + entries, bearing-only fallback, degraded
NOT_READY, classes filter — exact contract the LLM reads."""
import math
import time

from agents.core.contact import Frame
from agents.pilot.detect_text import make_detect_text
from agents.vision.pipeline import PerceptionSnapshot
from agents.vision.types import Detection


class FakePipeline:
    def __init__(self, snap):
        self._snap = snap

    def latest(self):
        return self._snap


class FakeBridge:
    class _P:
        x = 0.0
        y = 0.0
        z = -12.0
        heading = 0.0
        xy_valid = True

    def latest(self, topic):
        return self._P()


def snap(dets, healthy=True, stamp=10.0):
    return PerceptionSnapshot(
        schema_version=1, frame_seq=412, sim_stamp=stamp, frame_w=640,
        frame_h=360, completed_monotonic=time.monotonic() - 0.2,
        dets=dets, contacts=[],
        detector={"healthy": healthy, "latency_ms": 14.0})


def world_with_pose():
    from agents.world.model import World
    w = World(path="/nonexistent")
    w.note_pose(9.99, 50.0, 20.0, 12.0, 0.0)
    w.note_pose(10.01, 50.0, 20.0, 12.0, 0.0)
    w.note_attitude(9.99, 0.0, 0.0, 0.0)
    w.note_attitude(10.01, 0.0, 0.0, 0.0)
    return w


def test_no_frames_yet_not_ready():
    assert make_detect_text(world_with_pose(), FakeBridge(),
                            FakePipeline(None))(None).startswith(
                                "NOT_READY: nothing detected yet")


def test_degraded_detector_not_ready():
    out = make_detect_text(world_with_pose(), FakeBridge(),
                           FakePipeline(snap([], healthy=False)))(None)
    assert out.startswith("NOT_READY: sensing degraded")


def test_empty_detections_header():
    out = make_detect_text(world_with_pose(), FakeBridge(),
                           FakePipeline(snap([])))(None)
    assert out == "0 detections (frame #412, 0.2s old): nothing detected"


def test_entry_grammar_with_geom_range():
    # box at bottom-center of frame -> steep depression -> geom range ~ alt
    d = Detection("target", 0.91, (300.0, 340.0, 340.0, 360.0))
    out = make_detect_text(world_with_pose(), FakeBridge(),
                           FakePipeline(snap([d])))(None)
    head, _, entry = out.partition(": ")
    assert head.startswith("1 detections (frame #412, 0.2s old)")
    assert entry.startswith("vis_target_0 target conf 0.91")
    assert "ahead" in entry and "~35m geom" in entry   # 12m alt / sin(19.8deg)
    assert "(at E" in entry and "N" in entry


def test_bearing_only_when_above_horizon():
    # box at top of frame (above horizon) -> range unobservable
    d = Detection("box", 0.66, (300.0, 0.0, 340.0, 20.0))
    out = make_detect_text(world_with_pose(), FakeBridge(),
                           FakePipeline(snap([d])))(None)
    assert "(bearing only)" in out


def test_classes_filter():
    d1 = Detection("target", 0.9, (10.0, 300.0, 30.0, 320.0))
    d2 = Detection("box", 0.8, (300.0, 300.0, 320.0, 320.0))
    out = make_detect_text(world_with_pose(), FakeBridge(),
                           FakePipeline(snap([d1, d2])))("box")
    assert "vis_box_" in out and "vis_target_" not in out
    assert out.startswith("1 detections")
