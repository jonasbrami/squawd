"""Tracker registry + follow lifecycle (ICD §6.8): lazy availability, unknown
names rejected, FollowTarget deadlines from TrackerConfig, raw hits, health
mapping, no auto-expire of LOST."""
import pytest

from agents.core.contact import Frame
from agents.vision import follow as F
from agents.vision.trackers import available_trackers, create_tracker
from agents.vision.types import Detection


def test_unknown_tracker_rejected():
    with pytest.raises(ValueError, match="unknown tracker"):
        create_tracker("magic")


def test_none_tracker_is_a_first_class_noop():
    """W0.2: `none` (the config default) is in the registry; lock() resolves
    and stores the seed, update() yields to the world-space gate."""
    from agents.vision.trackers.none import NoOpTracker
    t = create_tracker("none")
    assert isinstance(t, NoOpTracker)
    assert "none" in available_trackers()
    frame = Frame(1, 10.0, 64, 48, bytes(64 * 48 * 3))
    dets = [Detection("car", 0.9, (10, 10, 50, 50)),
            Detection("car", 0.8, (100, 100, 150, 150))]
    hit = t.lock(frame, dets, seed_xy=(30.0, 30.0))
    assert hit.detection_index == 0 and hit.xyxy == (10, 10, 50, 50)
    assert t._seed == ((30.0, 30.0), None)
    hit = t.lock(frame, dets, seed_xy=(30.0, 30.0), seed_index=1)
    assert hit.detection_index == 1               # explicit index wins
    assert t.lock(frame, [], seed_xy=(30.0, 30.0)) is None
    assert t.update(frame, dets) is None          # world-space gate takes over
    assert t.mask() is None
    t.reset()
    assert t._seed is None


def test_available_trackers_reflect_installed_extras_only():
    # on this host (no cv2/ultralytics/sam2) nothing is available; the call
    # must not raise regardless
    names = available_trackers()
    assert isinstance(names, list)


def test_follow_lifecycle_deadlines_and_raw_hits():
    f = F.FollowTarget(dt_nominal_s=0.2, coast_s=1.0, lost_s=2.0)
    assert f.coast_frames == 5 and f.lost_frames == 10
    f.lock("target", 100.0, 90.0, conf=0.9)
    assert f.status == F.TRACKING and f.health() == "MEASURED"
    f.step((101.0, 90.0, 0.8), 640, 360)
    assert (f.x, f.y, f.conf) == (101.0, 90.0, 0.8)   # RAW, no EMA smoothing
    for _ in range(6):
        f.step(None, 640, 360)
    assert f.status == F.COAST and f.health() == "COASTING"
    for _ in range(5):
        f.step(None, 640, 360)
    assert f.status == F.LOST and f.health() == "LOST"
    for _ in range(50):
        f.step(None, 640, 360)
    assert f.status == F.LOST                          # persists (no auto-IDLE)


def test_follow_bearing_uses_pinhole():
    import math
    f = F.FollowTarget()
    f.lock("target", 320.0, 180.0)
    ax, ay = f.bearing_elevation(640, 360)
    assert abs(ax) < 1e-9 and abs(ay) < 1e-9
    f.x = 640.0
    ax, _ = f.bearing_elevation(640, 360)
    assert abs(ax - math.radians(34.5)) < 0.02
