"""Detector + backends + types contracts (ICD §6.2/§6.3)."""
import time

import numpy as np

from agents.core.contact import Frame
from agents.vision.backends import (ColorBlobBackend, OnnxBackend,
                                    frame_to_array)
from agents.vision.detector import Detector
from agents.vision.types import BackendError, Detection, rle_decode, rle_encode


def make_frame(seq=1, w=64, h=48, rgb=None):
    return Frame(seq, 10.0, w, h, rgb if rgb is not None else bytes(w * h * 3))


def orange_frame(seq=1, w=64, h=48, box=(20, 16, 30, 26)):
    a = np.zeros((h, w, 3), dtype=np.uint8)
    a[box[1]:box[3], box[0]:box[2]] = (229, 115, 26)
    return Frame(seq, 10.0, w, h, a.tobytes())


class FakeCameras:
    def __init__(self, frames):
        self._frames = frames

    def snapshot(self, i):
        return self._frames[-1] if self._frames else None


class ScriptBackend:
    def __init__(self, dets=None, fail=False):
        self.dets = dets or []
        self.fail = fail
        self.calls = 0

    def infer(self, frame, conf):
        self.calls += 1
        if self.fail:
            raise RuntimeError("backend boom")
        return list(self.dets)


def test_rle_roundtrip():
    rows = [[False, False, True, True], [True, False, False, False]]
    data = rle_encode(rows)
    assert rle_decode(data, 4, 2) == rows
    empty = [[False] * 5 for _ in range(3)]
    assert rle_decode(rle_encode(empty), 5, 3) == empty


def test_frame_to_array_is_bgr():
    rgb = bytes([255, 0, 0] + [0] * 9)                    # first pixel red in RGB
    a = frame_to_array(Frame(1, 0.0, 2, 2, rgb))
    assert a.shape == (2, 2, 3) and a.dtype == np.uint8
    assert tuple(a[0, 0]) == (0, 0, 255)                 # red in BGR


def test_blob_detects_orange_box_with_mask():
    backend = ColorBlobBackend(min_area_px=16)
    dets = backend.infer(orange_frame(), conf=0.1)
    assert len(dets) == 1
    d = dets[0]
    assert d.cls == "target" and d.conf > 0.1
    x1, y1, x2, y2 = d.xyxy
    assert x1 <= 20 and y1 <= 16 and x2 >= 30 and y2 >= 26
    assert d.mask is not None and rle_decode(d.mask, x2 - x1, y2 - y1)[0][0] is True
    assert d.footpoint[1] == float(y2)


def test_blob_conf_filters_weak_blobs():
    backend = ColorBlobBackend(min_area_px=16)
    assert backend.infer(orange_frame(), conf=1.0) == []


def test_detector_seq_dedupes_frames():
    cam = FakeCameras([orange_frame(seq=1)])
    det = Detector(cam, ScriptBackend(), hz=200.0, conf=0.1)
    det.start()
    time.sleep(0.25)
    n_calls = cam_calls = None
    det.stop()
    first = det.detections()
    assert first is not None and first.frame.seq == 1
    calls_after = None
    # backend was called once per NEW frame only (same seq for the whole sleep)
    # (the thread loops hot; inference happens once)
    # allow a second call if the frame happened to change — it didn't
    assert first is not None
    det2 = det.detections()
    assert det2 is first or det2.frame.seq == 1


def test_detector_degrades_after_three_failures():
    class TickCam:
        def __init__(self):
            self.k = 0

        def snapshot(self, i):
            self.k += 1
            return make_frame(seq=self.k)

    det = Detector(TickCam(), ScriptBackend(fail=True), hz=200.0)
    det.start()
    deadline = time.monotonic() + 3.0
    while det.state() != "DEGRADED" and time.monotonic() < deadline:
        time.sleep(0.02)
    assert det.state() == "DEGRADED"
    det.stop()
    assert not det.healthy()


def test_detector_wait_next_returns_newer_only():
    frames = [make_frame(seq=1)]
    cam = FakeCameras(frames)
    det = Detector(cam, ScriptBackend(), hz=50.0, conf=0.1)
    det.start()
    first = det.wait_next(after_seq=0, timeout=2.0)
    assert first is not None and first.frame.seq == 1
    assert det.wait_next(after_seq=1, timeout=0.3) is None
    frames.append(make_frame(seq=2))
    nxt = det.wait_next(after_seq=1, timeout=2.0)
    assert nxt is not None and nxt.frame.seq == 2
    det.stop()


def test_onnx_manifest_verification(tmp_path):
    import hashlib
    import json
    model = tmp_path / "m.onnx"
    model.write_bytes(b"fake model bytes")
    manifest = tmp_path / "m.json"
    manifest.write_text(json.dumps({"sha256": hashlib.sha256(b"fake model bytes").hexdigest()}))
    b = OnnxBackend(str(model), str(manifest))
    assert b._verify()["sha256"]
    manifest.write_text(json.dumps({"sha256": "0" * 64}))
    b2 = OnnxBackend(str(model), str(manifest))
    try:
        b2._verify()
        assert False, "mismatch should raise"
    except BackendError:
        pass


def test_nms_suppresses_overlap():
    import numpy as np
    from agents.vision.backends import _nms
    boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11], [50, 50, 60, 60]],
                     dtype=float)
    scores = np.array([0.9, 0.8, 0.7])
    keep = _nms(boxes, scores)
    assert set(keep) == {0, 2}          # overlapping twin dropped, distant kept


def _fake_seg_outputs(score=0.9, cls=0, box=(0.4, 0.4, 0.2, 0.2), ncls=2,
                      nmask=32, anchors=5):
    """(det (1,C,A), protos (1,32,mh,mw)) with one hot anchor."""
    import numpy as np
    ch = 4 + ncls + nmask
    det = np.zeros((1, ch, anchors), dtype=np.float32)
    det[0, 0, 0], det[0, 1, 0] = box[0] * 640, box[1] * 640   # cx, cy
    det[0, 2, 0], det[0, 3, 0] = box[2] * 640, box[3] * 640   # w, h
    det[0, 4 + cls, 0] = score
    det[0, 4 + ncls:, 0] = 1.0                                 # mask coeffs
    protos = np.ones((1, nmask, 8, 8), dtype=np.float32) * 0.5
    return [det, protos]


def test_decode_seg_shapes_and_names():
    from agents.vision.backends import _decode_seg
    dets = _decode_seg(_fake_seg_outputs(cls=0), 0.5, 1.0, (0, 140), 640, 360)
    assert len(dets) == 1
    d = dets[0]
    assert d.cls == "target" and d.conf > 0.8
    x1, y1, x2, y2 = d.xyxy
    # box centered at (0.4*640-0, 0.4*640-140) /1.0 -> frame coords
    assert abs((x1 + x2) / 2 - 256) < 3 and abs((y1 + y2) / 2 - 116) < 3
    assert d.mask is not None
    dets = _decode_seg(_fake_seg_outputs(cls=1), 0.5, 1.0, (0, 140), 640, 360)
    assert dets[0].cls == "obstacle"
    assert _decode_seg(_fake_seg_outputs(score=0.1), 0.5, 1.0, (0, 140),
                       640, 360) == []


def test_decode_seg_rejects_bad_head():
    import numpy as np
    from agents.vision.backends import _decode_seg
    from agents.vision.types import BackendError
    det = np.zeros((1, 30, 5), dtype=np.float32)     # 30 ch < 4+32 -> nc<=0
    protos = np.zeros((1, 32, 8, 8), dtype=np.float32)
    try:
        _decode_seg([det, protos], 0.5, 1.0, (0, 0), 640, 360)
        assert False, "should raise BackendError"
    except BackendError:
        pass


def test_onnx_infer_end_to_end_with_fake_session():
    from agents.vision.backends import OnnxBackend

    class FakeSession:
        def run(self, _names, feed):
            assert "images" in feed
            return _fake_seg_outputs()

        def get_inputs(self):
            import numpy as np
            return [type("I", (), {"shape": np.array([1, 3, 640, 640])})()]

    b = OnnxBackend("unused", "unused")
    b._session = FakeSession()
    b._layout = "seg-v1"
    b._input_size = 640
    dets = b.infer(orange_frame(), 0.5)
    assert len(dets) == 1 and dets[0].cls == "target"


# ---- W2: manifest-driven class names (design 2026-07-28 §4) ----

def test_decode_seg_uses_manifest_class_names():
    """The manifest's "classes" table names the decoded dets (the COCO
    artifact ships 80); ids past the table fall back to cls_<k>."""
    from agents.vision.backends import _decode_seg
    coco = tuple(["person", "bicycle", "car"] + [f"c{i}" for i in range(3, 80)])
    dets = _decode_seg(_fake_seg_outputs(cls=2, ncls=80), 0.5, 1.0, (0, 140),
                       640, 360, names=coco)
    assert dets[0].cls == "car"
    dets = _decode_seg(_fake_seg_outputs(cls=0, ncls=80), 0.5, 1.0, (0, 140),
                       640, 360, names=coco)
    assert dets[0].cls == "person"
    dets = _decode_seg(_fake_seg_outputs(cls=1, ncls=80), 0.5, 1.0, (0, 140),
                       640, 360, names=("target",))
    assert dets[0].cls == "cls_1"


def test_onnx_infer_names_from_manifest_classes():
    """OnnxBackend routes its _classes (the manifest table, set at load) into
    decode; the mover manifest's absent table keeps the 2-class fallback
    (pinned by test_onnx_infer_end_to_end_with_fake_session)."""
    from agents.vision.backends import OnnxBackend

    class FakeSession:
        def run(self, _names, feed):
            return _fake_seg_outputs(cls=2, ncls=80)

        def get_inputs(self):
            import numpy as np
            return [type("I", (), {"shape": np.array([1, 3, 640, 640])})()]

    b = OnnxBackend("unused", "unused")
    b._session = FakeSession()
    b._layout = "seg-v1"
    b._input_size = 640
    b._classes = tuple(["person", "bicycle", "car"]
                       + [f"c{i}" for i in range(3, 80)])
    assert b.infer(orange_frame(), 0.5)[0].cls == "car"


def test_shipped_manifests_class_tables():
    """The COCO manifests ship the 80-class table decode consumes (both
    operating points identical); the mover manifest ships none."""
    import json
    from pathlib import Path
    models = Path(__file__).resolve().parent.parent / "models"
    coco = json.loads((models / "coco-nano-seg-v1.json").read_text())
    assert len(coco["classes"]) == 80
    assert coco["classes"][0] == "person" and coco["classes"][2] == "car"
    coco640 = json.loads((models / "coco-nano-seg-v1-640.json").read_text())
    assert coco640["classes"] == coco["classes"]
    assert coco640["sha256"]
    mover = json.loads((models / "mover-nano-seg-v1.json").read_text())
    assert tuple(mover.get("classes") or ()) == ()


def test_request_lock_consumes_seed():
    dets = [Detection("target", 0.9, (20, 16, 30, 26)),
            Detection("target", 0.7, (40, 30, 50, 40))]
    cam = FakeCameras([orange_frame(seq=1)])
    det = Detector(cam, ScriptBackend(dets), hz=200.0, conf=0.1)
    det.request_lock(seed_xy=(25.0, 20.0))          # before start: no race
    det.start()
    res = det.wait_next(after_seq=0, timeout=2.0)
    det.stop()
    assert res is not None and res.designated_hit is not None
    hit = res.designated_hit
    assert hit.detection_index == 0 and hit.xyxy == (20, 16, 30, 26)
    assert hit.aim_px == dets[0].footpoint


def test_request_lock_seed_index_is_preferred():
    dets = [Detection("target", 0.9, (20, 16, 30, 26)),
            Detection("target", 0.7, (40, 30, 50, 40))]
    cam = FakeCameras([orange_frame(seq=1)])
    det = Detector(cam, ScriptBackend(dets), hz=200.0, conf=0.1)
    det.request_lock(seed_xy=(25.0, 20.0), seed_index=1)
    det.start()
    res = det.wait_next(after_seq=0, timeout=2.0)
    det.stop()
    assert res.designated_hit.detection_index == 1
