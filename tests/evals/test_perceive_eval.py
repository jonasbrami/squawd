"""M5 load-bearing ① (ICD §11): accuracy_report fixtures — timestamp-joined
precision/recall at IoU>=0.5, center-error by TRUTH range, ID-switch rate +
track fragmentation. Pure + offline: synthetic frames + scripted backend."""
import pytest

from agents.core.contact import Frame
from agents.vision.types import Detection
from evals.perceive_eval import accuracy_report, join_frames_to_truth

W, H = 640, 360


def frame(seq, stamp):
    return Frame(seq, stamp, W, H, bytes(W * H * 3))


class ScriptBackend:
    """Returns the scripted detections for each frame seq."""

    def __init__(self, per_frame):
        self._pf = per_frame

    def infer(self, f, conf):
        return list(self._pf.get(f.seq, []))


def truth(stamp, boxes, ids=None, ranges=None):
    row = {"stamp": stamp, "boxes": boxes}
    if ids is not None:
        row["ids"] = ids
    if ranges is not None:
        row["ranges"] = ranges
    return row


TBOX = (10.0, 10.0, 20.0, 20.0)   # truth box; center (15, 15)


def _fixture_frames_truths():
    frames = [frame(1, 10.00), frame(2, 10.10), frame(3, 10.20),
              frame(4, 10.30), frame(5, 10.60)]
    # truth stamped OFF the detector ticks (10 Hz det vs ~2.5 Hz truth) — the
    # join must tolerate the 40 ms skew (Codex-B2: not "same tick")
    truths = [
        truth(10.04, {"target": [TBOX]}, {"target": ["mov_true"]}, {"target": [25.0]}),
        truth(10.14, {"target": [TBOX]}, {"target": ["mov_true"]}, {"target": [25.0]}),
        truth(10.20, {"target": [TBOX]}, {"target": ["mov_true"]}, {"target": [25.0]}),
        truth(10.30, {"target": [TBOX]}, {"target": ["mov_true"]}, {"target": [45.0]}),
    ]
    return frames, truths


def _fixture_backend():
    return ScriptBackend({
        1: [Detection("target", 0.9, (11.0, 10.0, 21.0, 20.0), tid=7)],   # TP (center err 1px)
        2: [Detection("target", 0.9, TBOX, tid=7),                        # TP
            Detection("target", 0.8, (40.0, 40.0, 50.0, 50.0), tid=8)],   # FP
        3: [],                                                            # FN (track gap)
        4: [Detection("target", 0.9, TBOX, tid=9)],                       # TP, NEW tid -> ID switch
        5: [Detection("target", 0.9, TBOX, tid=9)],                       # unjoined frame
    })


def test_accuracy_report_precision_recall_and_join():
    frames, truths = _fixture_frames_truths()
    rep = accuracy_report(frames, truths, _fixture_backend())
    # frame 5 (t=10.60) has no truth within 50 ms -> excluded, counted
    assert (rep["n_frames"], rep["n_joined"], rep["n_unjoined"]) == (5, 4, 1)
    st = rep["per_class"]["target"]
    assert (st["tp"], st["fp"], st["fn"]) == (3, 1, 1)
    assert st["precision"] == 0.75 and st["recall"] == 0.75


def test_accuracy_report_center_error_by_truth_range():
    frames, truths = _fixture_frames_truths()
    rep = accuracy_report(frames, truths, _fixture_backend())
    # errors: 1.0 (f1), 0.0 (f2), 0.0 (f4); f3 unmatched -> no error sample
    assert rep["center_err_p50"] == 0.0
    by = rep["center_err_by_range"]
    assert by["<=30m"]["n"] == 2 and by["30-60m"]["n"] == 1
    assert by["<=30m"]["p50"] == 0.5          # median of [0.0, 1.0]


def test_accuracy_report_id_switch_and_fragmentation():
    frames, truths = _fixture_frames_truths()
    rep = accuracy_report(frames, truths, _fixture_backend())
    # mov_true held tid 7 for 2 frames, gapped at f3, returned as tid 9:
    # exactly one ID switch and one broken track segment over 3 matched pairs
    assert rep["id_switches"] == 1
    assert rep["fragmentations"] == 1
    assert rep["id_switch_rate"] == 1 / 3
    assert rep["fragmentation_rate"] == 1 / 3


def test_accuracy_report_id_metrics_none_without_track_ids():
    frames = [frame(1, 10.00)]
    truths = [truth(10.00, {"target": [TBOX]}, {"target": ["mov_true"]})]

    class NoTidBackend(ScriptBackend):
        pass

    rep = accuracy_report(frames, truths,
                          NoTidBackend({1: [Detection("target", 0.9, TBOX)]}))
    assert rep["id_switches"] is None and rep["id_switch_rate"] is None
    assert rep["fragmentations"] is None


def test_accuracy_report_separates_classes():
    frames = [frame(1, 10.00)]
    truths = [truth(10.00, {"target": [TBOX], "decoy": [(40.0, 40.0, 50.0, 50.0)]})]
    backend = ScriptBackend({
        1: [Detection("decoy", 0.9, (40.0, 40.0, 50.0, 50.0)),   # decoy TP
            Detection("target", 0.9, (100.0, 100.0, 110.0, 110.0))],  # target FP
    })
    rep = accuracy_report(frames, truths, backend)
    assert rep["per_class"]["decoy"]["tp"] == 1
    assert rep["per_class"]["target"]["fp"] == 1
    assert rep["per_class"]["target"]["recall"] == 0.0   # truth target missed


def test_join_tolerance_is_50ms_not_same_tick():
    frames = [frame(1, 10.00), frame(2, 10.20)]
    truths = [truth(10.04, {"target": []}),     # 40 ms off -> joins f1
              truth(10.16, {"target": []})]     # 40 ms off -> joins f2
    joined, unjoined = join_frames_to_truth(frames, truths)
    assert len(joined) == 2 and not unjoined
    joined, unjoined = join_frames_to_truth(frames, [truth(10.30, {"target": []})])
    assert len(joined) == 0 and len(unjoined) == 2   # 100+ ms off -> no join


def test_project_truth_box_roundtrips_through_pixel_to_angles():
    """The world→pixel projection must be the exact inverse of the forward
    path (pixel_to_angles) at box center — else live truth boxes would be
    systematically offset from what the detector sees."""
    import math

    from agents.perception.projection import pixel_to_angles
    from evals.perceive_eval import project_truth_box

    # drone at (0,0,12) heading north(0); mover E10 N40, base-center z 0.6
    box, slant = project_truth_box(0.0, 0.0, 12.0, 0.0, 10.0, 40.0, 0.6,
                                   1.8, 1.2, W, H)
    assert box is not None
    cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    ax, ay = pixel_to_angles(cx, cy, W, H)
    assert ax == pytest.approx(math.atan2(10.0, 40.0), abs=1e-6)
    assert ay == pytest.approx(math.atan2(12.0 - 0.6, math.hypot(10.0, 40.0)),
                               abs=1e-6)
    assert slant == pytest.approx(math.hypot(math.hypot(10.0, 40.0), 11.4))
    # behind the camera / out of FOV -> no box, but the range still returns
    assert project_truth_box(0, 0, 12, 0.0, 0.0, -40.0, 0.6, 1.8, 1.2, W, H)[0] is None
    assert project_truth_box(0, 0, 12, 0.0, 0.0, 40.0, 0.6, 1.8, 1.2, W, H)[0] is not None
