"""M5 load-bearing ③ (ICD §11): per-cell VisionContacts.reset() at soft_reset
— no EKF filter state or vis_* ID leaks across anchored repeats (design §3.8).
The runner hook must reset the FLIGHT contacts only, never the oracle truth."""
import math

from agents.core.contact import Frame
from agents.vision.contacts import VisionContacts
from agents.vision.types import Detection, InferenceResult
from evals.runner import Deps, reset_per_cell

W, H = 640, 360
ALT = 12.0


class FakeWorld:
    def pose_at(self, t):
        return (0.0, 0.0, ALT, 0.0)

    def attitude_at(self, t):
        return (0.0, 0.0, 0.0)


def _px(ax, ay):
    from agents.perception.projection import vfov_deg
    fx = (W / 2) / math.tan(math.radians(69.0) / 2)
    fy = (H / 2) / math.tan(math.radians(vfov_deg(W, H)) / 2)
    return (W / 2 + fx * math.tan(ax), H / 2 + fy * math.tan(ay))


def _det_target(e, n):
    g = math.hypot(e, n)
    ax = math.atan2(e, n)
    ay = math.asin(min((ALT - 0.6) / g, 1.0))
    u, v = _px(ax, ay)
    return Detection("target", 0.9, (u - 3.0, v - 6.0, u + 3.0, v))


_seq = [0]


def _result(t, dets):
    _seq[0] += 1
    return InferenceResult(Frame(_seq[0], t, W, H, b""), list(dets), 0.0)


def _birth(vc, t0=100.0):
    vc.update(_result(t0, [_det_target(0.0, 40.0)]))
    vc.update(_result(t0 + 0.2, [_det_target(0.0, 40.0)]))


class FakeTruth:
    """GzPoses duck-type: NO reset method — the truth-fed control lane."""

    def poses(self):
        return {"mov_true": (11.0, 1.0, 1.2)}

    def sim_time(self):
        return 1.0

    def anchor(self):
        pass


class FakeVisionContacts:
    def __init__(self):
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1


def test_reset_per_cell_resets_flight_contacts_only():
    vc = FakeVisionContacts()
    deps = Deps(world=None, bridge=None, cameras=None,
                oracle_truth=FakeTruth(), flight_contacts=vc)
    reset_per_cell(deps)
    reset_per_cell(deps)
    assert vc.reset_calls == 2                       # once per anchored repeat


def test_reset_per_cell_tolerates_truth_fed_control():
    # the explicit truth-fed lane (flight_contacts IS a GzPoses) has no reset —
    # the hook must be a no-op there, never an AttributeError
    deps = Deps(world=None, bridge=None, cameras=None,
                oracle_truth=FakeTruth(), flight_contacts=FakeTruth())
    reset_per_cell(deps)
    reset_per_cell(Deps(world=None, bridge=None, cameras=None))   # no contacts


def test_vision_contacts_reset_leaves_no_filter_or_id_leak():
    vc = VisionContacts(FakeWorld())
    _birth(vc)
    assert vc.poses() and vc.all_views()             # track alive pre-reset
    first_name = vc.all_views()[0].name
    vc.reset()
    assert vc.poses() == {} and vc.all_views() == []
    assert vc.sim_time() == 0.0
    # the NEXT cell rebirths under the SAME id namespace — a leak would show
    # as a monotonically increasing counter (vis_target_1, _2, ...) or a
    # graveyard rebind of the stale track
    _birth(vc)
    assert [v.name for v in vc.all_views()] == [first_name]
