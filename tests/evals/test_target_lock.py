"""M5 load-bearing ② (ICD §11): TargetLockEvent -> identified_target oracle
path (design §3.8, Codex-B5). The first track/goto aimed at a vis_* id is
associated to oracle truth AT that sim moment; the oracle grades the truth id
from run_meta — report text is never graded (§4.3)."""
from agents.flight.backend import Text, ToolCall

from evals.oracle import grade
from evals.perceive_eval import associate_to_truth, note_target_lock
from evals.runner import Trace
from evals.worldstate import WorldTrack


class FakeContacts:
    """The flight contact provider (VisionContacts duck-type, poses only)."""

    def __init__(self, poses, t=42.0):
        self._poses = poses
        self._t = t

    def poses(self):
        return dict(self._poses)

    def sim_time(self):
        return self._t

    def velocities(self):
        return {}


class FakeTruth:
    def __init__(self, poses):
        self._poses = poses

    def poses(self):
        return dict(self._poses)


def _tool_msg(name, args, mid="t1"):
    return ToolCall(id=mid, name=name, input=args, model="m")


def _trace_with(*events):
    tr = Trace()
    for i, ev in enumerate(events):
        tr.observe(ev, float(i + 1))
    return tr


CONTACTS = FakeContacts({"vis_target_0": (10.0, 0.0, 1.0)}, t=42.0)
TRUTH = FakeTruth({"mov_true": (11.0, 1.0, 1.2),
                   "mov_decoy_red": (60.0, 60.0, 1.5)})


def test_first_vis_lock_associates_to_nearest_truth_mover():
    tr = _trace_with(
        Text(text="scanning", model="m"),
        _tool_msg("mcp__pilot__track", {"target": "vis_target_0", "mode": "shadow"}),
    )
    note_target_lock(tr, CONTACTS, TRUTH)
    lock = tr.meta["target_lock"]
    assert lock["contact_id"] == "vis_target_0"
    assert lock["tool"] == "track"
    assert lock["sim_stamp"] == 42.0           # stamped at the sim moment, not wall time
    assert lock["truth_id"] == "mov_true"      # (10,0) is 1.41m from mov_true
    assert lock["assoc_err_m"] == 1.41


def test_only_first_lock_and_only_vis_targets_count():
    tr = _trace_with(
        _tool_msg("mcp__pilot__goto", {"target": "bldg_7"}, mid="t0"),
        _tool_msg("mcp__pilot__track", {"target": "vis_target_0"}, mid="t1"),
        _tool_msg("mcp__pilot__goto", {"target": "vis_decoy_1"}, mid="t2"),
    )
    note_target_lock(tr, CONTACTS, TRUTH)
    assert tr.meta["target_lock"]["contact_id"] == "vis_target_0"
    # idempotent: a second call must not rewrite the recorded lock
    tr2 = _trace_with(_tool_msg("pilot__track", {"target": "vis_decoy_1"}))
    note_target_lock(tr2, CONTACTS, TRUTH)
    assert tr2.meta["target_lock"]["contact_id"] == "vis_decoy_1"
    assert tr2.meta["target_lock"]["truth_id"] is None   # unknown contact -> no position


def test_lock_without_position_yields_no_truth_id():
    contacts = FakeContacts({}, t=1.0)          # bearing-only/unknown: no poses entry
    tr = _trace_with(_tool_msg("mcp__pilot__track", {"target": "vis_target_3"}))
    note_target_lock(tr, contacts, TRUTH)
    assert tr.meta["target_lock"]["truth_id"] is None


def test_association_gate_rejects_far_truth():
    far_truth = FakeTruth({"mov_true": (500.0, 500.0, 1.2)})
    tid, err = associate_to_truth((10.0, 0.0), far_truth)
    assert tid is None and err is None


def _grade_lock(lock_meta, truth_name):
    track = WorldTrack(snapshots=[], objects={}, geofence_m=300.0)
    run_meta = {"steps": 3, "crashed": False}
    if lock_meta is not None:
        run_meta["target_lock"] = lock_meta
    g = grade(track, [{"check": "identified_target", "truth": truth_name}], run_meta)
    return g


def test_identified_target_oracle_path_passes_on_correct_lock():
    tr = _trace_with(_tool_msg("mcp__pilot__track", {"target": "vis_target_0"}))
    note_target_lock(tr, CONTACTS, TRUTH)
    g = _grade_lock(tr.meta["target_lock"], "mov_true")
    assert g.passed and g.checks[0].name == "identified_target"
    # the empty track proves the check grades run_meta's association, never
    # the flight record or any report text (§4.3)


def test_identified_target_fails_on_wrong_or_missing_lock():
    tr = _trace_with(_tool_msg("mcp__pilot__track", {"target": "vis_target_0"}))
    note_target_lock(tr, CONTACTS, TRUTH)
    wrong = _grade_lock(tr.meta["target_lock"], "mov_decoy_red")
    assert not wrong.passed and "mov_decoy_red" in wrong.checks[0].detail
    none_g = _grade_lock(None, "mov_true")
    assert not none_g.passed and "no vis_* lock" in none_g.checks[0].detail
