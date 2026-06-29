import pytest
from evals.spec import load_task, SpecError

VALID = """
id: reach_marker_single
target_layer: single_drone
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 2}
setup:
  world: baylands
  n_drones: 1
  spawn: home
  seed_objects:
    - {kind: marker, id: tgt_a, east: 120, north: -40}
prompt: "Take off and fly to the marker tgt_a at east 120, north -40."
budget: {wall_clock_s: 120, max_steps: 20}
oracle:
  - {check: alive}
  - {check: reached, target: tgt_a, tol_m: 15}
"""


def _write(tmp_path, text):
    p = tmp_path / "t.yaml"
    p.write_text(text)
    return str(p)


def test_loads_valid(tmp_path):
    t = load_task(_write(tmp_path, VALID))
    assert t.id == "reach_marker_single"
    assert t.setup.n_drones == 1
    assert t.objects_map() == {"tgt_a": (120.0, -40.0)}
    assert t.budget.max_steps == 20


def test_unknown_check_rejected(tmp_path):
    bad = VALID.replace("check: reached", "check: teleported")
    with pytest.raises(SpecError):
        load_task(_write(tmp_path, bad))


def test_missing_field_rejected(tmp_path):
    bad = VALID.replace("target_layer: single_drone", "")
    with pytest.raises(SpecError):
        load_task(_write(tmp_path, bad))


def test_bundled_task_file_loads():
    t = load_task("evals/tasks/reach_marker_single.yaml")
    assert t.target_layer == "single_drone"
