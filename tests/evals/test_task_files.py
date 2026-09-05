import glob
from evals.spec import load_task


def test_all_flat_world_tasks_load_and_are_flat():
    paths = (glob.glob("evals/tasks/plan_depth/*.yaml")
             + glob.glob("evals/tasks/spatial/*.yaml")
             + glob.glob("evals/tasks/ambiguity/*.yaml")
             + glob.glob("evals/tasks/capstone/*.yaml"))
    assert len(paths) == 22   # 11 original + 8 discrimination + 3 ceiling rungs (2026-07-02)
    for p in paths:
        t = load_task(p)
        assert t.setup.world in ("default", "lawn"), f"{p} must use a flat world"
        assert t.suite in ("plan_depth", "spatial", "ambiguity", "capstone")
        assert t.difficulty.get(t.suite) is not None, f"{p} missing difficulty[{t.suite}]"


def test_obstacle_tasks_load_and_use_obstacles_world():
    paths = glob.glob("evals/tasks/obstacle/*.yaml")
    assert len(paths) == 4   # o1-o3 + c4 (2026-07-03)
    for p in paths:
        t = load_task(p)
        assert t.setup.world == "obstacles", f"{p} must use the obstacles world"
        assert t.suite == "obstacle"
        assert t.pilot, f"{p} must declare a pilot (trap gate)"
        # 2D clearance grading means overflight = collision: every obstacle task
        # must pin the drone below building height.
        assert any(c["check"] == "alt_ceiling" for c in t.oracle), f"{p} needs alt_ceiling"


def test_dynamic_tasks_load_with_dual_baselines():
    import glob
    from evals.spec import load_task

    paths = sorted(glob.glob("evals/tasks/dynamic/*.yaml"))
    assert len(paths) == 5   # d1-d5 (dynamic ladder L1-L5, 2026-07-03)
    for p in paths:
        t = load_task(p)
        assert t.setup.world == "dynamic", f"{p} must use the dynamic world"
        assert t.suite == "dynamic"
        assert t.pilot, f"{p} needs a must-PASS pilot"
        checks = {c["check"] for c in t.oracle}
        assert checks & {"intercept", "dwell_moving", "avoid_moving", "escort"}, \
            f"{p} must grade against a mover"
    # every rung above the entry rung carries the must-FAIL naive baseline
    for p in paths[1:]:
        assert load_task(p).null_pilot, f"{p} needs a null_pilot"


def test_step_budget_check_matches_budget_everywhere():
    """within_step_budget's max_steps must equal budget.max_steps in EVERY task —
    a drifted pair grades against a budget the prompt never promised."""
    import glob
    from evals.spec import load_task

    for p in sorted(glob.glob("evals/tasks/**/*.yaml", recursive=True)):
        t = load_task(p)
        for chk in t.oracle:
            if chk["check"] == "within_step_budget":
                assert int(chk["max_steps"]) == t.budget.max_steps, p


def test_perceive_tasks_load_with_dual_gates_and_identity_check():
    """Perceive ladder (M5): true target + visually distinct decoys in the
    perceive world; every rung grades the IDENTIFICATION act (TargetLockEvent
    path) plus a mover shadow, and carries both pilot baselines. The glob is
    the p*-RUNGS: s6_kimi_spike is an M6 backend smoke (no baselines by
    design) with its own loader test below — the dual gate applies to every
    ladder rung, current and future."""
    import glob
    from evals.spec import load_task

    paths = sorted(glob.glob("evals/tasks/perceive/p*.yaml"))
    assert len(paths) == 2   # p1_identify + p2_crossing (2026-07-22)
    for p in paths:
        t = load_task(p)
        assert t.setup.world == "perceive", f"{p} must use the perceive world"
        assert t.suite == "perceive"
        assert t.pilot, f"{p} needs a must-PASS pilot"
        assert t.null_pilot, f"{p} needs a must-FAIL null_pilot"
        id_checks = [c for c in t.oracle if c["check"] == "identified_target"]
        assert id_checks and id_checks[0].get("truth") == "mov_true", p
        assert any(c["check"] == "dwell_moving" and c.get("mover") == "mov_true"
                   for c in t.oracle), p


def test_s6_kimi_spike_loads_as_a_four_step_perceive_smoke():
    """M6 S6 (design §5.6): the first live Kimi cell — exactly take_off ->
    scan -> detect -> report on the perceive world, graded alive + the step
    budget. A backend smoke, NOT a ladder rung (no pilot baselines)."""
    from evals.spec import load_task

    t = load_task("evals/tasks/perceive/s6_kimi_spike.yaml")
    assert t.id == "s6_kimi_spike"
    assert t.setup.world == "perceive" and t.suite == "perceive"
    assert t.budget.max_steps == 4
    assert [c["check"] for c in t.oracle] == ["alive", "within_step_budget"]
    budget_chk = t.oracle[1]
    assert int(budget_chk["max_steps"]) == t.budget.max_steps


def test_backend_switch_smoke_has_identical_bounded_pilot_sequence():
    from evals.spec import load_task

    t = load_task("evals/tasks/smoke/backend_switch.yaml")
    assert t.setup.world == "default"
    assert t.budget.max_steps == 4
    assert [s["tool"] for s in t.pilot] == [
        "take_off", "scan", "report", "land"]
    assert [c["check"] for c in t.oracle] == [
        "alive", "final_pos", "landed", "within_step_budget"]
