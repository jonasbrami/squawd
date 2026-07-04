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
        assert t.setup.n_drones == 1
        assert t.suite in ("plan_depth", "spatial", "ambiguity", "capstone")
        assert t.difficulty.get(t.suite) is not None, f"{p} missing difficulty[{t.suite}]"


def test_obstacle_tasks_load_and_use_obstacles_world():
    paths = glob.glob("evals/tasks/obstacle/*.yaml")
    assert len(paths) == 4   # o1-o3 + c4 (2026-07-03)
    for p in paths:
        t = load_task(p)
        assert t.setup.world == "obstacles", f"{p} must use the obstacles world"
        assert t.setup.n_drones == 1
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


def test_swarm_tasks_load_operator_layer_and_verified_geometry():
    import glob
    import math
    from evals.spec import load_task

    paths = sorted(glob.glob("evals/tasks/swarm/*.yaml"))
    assert len(paths) == 5   # w1-w5
    for p in paths:
        t = load_task(p)
        assert t.target_layer == "operator" and t.suite == "swarm"
        assert t.setup.n_drones == 2
        assert t.pilot, f"{p} needs a pilot"

    # w4 uses dynamic world; all others use default
    assert load_task("evals/tasks/swarm/w4_double_intercept.yaml").setup.world == "dynamic"
    for name in ["w1_split_reach", "w2_allocation", "w3_crossing", "w5_sync_mark"]:
        assert load_task(f"evals/tasks/swarm/{name}.yaml").setup.world == "default"

    # w2 allocation numbers: budget must separate optimal from interleaved+solo
    A, B, C, D = (120, 20), (140, -30), (-100, 60), (-90, -70)
    s0, s1 = (0, 0), (0, 3)
    d = math.dist
    optimal = d(s0, A) + d(A, B) + d(s1, C) + d(C, D)
    interleaved = d(s0, A) + d(A, C) + d(s1, B) + d(B, D)
    solo = d(s0, C) + d(C, D) + d(D, B) + d(B, A)
    assert optimal < 460 < 500 < min(interleaved, solo), \
        (optimal, interleaved, solo)


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
