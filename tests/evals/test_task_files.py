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
