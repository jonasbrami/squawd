import glob
from evals.spec import load_task


def test_all_flat_world_tasks_load_and_are_flat():
    paths = (glob.glob("evals/tasks/plan_depth/*.yaml")
             + glob.glob("evals/tasks/spatial/*.yaml")
             + glob.glob("evals/tasks/ambiguity/*.yaml")
             + glob.glob("evals/tasks/capstone/*.yaml"))
    assert len(paths) == 19   # 11 original + 8 discrimination rungs (2026-07-02)
    for p in paths:
        t = load_task(p)
        assert t.setup.world in ("default", "lawn"), f"{p} must use a flat world"
        assert t.setup.n_drones == 1
        assert t.suite in ("plan_depth", "spatial", "ambiguity", "capstone")
        assert t.difficulty.get(t.suite) is not None, f"{p} missing difficulty[{t.suite}]"
