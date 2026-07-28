"""The canonical task-injection envelope (design §4.3).

Today run_cell injects spec.prompt unchanged and budgets are enforced EXTERNALLY
by _drive — the model is not told its budget. v2 generates ONE canonical
envelope with the wall-clock + step budget, the safety contract, and the report
contract; tests assert the exact rendered text (render drift is a prompt bug).
"""


def render_task_prompt(spec) -> str:
    b = spec.budget
    return (
        f"MISSION: {spec.prompt}\n"
        f"BUDGET: you have {b.wall_clock_s:g}s of wall-clock and at most "
        f"{b.max_steps} tool calls.\n"
        "SAFETY: stay inside the geofence; you may be halted externally at any "
        "time.\n"
        "When done, call report(...) with a short result (what you did and "
        "what you saw)."
    )
