"""§4.3 canonical task-injection envelope: exact rendered text is the contract."""
from types import SimpleNamespace

from evals.task_prompt import render_task_prompt


def _spec(prompt, wall, steps):
    return SimpleNamespace(prompt=prompt,
                           budget=SimpleNamespace(wall_clock_s=wall,
                                                  max_steps=steps))


def test_render_task_prompt_exact_text():
    out = render_task_prompt(_spec("find the rover and shadow it", 330, 30))
    assert out == (
        "MISSION: find the rover and shadow it\n"
        "BUDGET: you have 330s of wall-clock and at most 30 tool calls.\n"
        "SAFETY: stay inside the geofence; you may be halted externally at any time.\n"
        "When done, call report(...) with a short result (what you did and what you saw)."
    )
