# tests/test_prompts.py
from dronebot.agent.prompts import SYSTEM_PROMPT


def test_prompt_sets_role_and_safety_framing():
    p = SYSTEM_PROMPT.lower()
    assert "drone" in p
    assert "tool" in p           # must act via tools, not narration
    assert "status" in p         # encourage grounding in real state
