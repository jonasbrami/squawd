"""Scripted reference pilot: fly a task's ideal tool sequence with NO LLM.

The single biggest validity threat found in the first sweeps was tooling, not
models (the fire-and-forget goto trap zeroed plan_depth for every tier and
inverted c1). The pilot is the gate: each task YAML may declare `pilot:` — the
ideal tool calls — and `run_evals --pilot` drives them through the SAME
run_cell / sampler / oracle path as a real agent. A task whose pilot can't pass
is quarantined as a harness bug before any LLM cells are spent on it; run it at
K=3 and the reach-distance spread is the sim-noise floor for free.

ScriptedClient mimics the ClaudeSDKClient surface run_cell uses (async context
manager + query + receive_response) and yields REAL SDK message objects, so
Trace, step budgets, and transcripts.jsonl work unchanged."""
from claude_agent_sdk import AssistantMessage, ToolResultBlock, ToolUseBlock, UserMessage


class ScriptedClient:
    """Executes `script` (list of {tool, args}) against a FlightOps built lazily by
    `ops_provider` (the shared MAVSDK System isn't connected until the harness is)."""

    def __init__(self, ops_provider, script: list[dict]) -> None:
        self._ops_provider = ops_provider
        self._script = list(script or [])

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        return False

    async def query(self, prompt: str) -> None:  # the pilot ignores the prose
        pass

    async def receive_response(self):
        ops = await self._ops_provider()
        for i, step in enumerate(self._script):
            tool = step["tool"]
            args = step.get("args", {})
            yield AssistantMessage(
                content=[ToolUseBlock(id=f"pilot{i}", name=f"pilot__{tool}", input=args)],
                model="pilot")
            fn = getattr(ops, tool, None)
            if fn is None or tool.startswith("_"):
                raise ValueError(f"pilot step {i}: unknown tool '{tool}'")
            out = fn(**args)
            if hasattr(out, "__await__"):
                out = await out
            yield UserMessage(content=[ToolResultBlock(
                tool_use_id=f"pilot{i}", content=str(out), is_error=False)])


def pilot_client_builder(harness, deps):
    """A DroneHarness client_builder for pilot mode. The script varies per cell, so
    run_evals sets `harness.pilot_script` before each run_cell; the builder reads it."""

    async def ops_provider():
        from agents.flight.ops import FlightOps
        system = await harness.system()
        return FlightOps(system, deps.world, deps.bridge, 0, 1,
                         gzposes=deps.gzposes)

    def build(model):
        return ScriptedClient(ops_provider, getattr(harness, "pilot_script", []))

    return build
