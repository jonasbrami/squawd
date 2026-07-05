"""CommanderSession (C1): the commander LLM does not fly — it dispatches
per-drone LLM agents and reads their reports. This is the multi-agent step up
from the operator layer (one LLM flying all N drones itself): here the
commander only ever sees `situation()` + report text, and each drone gets its
own fresh single-drone client + system prompt (agents/flight/tools.make_drone_options),
one per dispatched task.

Architecture:
- Commander MCP server `cmd`: situation() / dispatch(drone, task) / done(summary).
  These are real tool functions (executed by the SDK for a live client, or by a
  test fake standing in for the SDK round-trip) that call straight into
  `_situation` / `_dispatch` / `_mark_done` below — those are the single source
  of truth for the session's state, so tests can drive them directly instead of
  faking an entire SDK subprocess.
- One worker coroutine per drone: `while True: task = await queue.get()`, build a
  FRESH single-drone client for that one task, drain it (each tool call counts
  toward `_drone_steps`; a `report` tool call is parsed out of the raw message
  stream and appended to `_reports` — mirroring how Trace.observe reads tool
  calls off the stream rather than requiring a callback to fire).
- Commander drive loop: query the mission prompt, drain into the shared Trace
  (`trace.steps` therefore counts ONLY commander tool calls); then repeat:
  done -> stop; new reports -> feed them back; all idle + no reports for
  STATUS_IDLE_S -> nudge for a status check. Deadline + step budget checked
  every turn; workers are cancelled on the way out either way.
"""
import asyncio
import os
import shutil
import time

from claude_agent_sdk import (AssistantMessage, ClaudeAgentOptions, ClaudeSDKClient,
                              ToolUseBlock, create_sdk_mcp_server, tool)

from agents.flight import make_drone_options
from agents.flight.fleet import FleetOps
from agents.perception import situation_text
from evals.runner import Trace


def _agent_env(i: int) -> dict:
    """Per-drone CLAUDE_CONFIG_DIR, SEEDED with credentials (same discipline as
    agents/swarm/run.py agent_env): an empty isolated config dir silently
    no-ops the client — observed live as a commander dispatching perfectly
    while every drone sat parked (drone_steps=0). No creds found -> no
    override (single shared config beats N dead clients)."""
    base = os.environ.get("CLAUDE_CONFIG_DIR", os.path.expanduser("~/.claude"))
    src = os.path.join(base, ".credentials.json")
    if not os.path.exists(src):
        return {}
    d = f"/tmp/claude-agent-{i}"
    os.makedirs(d, exist_ok=True)
    try:
        shutil.copy(src, os.path.join(d, ".credentials.json"))
    except Exception:
        return {}
    return {"CLAUDE_CONFIG_DIR": d}

STATUS_IDLE_S = 20.0   # nudge the commander if the fleet goes quiet this long
_POLL_S = 0.05          # how often the drive loop rechecks reports/idle while waiting

COMMANDER_SYSTEM_PROMPT = (
    "You are the COMMANDER of a fleet of {n} drones. You do not fly; you delegate.\n"
    "TOOLS: situation() — live fleet map; dispatch(drone, task) — send ONE drone a\n"
    "self-contained natural-language task (it flies autonomously and reports back);\n"
    "done(summary) — end the mission when the objective is met.\n"
    "Write dispatch tasks the way you'd brief a pilot who knows nothing else:\n"
    "exact coordinates, altitudes, hold times, constraints. Drones cannot hear\n"
    "each other; only you see the whole picture. Re-dispatch on bad reports.\n"
    "Mind the fleet constraints in the mission (separation, budgets, windows)."
)


def _ok(text: str) -> dict:
    return {"content": [{"type": "text", "text": text}]}


def _err(text: str) -> dict:
    return {"content": [{"type": "text", "text": text}], "is_error": True}


def _is_report_call(name: str) -> bool:
    """Matches both the real MCP name (`mcp__d{i}__report`) and a bare "report"
    (test fakes) — the worker reads this off the raw tool-call stream, the same
    way Trace.observe reads tool calls, rather than depending on a callback."""
    return name.rsplit("__", 1)[-1] == "report"


class CommanderSession:
    def __init__(self, deps, systems, commander_model, drone_model,
                client_factory=None, drone_client_factory=None) -> None:
        self._deps = deps
        self._systems = list(systems)
        self._n = len(self._systems)
        self._commander_model = commander_model
        self._drone_model = drone_model
        self._client_factory = client_factory
        self._drone_client_factory = drone_client_factory

        self._done = False
        self._done_summary = ""
        self._reports: list[tuple[int, str]] = []
        self._drone_steps = 0
        self._busy: dict[int, bool] = {}
        self._queues: dict[int, "asyncio.Queue"] = {}
        self._workers: list[asyncio.Task] = []

    # ---- tool-backing logic (single source of truth; MCP tools + test fakes
    #      both call these) ----

    def _situation(self) -> str:
        base = situation_text(self._deps.world, self._deps.bridge, self._n)
        recent = self._reports[-10:]
        if not recent:
            return base
        lines = "\n".join(f"drone_{i}: {msg}" for i, msg in recent)
        return f"{base}\n\nRecent reports:\n{lines}"

    async def _dispatch(self, drone, task: str) -> tuple[bool, str]:
        try:
            idx = FleetOps._coerce_id(drone)
        except (TypeError, ValueError):
            return False, f"unknown drone {drone!r}"
        if not 0 <= idx < self._n:
            return False, f"unknown drone {drone!r}"
        if self._busy.get(idx):
            return False, f"drone_{idx} is busy"
        self._busy[idx] = True
        self._queues[idx].put_nowait(task)
        return True, f"dispatched to drone_{idx}"

    def _mark_done(self, summary: str) -> None:
        self._done = True
        self._done_summary = summary

    # ---- commander MCP server ----

    def _commander_options(self) -> ClaudeAgentOptions:
        @tool("situation", "Live fleet map: each drone's position + nearest "
              "building, plus the last 10 reports from drones.", {})
        async def situation_tool(args):
            return _ok(self._situation())

        @tool("dispatch", "Send ONE drone a self-contained natural-language task "
              "(it flies autonomously and reports back).",
              {"drone": {"type": "string"}, "task": {"type": "string"}})
        async def dispatch_tool(args):
            ok, text = await self._dispatch(args.get("drone"), args.get("task", ""))
            return _ok(text) if ok else _err(text)

        @tool("done", "End the mission when the objective is met.",
              {"summary": {"type": "string"}})
        async def done_tool(args):
            self._mark_done(args.get("summary", ""))
            return _ok("mission marked done")

        server = create_sdk_mcp_server(name="cmd",
                                       tools=[situation_tool, dispatch_tool, done_tool])
        return ClaudeAgentOptions(
            mcp_servers={"cmd": server},
            allowed_tools=["mcp__cmd__situation", "mcp__cmd__dispatch", "mcp__cmd__done"],
            setting_sources=[],
            model=self._commander_model,
            system_prompt=COMMANDER_SYSTEM_PROMPT.format(n=self._n),
        )

    def _build_commander_client(self):
        options = self._commander_options()
        if self._client_factory is not None:
            return self._client_factory(options)
        return ClaudeSDKClient(options=options)

    # ---- per-drone workers ----

    def _build_drone_client(self, i: int, task: str):
        if self._drone_client_factory is not None:
            return self._drone_client_factory(i, task)
        opts = make_drone_options(
            i, self._systems[i], self._deps.world, self._deps.bridge, self._n,
            self._deps.cameras, report=lambda _m: None,
            env=_agent_env(i), model=self._drone_model,
            gzposes=self._deps.gzposes)
        return ClaudeSDKClient(options=opts)

    async def _worker(self, i: int) -> None:
        while True:
            task = await self._queues[i].get()
            try:
                client = self._build_drone_client(i, task)
                async with client:
                    await client.query(task)
                    async for msg in client.receive_response():
                        if isinstance(msg, AssistantMessage):
                            for blk in msg.content:
                                if isinstance(blk, ToolUseBlock):
                                    self._drone_steps += 1
                                    if _is_report_call(blk.name):
                                        self._reports.append(
                                            (i, blk.input.get("message", "")))
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self._reports.append((i, f"ERROR: {e}"))
            finally:
                self._busy[i] = False

    # ---- the drive loop ----

    async def run(self, prompt: str, deadline_s: float, max_steps: int
                 ) -> tuple[Trace, bool, str]:
        """Same (trace, crashed, reason) triple as evals.runner._drive. trace.steps
        counts COMMANDER tool calls only; trace.meta["drone_steps"] is the total
        across every dispatched drone task."""
        trace = Trace()
        trace.meta["drone_steps"] = 0   # Trace.meta is first-class, default {}; mutate it
        self._done = False
        self._done_summary = ""
        self._reports = []
        self._drone_steps = 0
        self._busy = {i: False for i in range(self._n)}
        self._queues = {i: asyncio.Queue() for i in range(self._n)}

        t0 = time.monotonic()
        reason = ""
        # Build the client BEFORE spawning workers: a client-build failure must not
        # leak unstarted worker tasks with nothing left to cancel them.
        client = self._build_commander_client()
        self._workers = [asyncio.create_task(self._worker(i)) for i in range(self._n)]

        async def _drain_turn():
            async for msg in client.receive_response():
                trace.observe(msg, time.monotonic() - t0)
                if trace.steps > max_steps:
                    return "step budget exceeded"
            return None

        async def _run():
            nonlocal reason
            async with client:
                await client.query(prompt)
                reason = await _drain_turn()
                if reason:
                    return
                last_report_len = 0
                last_activity = time.monotonic()
                while True:
                    if self._done:
                        reason = "commander done"
                        return
                    if trace.steps > max_steps:
                        reason = "step budget exceeded"
                        return
                    new_reports = self._reports[last_report_len:]
                    if new_reports:
                        last_report_len = len(self._reports)
                        last_activity = time.monotonic()
                        formatted = "\n".join(f"drone_{i}: {m}" for i, m in new_reports)
                        await client.query(f"REPORTS:\n{formatted}")
                    elif (not any(self._busy.values())
                          and time.monotonic() - last_activity >= STATUS_IDLE_S):
                        last_activity = time.monotonic()
                        await client.query(
                            "STATUS: all drones idle, no new reports. Finish or "
                            "re-dispatch.")
                    else:
                        await asyncio.sleep(_POLL_S)
                        continue
                    reason = await _drain_turn()
                    if reason:
                        return

        try:
            await asyncio.wait_for(_run(), timeout=deadline_s)
        except asyncio.TimeoutError:
            reason = "wall-clock deadline"
        finally:
            for w in self._workers:
                w.cancel()
            await asyncio.gather(*self._workers, return_exceptions=True)

        trace.meta["drone_steps"] = self._drone_steps
        return trace, False, reason
