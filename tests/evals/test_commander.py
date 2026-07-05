"""CommanderSession (C1): a commander LLM dispatches per-drone LLM agents and
reads their reports. No real SDK/API involved — fakes mimic ScriptedClient's
surface (async ctx manager + query + receive_response yielding SDK message
objects) and, since there is no real MCP subprocess to execute the registered
`cmd` tools, the fakes call straight into CommanderSession's tool logic
(`_dispatch` / `_mark_done`) exactly like a live SDK round-trip would."""
import asyncio

from claude_agent_sdk import AssistantMessage, ToolResultBlock, ToolUseBlock, UserMessage

from evals.commander import CommanderSession
from evals.runner import Deps


class FakeCommanderClient:
    """`turns` is a list of per-query-call scripts; each script is a list of
    {"tool": ..., "args": {...}} dispatched in order within that one turn's
    receive_response(). Tool side effects run through `session` so dispatch/
    done have real effect, matching what a live SDK round-trip would do."""

    def __init__(self, session, turns):
        self.session = session
        self.turns = list(turns)
        self.queries = []
        self.tool_results = []  # [(tool, ok, text), ...] in call order

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        return False

    async def query(self, prompt):
        self.queries.append(prompt)

    async def receive_response(self):
        if not self.turns:
            return
        steps = self.turns.pop(0)
        for k, step in enumerate(steps):
            tool_id = f"c{k}"
            args = step.get("args", {})
            if step["tool"] == "dispatch":
                ok, text = await self.session._dispatch(args["drone"], args["task"])
            elif step["tool"] == "done":
                self.session._mark_done(args.get("summary", ""))
                ok, text = True, "mission marked done"
            elif step["tool"] == "situation":
                ok, text = True, self.session._situation()
            else:
                raise ValueError(f"unknown commander tool {step['tool']!r}")
            self.tool_results.append((step["tool"], ok, text))
            yield AssistantMessage(
                content=[ToolUseBlock(id=tool_id, name=f"mcp__cmd__{step['tool']}", input=args)],
                model="fake-commander")
            yield UserMessage(content=[ToolResultBlock(
                tool_use_id=tool_id, content=text, is_error=not ok)])


class HangingCommanderClient:
    """Never resolves — used to prove the deadline actually fires."""

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        return False

    async def query(self, prompt):
        pass

    async def receive_response(self):
        await asyncio.Event().wait()
        yield None  # pragma: no cover - unreachable, keeps this an async generator


class FakeDroneClient:
    """Scripted per-drone client: yields a fixed tool_use/tool_result sequence,
    including a final `report` call the worker parses out of the raw stream
    (mirroring how a real drone client's report() tool call would look)."""

    def __init__(self, i, task, calls):
        self.i = i
        self.task = task
        self.calls = list(calls)  # [(tool_name, args), ...]
        self.closed = False

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        self.closed = True
        return False

    async def query(self, prompt):
        pass

    async def receive_response(self):
        for k, (name, args) in enumerate(self.calls):
            yield AssistantMessage(
                content=[ToolUseBlock(id=f"d{self.i}_{k}", name=name, input=args)],
                model="fake-drone")
            yield UserMessage(content=[ToolResultBlock(
                tool_use_id=f"d{self.i}_{k}", content="ok", is_error=False)])


class HangingDroneClient:
    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        return False

    async def query(self, prompt):
        pass

    async def receive_response(self):
        await asyncio.Event().wait()
        yield None  # pragma: no cover


def _deps():
    return Deps(world=None, bridge=None, cameras=None)


def test_dispatch_routes_task_and_report_flows_into_next_turn():
    """dispatch(drone, task) hands the task to the RIGHT worker; the worker's
    report() call lands in self._reports and shows up in the next commander
    query() as REPORTS: text."""
    session = CommanderSession(_deps(), systems=[object(), object()],
                               commander_model="c", drone_model="d")
    turns = [
        [{"tool": "dispatch", "args": {"drone": "1", "task": "survey bldg_3"}}],
        [{"tool": "done", "args": {"summary": "ok"}}],
    ]
    fake = FakeCommanderClient(session, turns)
    session._client_factory = lambda options: fake

    seen_tasks = {}

    def drone_factory(i, task):
        seen_tasks[i] = task
        return FakeDroneClient(i, task, [
            (f"mcp__d{i}__report", {"message": "surveyed bldg_3, all clear"})])

    session._drone_client_factory = drone_factory

    async def go():
        return await session.run("find the target", deadline_s=5.0, max_steps=20)

    trace, crashed, reason = asyncio.run(go())
    assert seen_tasks == {1: "survey bldg_3"}
    assert not crashed
    assert reason == "commander done"
    assert len(fake.queries) == 2
    assert "drone_1" in fake.queries[1] and "surveyed bldg_3" in fake.queries[1]


def test_done_ends_the_commander_loop():
    session = CommanderSession(_deps(), systems=[object()],
                               commander_model="c", drone_model="d")
    fake = FakeCommanderClient(session, [
        [{"tool": "done", "args": {"summary": "mission complete"}}],
    ])
    session._client_factory = lambda options: fake
    session._drone_client_factory = lambda i, task: FakeDroneClient(i, task, [])

    async def go():
        return await session.run("mission", deadline_s=5.0, max_steps=20)

    trace, crashed, reason = asyncio.run(go())
    assert reason == "commander done"
    assert not crashed
    assert session._done is True


def test_dispatch_to_busy_drone_errors_without_queueing():
    """drone_0 is dispatched task-A, which never finishes (HangingDroneClient) ->
    it stays busy for the rest of the run; a second dispatch to it in the SAME
    turn must be rejected with a busy error, not queued behind the first."""
    session = CommanderSession(_deps(), systems=[object()],
                               commander_model="c", drone_model="d")
    dispatched = []

    def drone_factory(i, task):
        dispatched.append(task)
        return HangingDroneClient()  # never finishes -> stays busy

    fake = FakeCommanderClient(session, [
        [{"tool": "dispatch", "args": {"drone": 0, "task": "task-A"}},
         {"tool": "dispatch", "args": {"drone": "d0", "task": "task-B"}}],
    ])
    session._client_factory = lambda options: fake
    session._drone_client_factory = drone_factory

    async def go():
        return await session.run("mission", deadline_s=0.2, max_steps=20)

    trace, crashed, reason = asyncio.run(go())
    assert dispatched == ["task-A"]                 # task-B never reached a worker
    assert reason == "wall-clock deadline"
    assert fake.tool_results[0] == ("dispatch", True, "dispatched to drone_0")
    assert fake.tool_results[1] == ("dispatch", False, "drone_0 is busy")


def test_commander_steps_counted_separately_from_drone_steps():
    session = CommanderSession(_deps(), systems=[object()],
                               commander_model="c", drone_model="d")
    fake = FakeCommanderClient(session, [
        [{"tool": "dispatch", "args": {"drone": 0, "task": "go look"}}],
        [{"tool": "done", "args": {"summary": "done"}}],
    ])
    session._client_factory = lambda options: fake
    session._drone_client_factory = lambda i, task: FakeDroneClient(i, task, [
        ("mcp__d0__goto", {"east": 1, "north": 2}),
        ("mcp__d0__look", {}),
        ("mcp__d0__report", {"message": "seen"}),
    ])

    async def go():
        return await session.run("mission", deadline_s=5.0, max_steps=20)

    trace, crashed, reason = asyncio.run(go())
    assert trace.steps == 2                       # dispatch + done, commander-side only
    assert trace.meta["drone_steps"] == 3          # goto + look + report, drone-side


def test_deadline_cancels_workers_and_reports_the_reason():
    session = CommanderSession(_deps(), systems=[object(), object()],
                               commander_model="c", drone_model="d")
    session._client_factory = lambda options: HangingCommanderClient()
    session._drone_client_factory = lambda i, task: FakeDroneClient(i, task, [])

    async def go():
        return await session.run("mission", deadline_s=0.05, max_steps=20)

    trace, crashed, reason = asyncio.run(go())
    assert reason == "wall-clock deadline"
    assert not crashed
    assert session._workers                       # workers were created
    assert all(w.cancelled() for w in session._workers)
