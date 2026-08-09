import httpx
import pytest

from agents.flight.backend import Result, Text, ToolCall, ToolResult
from agents.flight.codex_backend import (
    CodexBackendClient,
    PilotMCPServer,
    _low_level_server,
)
from agents.flight.tools import make_pilot_options, make_pilot_tools


class FakeOps:
    envelope = None

    def scan(self):
        return "scan"


def _specs():
    return make_pilot_tools(FakeOps(), report=lambda _message: None)


async def _listed_tools(server):
    from mcp import types

    request = types.ListToolsRequest(method="tools/list")
    result = await server.request_handlers[types.ListToolsRequest](request)
    return result.root.tools


async def test_codex_and_claude_adapters_publish_identical_tool_catalogs():
    specs = _specs()
    codex_tools = await _listed_tools(_low_level_server(specs))
    options = make_pilot_options(FakeOps(), report=lambda _message: None)
    claude_tools = await _listed_tools(options.mcp_servers["pilot"]["instance"])

    def catalog(tools):
        return {tool.name: (tool.description, tool.inputSchema) for tool in tools}

    assert catalog(codex_tools) == catalog(claude_tools)
    assert set(options.allowed_tools) == {
        f"mcp__pilot__{spec.name}" for spec in specs}


async def _call(server, name, arguments=None):
    from mcp import types

    request = types.CallToolRequest(
        method="tools/call", params=types.CallToolRequestParams(
            name=name, arguments=arguments or {}))
    return (await server.request_handlers[types.CallToolRequest](request)).root


async def test_codex_and_claude_share_guard_error_and_registry_semantics():
    from agents.flight.errors import OperatorActiveError

    class Registry:
        def __init__(self):
            self.registered = 0
            self.cleared = []

        def register(self, _task):
            self.registered += 1
            return self.registered

        def clear(self, generation):
            self.cleared.append(generation)

    def denied(_name):
        raise OperatorActiveError("operator lease held")

    results = []
    registries = []
    for adapter in ("codex", "claude"):
        registry = Registry()
        if adapter == "codex":
            specs = make_pilot_tools(
                FakeOps(), report=lambda _message: None,
                registry=registry, guard=denied)
            server = _low_level_server(specs)
        else:
            options = make_pilot_options(
                FakeOps(), report=lambda _message: None,
                registry=registry, guard=denied)
            server = options.mcp_servers["pilot"]["instance"]
        results.append(await _call(server, "scan"))
        registries.append(registry)

    assert results[0].model_dump() == results[1].model_dump()
    assert results[0].isError
    assert results[0].content[0].text == (
        "OPERATOR_ACTIVE: operator lease held")
    assert all(registry.registered == 0 and registry.cleared == []
               for registry in registries)


async def test_codex_and_claude_map_active_tool_cancellation_to_estopped():
    import asyncio

    class BlockingOps(FakeOps):
        async def hover(self, _seconds):
            await asyncio.Event().wait()

    class Registry:
        def __init__(self):
            self.cleared = []

        def register(self, _task):
            return 7

        def clear(self, generation):
            self.cleared.append(generation)

    results = []
    for adapter in ("codex", "claude"):
        registry = Registry()
        if adapter == "codex":
            server = _low_level_server(make_pilot_tools(
                BlockingOps(), report=lambda _message: None,
                registry=registry))
        else:
            options = make_pilot_options(
                BlockingOps(), report=lambda _message: None,
                registry=registry)
            server = options.mcp_servers["pilot"]["instance"]
        task = asyncio.create_task(_call(server, "hover", {"seconds": 5}))
        await asyncio.sleep(0)
        task.cancel()
        results.append(await task)
        assert registry.cleared == [7]

    assert results[0].model_dump() == results[1].model_dump()
    assert results[0].isError
    assert results[0].content[0].text == "ESTOPPED: operator halted hover"


async def test_loopback_mcp_requires_bearer_and_discovers_exact_tools():
    from mcp import ClientSession
    from mcp.client.streamable_http import streamable_http_client

    async with PilotMCPServer(_specs(), token="unit-test-token") as server:
        async with httpx.AsyncClient() as unauthorized:
            response = await unauthorized.post(server.url, json={})
        assert response.status_code == 401

        async with httpx.AsyncClient(
                headers={"Authorization": "Bearer unit-test-token"}) as http:
            async with streamable_http_client(
                    server.url, http_client=http) as (read, write, _session_id):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    tools = (await session.list_tools()).tools
        assert [tool.name for tool in tools] == [spec.name for spec in _specs()]


def _notifications():
    from openai_codex.generated.v2_all import (
        AgentMessageThreadItem,
        ItemCompletedNotification,
        ItemStartedNotification,
        McpToolCallResult,
        McpToolCallStatus,
        McpToolCallThreadItem,
        ThreadItem,
        ThreadTokenUsage,
        ThreadTokenUsageUpdatedNotification,
        TokenUsageBreakdown,
        Turn,
        TurnCompletedNotification,
        TurnStatus,
    )
    from openai_codex.models import Notification

    started = McpToolCallThreadItem(
        type="mcpToolCall", id="call-1", server="pilot", tool="scan",
        arguments={}, status=McpToolCallStatus.in_progress)
    completed = started.model_copy(update={
        "status": McpToolCallStatus.completed,
        "result": McpToolCallResult(
            content=[{"type": "text", "text": "scan complete"}]),
    })
    message = AgentMessageThreadItem(
        type="agentMessage", id="message-1", text="Done.")
    breakdown = TokenUsageBreakdown(
        cachedInputTokens=3, inputTokens=10, outputTokens=4,
        reasoningOutputTokens=2, totalTokens=16)
    turn = Turn(
        id="turn-1", status=TurnStatus.completed, items=[], durationMs=25)
    return [
        Notification(method="item/started", payload=ItemStartedNotification(
            item=ThreadItem(root=started), startedAtMs=1,
            threadId="thread-1", turnId="turn-1")),
        Notification(method="item/completed", payload=ItemCompletedNotification(
            item=ThreadItem(root=completed), completedAtMs=2,
            threadId="thread-1", turnId="turn-1")),
        Notification(method="item/completed", payload=ItemCompletedNotification(
            item=ThreadItem(root=message), completedAtMs=3,
            threadId="thread-1", turnId="turn-1")),
        Notification(method="thread/tokenUsage/updated",
                     payload=ThreadTokenUsageUpdatedNotification(
                         threadId="thread-1", turnId="turn-1",
                         tokenUsage=ThreadTokenUsage(
                             last=breakdown, total=breakdown))),
        Notification(method="turn/completed", payload=TurnCompletedNotification(
            threadId="thread-1", turn=turn)),
    ]


class _FakeHandle:
    async def stream(self):
        for notification in _notifications():
            yield notification


class _FakeThread:
    def __init__(self):
        self.prompts = []

    async def turn(self, prompt, **kwargs):
        self.prompts.append((prompt, kwargs))
        return _FakeHandle()


class _FakeSDK:
    def __init__(self):
        self.thread = _FakeThread()
        self.starts = []
        self.entered = False

    async def __aenter__(self):
        self.entered = True
        return self

    async def __aexit__(self, *_exc):
        self.entered = False

    async def thread_start(self, **kwargs):
        self.starts.append(kwargs)
        return self.thread


class _FakeMCP:
    def __init__(self, _tools):
        self.url = "http://127.0.0.1:43210/mcp"
        self.token = "generated-token"
        self.entered = False

    async def __aenter__(self):
        self.entered = True
        return self

    async def __aexit__(self, *_exc):
        self.entered = False


async def test_codex_client_normalizes_events_usage_and_reuses_one_thread(tmp_path):
    fake = _FakeSDK()
    client = CodexBackendClient(
        _specs(), system_prompt="pilot prompt", sdk_client=fake,
        mcp_server_factory=_FakeMCP, require_auth=False,
        workdir=str(tmp_path / "empty"))
    async with client:
        first = [event async for event in client.query("scan")]
        second = [event async for event in client.query("report")]

    assert [type(event) for event in first] == [ToolCall, ToolResult, Text, Result]
    assert first[0].name == "mcp__pilot__scan" and first[0].input == {}
    assert first[1].content == [{"type": "text", "text": "scan complete"}]
    assert first[2].text == "Done."
    assert first[3].usage == {
        "cached_input_tokens": 3, "input_tokens": 10, "output_tokens": 4,
        "reasoning_output_tokens": 2, "total_tokens": 16}
    assert first[3].inference_requests == 1
    assert first[3].is_error is False and first[3].api_ms == 25
    assert len(fake.starts) == 1
    assert [prompt for prompt, _kwargs in fake.thread.prompts] == ["scan", "report"]
    assert [type(event) for event in second] == [ToolCall, ToolResult, Text, Result]


async def test_codex_runtime_config_is_isolated_allowlisted_and_fail_closed(
        monkeypatch, tmp_path):
    import agents.flight.codex_backend as module

    fake = _FakeSDK()
    captured = {}

    def build_sdk(config):
        captured["config"] = config
        return fake

    monkeypatch.setattr(module, "AsyncCodex", build_sdk)
    monkeypatch.setenv("KIMI_API_KEY", "must-not-flow-to-codex")
    home = tmp_path / "codex"
    home.mkdir()
    (home / "auth.json").write_text("{}")
    workdir = tmp_path / "empty"
    client = CodexBackendClient(
        _specs(), system_prompt="pilot prompt", codex_home=str(home),
        workdir=str(workdir), mcp_server_factory=_FakeMCP)
    async with client:
        config = captured["config"]
        overrides = set(config.config_overrides)
        assert "mcp_servers.pilot.required=true" in overrides
        assert any(value.startswith("mcp_servers.pilot.enabled_tools=")
                   for value in overrides)
        assert "features.shell_tool=false" in overrides
        assert 'web_search="disabled"' in overrides
        assert 'shell_environment_policy.inherit="none"' in overrides
        assert config.cwd == str(workdir)
        assert config.env["CODEX_HOME"] == str(home)
        assert "KIMI_API_KEY" not in config.env
        assert fake.starts[0]["model"] == "gpt-5.6-terra"
        assert fake.starts[0]["ephemeral"] is True


async def test_codex_missing_auth_fails_before_mcp_start(tmp_path):
    calls = []

    def mcp_factory(tools):
        calls.append(tools)
        return _FakeMCP(tools)

    client = CodexBackendClient(
        _specs(), system_prompt="pilot", codex_home=str(tmp_path / "missing"),
        mcp_server_factory=mcp_factory)
    with pytest.raises(RuntimeError, match="codex login"):
        await client.__aenter__()
    assert calls == []


async def test_codex_mcp_startup_failure_is_not_hidden(tmp_path):
    class BrokenMCP(_FakeMCP):
        async def __aenter__(self):
            raise RuntimeError("MCP failed")

    client = CodexBackendClient(
        _specs(), system_prompt="pilot", sdk_client=_FakeSDK(),
        mcp_server_factory=BrokenMCP, require_auth=False,
        workdir=str(tmp_path))
    with pytest.raises(RuntimeError, match="MCP failed"):
        await client.__aenter__()
