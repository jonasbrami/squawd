"""Codex SDK backend and authenticated loopback MCP transport.

The flight handlers come from :mod:`agents.flight.tools`; this module only
adapts their provider-neutral definitions to Streamable HTTP MCP and normalizes
Codex app-server notifications into the existing backend event contract.
"""
from __future__ import annotations

import asyncio
import contextlib
import json
import os
import secrets
import socket
import statistics
import tempfile
import time
from collections.abc import AsyncIterator, Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import uvicorn
from mcp import types as mcp_types
from mcp.server import Server
from mcp.server.fastmcp.server import StreamableHTTPASGIApp
from mcp.server.streamable_http_manager import StreamableHTTPSessionManager
from mcp.server.transport_security import TransportSecuritySettings
from openai_codex import ApprovalMode, AsyncCodex, CodexConfig, Sandbox
from openai_codex.generated.v2_all import (
    AgentMessageThreadItem,
    ErrorNotification,
    ItemCompletedNotification,
    ItemStartedNotification,
    McpToolCallStatus,
    McpToolCallThreadItem,
    ThreadTokenUsageUpdatedNotification,
    TurnCompletedNotification,
    TurnStatus,
)
from openai_codex.models import Notification
from openai_codex.types import ReasoningEffort
from starlette.responses import PlainTextResponse
from starlette.routing import Route
from starlette.applications import Starlette

from agents.flight.tools import ToolSpec

DEFAULT_CODEX_MODEL = "gpt-5.6-terra"
DEFAULT_CODEX_EFFORT = "low"
_MCP_TOKEN_ENV = "SQUAWD_CODEX_MCP_TOKEN"


class _BearerTokenApp:
    """Minimal fixed-token ASGI guard for a process-local MCP endpoint."""

    def __init__(self, app, token: str) -> None:
        self._app = app
        self._expected = f"Bearer {token}".encode()

    async def __call__(self, scope, receive, send) -> None:
        if scope["type"] == "http":
            headers = {k.lower(): v for k, v in scope.get("headers", ())}
            supplied = headers.get(b"authorization", b"")
            if not secrets.compare_digest(supplied, self._expected):
                response = PlainTextResponse(
                    "unauthorized", status_code=401,
                    headers={"WWW-Authenticate": "Bearer"})
                await response(scope, receive, send)
                return
        await self._app(scope, receive, send)


def _low_level_server(specs: Sequence[ToolSpec]) -> Server:
    server = Server("pilot", version="1")
    by_name = {spec.name: spec for spec in specs}

    @server.list_tools()
    async def list_tools() -> list[mcp_types.Tool]:
        return [mcp_types.Tool(
            name=spec.name,
            description=spec.description,
            inputSchema=spec.input_schema,
        ) for spec in specs]

    @server.call_tool()
    async def call_tool(name: str, arguments: dict[str, Any]):
        spec = by_name.get(name)
        if spec is None:  # allowlist/list_tools should make this unreachable
            return mcp_types.CallToolResult(
                content=[mcp_types.TextContent(
                    type="text", text=f"INVALID_PARAM: unknown tool {name}")],
                isError=True)
        result = await spec.handler(arguments)
        content = [mcp_types.TextContent.model_validate(block)
                   for block in result.get("content", [])]
        return mcp_types.CallToolResult(
            content=content, isError=bool(result.get("is_error", False)))

    return server


class PilotMCPServer:
    """One stateless MCP server bound to an ephemeral loopback-only port."""

    def __init__(self, specs: Sequence[ToolSpec], *, token: str | None = None,
                 startup_timeout_s: float = 10.0) -> None:
        self.specs = tuple(specs)
        self.token = token or secrets.token_urlsafe(32)
        self.startup_timeout_s = startup_timeout_s
        self.url: str | None = None
        self._server: uvicorn.Server | None = None
        self._task: asyncio.Task | None = None
        self._socket: socket.socket | None = None

    async def __aenter__(self):
        low_level = _low_level_server(self.specs)
        security = TransportSecuritySettings(
            enable_dns_rebinding_protection=True,
            allowed_hosts=["127.0.0.1:*", "localhost:*"],
            allowed_origins=["http://127.0.0.1:*", "http://localhost:*"],
        )
        manager = StreamableHTTPSessionManager(
            app=low_level, json_response=True, stateless=True,
            security_settings=security)
        endpoint = _BearerTokenApp(StreamableHTTPASGIApp(manager), self.token)
        app = Starlette(
            routes=[Route("/mcp", endpoint=endpoint)],
            lifespan=lambda _app: manager.run())

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", 0))
        sock.listen(128)
        sock.setblocking(False)
        port = sock.getsockname()[1]
        self._socket = sock
        self.url = f"http://127.0.0.1:{port}/mcp"
        self._server = uvicorn.Server(uvicorn.Config(
            app, log_level="warning", lifespan="on", access_log=False))
        self._task = asyncio.create_task(
            self._server.serve(sockets=[sock]), name="pilot-codex-mcp")

        deadline = time.monotonic() + self.startup_timeout_s
        while not self._server.started:
            if self._task.done():
                await self._task
                raise RuntimeError("Codex pilot MCP server exited during startup")
            if time.monotonic() >= deadline:
                await self.__aexit__(None, None, None)
                raise RuntimeError("Codex pilot MCP server startup timed out")
            await asyncio.sleep(0.01)
        return self

    async def __aexit__(self, _exc_type, _exc, _tb):
        if self._server is not None:
            self._server.should_exit = True
        if self._task is not None:
            try:
                await asyncio.wait_for(self._task, timeout=5.0)
            except asyncio.TimeoutError:  # pragma: no cover - uvicorn guardrail
                self._task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await self._task
        if self._socket is not None:
            with contextlib.suppress(OSError):
                self._socket.close()
        self._server = None
        self._task = None
        self._socket = None
        return False


@dataclass(frozen=True, slots=True)
class CodexOptions:
    model: str
    effort: str
    system_prompt: str
    tools: tuple[ToolSpec, ...]
    codex_home: str
    workdir: str | None = None

    @property
    def allowed_tools(self) -> list[str]:
        return [f"mcp__pilot__{spec.name}" for spec in self.tools]


def _usage_dict(notification: ThreadTokenUsageUpdatedNotification) -> dict[str, int]:
    return notification.token_usage.last.model_dump(by_alias=False)


def _item(value):
    return value.root if hasattr(value, "root") else value


def normalize_codex_notification(
        notification: Notification, *, model: str) -> list[Any]:
    """Translate one Codex SDK notification into zero or more seam events."""
    # Local import avoids a backend.py -> codex_backend.py import cycle.
    from agents.flight.backend import Text, ToolCall, ToolResult

    payload = notification.payload
    if isinstance(payload, ItemStartedNotification):
        item = _item(payload.item)
        if isinstance(item, McpToolCallThreadItem):
            arguments = item.arguments if isinstance(item.arguments, dict) else {}
            return [ToolCall(
                id=item.id, name=f"mcp__{item.server}__{item.tool}",
                input=arguments, model=model)]
    if isinstance(payload, ItemCompletedNotification):
        item = _item(payload.item)
        if isinstance(item, AgentMessageThreadItem) and item.text.strip():
            return [Text(text=item.text, model=model)]
        if isinstance(item, McpToolCallThreadItem):
            content = []
            if item.result is not None:
                content = [block.model_dump(by_alias=True, exclude_none=True)
                           if hasattr(block, "model_dump") else block
                           for block in item.result.content]
            if item.error is not None and not content:
                content = item.error.message
            return [ToolResult(
                tool_use_id=item.id, content=content,
                is_error=(item.status == McpToolCallStatus.failed or
                          item.error is not None))]
    return []


class CodexBackendClient:
    """Persistent Codex thread implementing the existing backend contract."""

    def __init__(
        self,
        tools: Sequence[ToolSpec],
        *,
        system_prompt: str,
        model: str = DEFAULT_CODEX_MODEL,
        effort: str = DEFAULT_CODEX_EFFORT,
        codex_home: str | None = None,
        workdir: str | None = None,
        sdk_client=None,
        mcp_server_factory: Callable[..., PilotMCPServer] = PilotMCPServer,
        require_auth: bool = True,
    ) -> None:
        if effort not in {e.value for e in ReasoningEffort}:
            raise ValueError(f"invalid Codex reasoning effort: {effort!r}")
        resolved_home = codex_home or os.environ.get(
            "CODEX_HOME", str(Path.home() / ".codex"))
        self.options = CodexOptions(
            model=model, effort=effort, system_prompt=system_prompt,
            tools=tuple(tools), codex_home=resolved_home, workdir=workdir)
        self._injected_sdk = sdk_client
        self._mcp_server_factory = mcp_server_factory
        self._require_auth = require_auth
        self._sdk = None
        self._thread = None
        self._mcp = None
        self._tmpdir: tempfile.TemporaryDirectory | None = None
        self.queries = 0
        self.inference_requests = 0
        self.quota_errors = 0

    async def __aenter__(self):
        auth_file = Path(self.options.codex_home) / "auth.json"
        if self._require_auth and self._injected_sdk is None and not auth_file.is_file():
            raise RuntimeError(
                f"Codex backend requires logged-in subscription credentials at "
                f"{auth_file}; run `codex login` on the host first")

        if self.options.workdir is None:
            self._tmpdir = tempfile.TemporaryDirectory(prefix="squawd-codex-")
            workdir = self._tmpdir.name
        else:
            workdir = self.options.workdir
            Path(workdir).mkdir(parents=True, exist_ok=True)

        self._mcp = self._mcp_server_factory(self.options.tools)
        await self._mcp.__aenter__()
        try:
            if self._injected_sdk is not None:
                self._sdk = self._injected_sdk
            else:
                enabled = json.dumps([spec.name for spec in self.options.tools])
                overrides = (
                    f"mcp_servers.pilot.url={json.dumps(self._mcp.url)}",
                    f"mcp_servers.pilot.bearer_token_env_var={json.dumps(_MCP_TOKEN_ENV)}",
                    "mcp_servers.pilot.required=true",
                    f"mcp_servers.pilot.enabled_tools={enabled}",
                    'mcp_servers.pilot.default_tools_approval_mode="approve"',
                    'web_search="disabled"',
                    "tools.web_search=false",
                    "tools.view_image=false",
                    "features.shell_tool=false",
                    "features.unified_exec=false",
                    "features.skill_mcp_dependency_install=false",
                    'shell_environment_policy.inherit="none"',
                    'history.persistence="none"',
                    "project_doc_max_bytes=0",
                )
                sdk_config = CodexConfig(
                    config_overrides=overrides,
                    cwd=workdir,
                    env={
                        "CODEX_HOME": self.options.codex_home,
                        _MCP_TOKEN_ENV: self._mcp.token,
                    },
                )
                self._sdk = AsyncCodex(sdk_config)
            await self._sdk.__aenter__()
            self._thread = await self._sdk.thread_start(
                approval_mode=ApprovalMode.deny_all,
                base_instructions=self.options.system_prompt,
                cwd=workdir,
                ephemeral=True,
                model=self.options.model,
                sandbox=Sandbox.read_only,
            )
        except BaseException:
            await self._cleanup(None, None, None)
            raise
        return self

    async def _cleanup(self, exc_type, exc, tb):
        if self._sdk is not None:
            with contextlib.suppress(Exception):
                await self._sdk.__aexit__(exc_type, exc, tb)
        if self._mcp is not None:
            with contextlib.suppress(Exception):
                await self._mcp.__aexit__(exc_type, exc, tb)
        if self._tmpdir is not None:
            self._tmpdir.cleanup()
        self._thread = None
        self._sdk = None
        self._mcp = None
        self._tmpdir = None

    async def __aexit__(self, exc_type, exc, tb):
        await self._cleanup(exc_type, exc, tb)
        return False

    async def query(self, prompt: str) -> AsyncIterator[Any]:
        from agents.flight.backend import Result, is_quota_error

        if self._thread is None:
            raise RuntimeError("CodexBackendClient must be entered before query()")
        self.queries += 1
        t0 = time.monotonic()
        arrivals: list[float] = []
        usage: dict | None = None
        n_infer = 0
        n_quota = 0
        completed = None
        try:
            handle = await self._thread.turn(
                prompt,
                approval_mode=ApprovalMode.deny_all,
                effort=ReasoningEffort(self.options.effort),
                model=self.options.model,
                sandbox=Sandbox.read_only,
            )
            async for notification in handle.stream():
                arrivals.append(time.monotonic())
                payload = notification.payload
                if isinstance(payload, ThreadTokenUsageUpdatedNotification):
                    usage = _usage_dict(payload)
                elif isinstance(payload, ItemCompletedNotification):
                    if isinstance(_item(payload.item), AgentMessageThreadItem):
                        n_infer += 1
                elif isinstance(payload, ErrorNotification):
                    if is_quota_error(str(payload)):
                        n_quota += 1
                elif isinstance(payload, TurnCompletedNotification):
                    completed = payload.turn
                    if completed.error is not None and is_quota_error(
                            completed.error.message):
                        n_quota += 1
                for event in normalize_codex_notification(
                        notification, model=self.options.model):
                    yield event

            if completed is None:
                raise RuntimeError("Codex turn completed event not received")
            gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
            is_error = completed.status == TurnStatus.failed
            yield Result(
                usage=usage,
                cost_usd=None,
                num_turns=1,
                api_ms=completed.duration_ms,
                is_error=is_error,
                inference_requests=n_infer,
                quota_errors=n_quota,
                ttfa_s=round(arrivals[0] - t0, 3) if arrivals else None,
                gap_p50_s=(round(statistics.median(gaps), 3) if gaps else 0.0),
                wall_ms=int((time.monotonic() - t0) * 1000),
            )
        except Exception as exc:
            if is_quota_error(str(exc)):
                n_quota += 1
            raise
        finally:
            self.inference_requests += n_infer
            self.quota_errors += n_quota
