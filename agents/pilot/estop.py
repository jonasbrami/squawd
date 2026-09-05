"""Estop arbiter (ICD §7.1): one authoritative interruption path.

The ActiveToolRegistry records the currently running MCP tool task. The
supervisor watches /pilot/estop and, on a line, cancels that task (the agent
TURN survives to receive the ESTOPPED result code — option A, Fable-MAJOR-1),
awaits its cleanup under asyncio.shield, then drives the emergency action
through the SAME FlightOps instance the tools use. A generation counter blocks
a cancelled controller from ever resuming stale setpoints.

std_msgs and the bus are imported lazily inside the supervisor (same pattern
as core/gzposes.py) so the registry is unit-testable without ROS.
"""
import asyncio

from agents.core.store import TopicLog


class ActiveToolRegistry:
    """One writer per tool call (the tool wrapper), one reader (the supervisor)."""

    def __init__(self) -> None:
        self._task: asyncio.Task | None = None
        self._generation = 0

    def register(self, task: asyncio.Task) -> int:
        self._task = task
        self._generation += 1
        return self._generation

    def clear(self, generation: int | None = None) -> None:
        """No-arg: unconditional (the original tool-wrapper contract). With
        the generation captured at register(), a STALE writer's completion
        cannot clear a newer owner's slot (the preemption race, W0.4)."""
        if generation is not None and generation != self._generation:
            return
        self._task = None

    @property
    def generation(self) -> int:
        return self._generation

    async def cancel_current(self) -> bool:
        t, self._task = self._task, None
        if t is None or t.done():
            return False
        self._generation += 1            # invalidate any stale writer
        t.cancel()
        try:
            await asyncio.shield(asyncio.wait([t], timeout=5))
        except Exception:
            pass
        return True


async def estop_supervisor(bridge, registry: ActiveToolRegistry, ops, *,
                           msg_type=None, cmd_qos=None, chat_qos=None) -> None:
    """Independent asyncio task: /pilot/estop line -> cancel tool, then
    emergency_hold() (default) or emergency_land() (line == 'land').
    std_msgs/bus are resolved lazily at runtime; tests inject fakes."""
    if msg_type is None or cmd_qos is None or chat_qos is None:
        from std_msgs.msg import String as _S
        from agents.core.bus import CHAT_QOS as _CHAT, CMD_QOS as _CMD
        msg_type = _S if msg_type is None else msg_type
        cmd_qos = _CMD if cmd_qos is None else cmd_qos
        chat_qos = _CHAT if chat_qos is None else chat_qos
    log = TopicLog(bridge, "/pilot/estop", msg_type, cmd_qos)
    seen = len(log.all())                    # never act on latched history
    while True:
        await asyncio.sleep(0.2)
        new, seen = log.since(seen)
        for line in new:
            action = line.strip().lower() or "hold"
            cancelled = await registry.cancel_current()
            if action == "land":
                msg = await asyncio.shield(ops.emergency_land())
            else:
                msg = await asyncio.shield(ops.emergency_hold())
            m = msg_type()
            m.data = f"estop: {msg} (tool cancelled: {cancelled})"
            bridge.publish("/pilot/chat", msg_type, m, chat_qos)
