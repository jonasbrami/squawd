"""Typed tool failures (ICD §9). The tool layer maps these to result codes IN
ORDER; every code here has a producible path (test asserted in tests/test_tools_errors.py).

Result dictionary shapes (produced in tools.py):
  success  = {"content": [{"type": "text", "text": str}]}
  failure  = success-shape + "is_error": True
  LOST     = success-shape (degraded completion, not an error)
"""


class ToolFailure(Exception):
    """A failure with a stable, machine-readable code the LLM may reason on."""

    def __init__(self, code: str, text: str) -> None:
        super().__init__(text)
        self.code = code
        self.text = text


class InvalidParamError(ToolFailure):
    def __init__(self, text: str) -> None:
        super().__init__("INVALID_PARAM", text)


class NotReadyError(ToolFailure):
    def __init__(self, text: str) -> None:
        super().__init__("NOT_READY", text)


class BlockedError(ToolFailure):
    def __init__(self, text: str) -> None:
        super().__init__("BLOCKED", text)


# Result codes produced without an exception (kept in sync with tools.py):
#   LOST      — track O2: a RETURN value, degraded completion (is_error=False)
#   TIMEOUT   — asyncio.TimeoutError (run_mission)
#   ESTOPPED  — asyncio.CancelledError in a tool wrapper (estop arbiter)
#   INTERNAL  — any other Exception (logged with traceback server-side)
