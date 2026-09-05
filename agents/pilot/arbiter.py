"""pilot/arbiter.py — the ONE serialized command arbiter (design v0.3, W0.4).

Precedence: estop (latched, highest) > operator lease (UI ops from
/pilot/cmd) > LLM (lowest). The codex warning was that cloned supervisors
around the single ActiveToolRegistry slot would race — so there is exactly
ONE arbiter wrapping THE shared registry: LLM tool wrappers, operator ops
and the estop all register/cancel through the same slot, and ownership can
never race.

- estop(): latch, cancel whatever holds the slot (LLM tool or operator
  lease), then the shielded emergency action through the SAME FlightOps —
  estop_supervisor's exact sequence (ICD §7.1). Stays latched until
  release().
- submit_operator(op): preempt the slot via cancel_current(), take the
  lease, run the op as a registry-tracked task. The lease ends on op
  completion, on an explicit stop+hold, or on the lease timeout (default
  90 s: the op is cancelled and the drone held before release).
- guard_llm(tool): wired into the tool wrapper BEFORE register() (W3a) —
  while the lease is held (or the estop latched) the LLM tool is rejected
  with a structured OPERATOR_ACTIVE (ICD §9, errors.py).

W3a wiring (pilot/cmd.py + pilot/agent.py): /pilot/cmd -> submit_operator,
the tool wrapper -> guard_llm, and run_op -> the real FlightOps locked-object
ops (make_run_op's dispatch map).
"""
import asyncio

from agents.flight.errors import OperatorActiveError


class CommandArbiter:
    OPERATOR_OPS = ("lock", "orbit", "standoff", "stop", "resume")

    def __init__(self, registry, ops, run_op, *, lease_s: float = 90.0):
        """registry: THE shared ActiveToolRegistry (the LLM tool wrappers
        register here too). ops: THE shared FlightOps (only its idempotent
        estop surface is used). run_op: async callable(op: dict) -> str
        executing one operator op (pilot/cmd.py's make_run_op binds the real
        FlightOps here)."""
        self._registry, self._ops, self._run_op = registry, ops, run_op
        self._lease_s = lease_s
        self._lease: asyncio.Task | None = None
        self._estopped = False

    @property
    def estopped(self) -> bool:
        return self._estopped

    @property
    def lease_held(self) -> bool:
        return self._lease is not None and not self._lease.done()

    def guard_llm(self, tool: str = "") -> None:
        """The LLM rejection gate (called BEFORE registry.register())."""
        if self._estopped:
            raise OperatorActiveError(
                "estop latched — the operator must release before the LLM "
                "flies again")
        if self.lease_held:
            raise OperatorActiveError(
                f"operator active — LLM tool {tool!r} rejected")

    async def submit_operator(self, op: dict) -> dict:
        """One UI op from /pilot/cmd: preempt the slot (LLM tool or prior
        operator op), take the lease, run the op registry-tracked."""
        if self._estopped:
            return {"ok": False,
                    "error": "ESTOPPED: estop latched — release first"}
        name = (op or {}).get("op")
        if name not in self.OPERATOR_OPS:
            return {"ok": False,
                    "error": f"INVALID_PARAM: unknown op {name!r}"}
        await self._registry.cancel_current()      # preempts LLM or prior op
        self._lease = asyncio.create_task(self._run_lease(op))
        self._registry.register(self._lease)
        return {"ok": True, "op": name}

    async def estop(self, action: str = "hold") -> str:
        """Estop > all, always: latch FIRST (no submission lands mid-cancel),
        cancel whatever holds the slot, then the shielded emergency action —
        mirroring estop_supervisor's cancel -> shield -> chat text."""
        self._estopped = True
        cancelled = await self._registry.cancel_current()
        if action == "land":
            msg = await asyncio.shield(self._ops.emergency_land())
        else:
            msg = await asyncio.shield(self._ops.emergency_hold())
        return f"estop: {msg} (tool cancelled: {cancelled})"

    def release(self) -> None:
        """The operator's deliberate end of the estop latch (W3: Resume)."""
        self._estopped = False

    async def _run_lease(self, op: dict) -> str:
        try:
            return await asyncio.wait_for(self._run_op(op),
                                          timeout=self._lease_s)
        except asyncio.TimeoutError:
            # lease timeout: wait_for already cancelled the op; hold before
            # releasing — never leave the drone mid-op without setpoints
            await asyncio.shield(self._ops.emergency_hold())
            return "TIMEOUT: operator lease expired"
        finally:
            if self._lease is asyncio.current_task():
                self._lease = None
