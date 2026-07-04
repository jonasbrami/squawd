"""FleetOps: one object owning every drone's FlightOps, plus the fleet-level
movement primitive.

`goto_all` exists because the per-drone `goto` BLOCKS until arrival: an
operator commanding drones one tool call at a time would serialize the fleet
— the harness, not the model, would forbid coordination (the blocking-goto
lesson at fleet granularity). goto_all issues every move concurrently and
returns when ALL arrive, reporting per-drone outcomes (one drone's error
never hides the others')."""
import asyncio


class FleetOps:
    def __init__(self, ops_list) -> None:
        self._ops = list(ops_list)

    @property
    def n(self) -> int:
        return len(self._ops)

    @staticmethod
    def _coerce_id(i) -> int:
        """Accept the names models actually use: 0, "0", "d0", "drone_0" — the
        tool namespaces are called d0/d1 and scan says drone_1, so an operator
        LLM passing those strings is right, not wrong (observed live: opus's
        goto_all was rejected for '"drone":"d0"' and had to fall back)."""
        if isinstance(i, str):
            s = i.strip().lower()
            for prefix in ("drone_", "drone", "d"):
                if s.startswith(prefix) and s[len(prefix):].isdigit():
                    return int(s[len(prefix):])
        return int(i)

    def drone(self, i):
        try:
            idx = self._coerce_id(i)
        except (TypeError, ValueError):
            raise ValueError(f"unknown drone {i!r} (fleet of {len(self._ops)})")
        if not 0 <= idx < len(self._ops):
            raise ValueError(f"unknown drone {i!r} (fleet of {len(self._ops)})")
        return self._ops[idx]

    async def goto_all(self, moves: list[dict]) -> str:
        tasks = []
        for mv in moves:
            ops = self.drone(mv["drone"])   # validate BEFORE launching any move
            tasks.append((mv["drone"], ops.goto(
                east=mv.get("east"), north=mv.get("north"), up=mv.get("up"),
                wait=True)))
        results = await asyncio.gather(*(t for _, t in tasks),
                                       return_exceptions=True)
        lines = []
        for (i, _), r in zip(tasks, results):
            if isinstance(r, BaseException):
                lines.append(f"drone_{i} ERROR: {r}")
            else:
                lines.append(str(r))
        return "\n".join(lines)
