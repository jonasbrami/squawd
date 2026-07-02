"""Sweep grid: cross product of {tasks} x {model assignments} x K repeats, plus
resume-skip. Pure (no sim) so the schedule is testable and a killed sweep can be
restarted, skipping cells already in results.jsonl."""
import random
from dataclasses import dataclass

from evals.runner import assignment_label


@dataclass(frozen=True)
class Cell:
    task_id: str
    assignment: dict
    repeat: int

    def key(self) -> str:
        return f"{self.task_id}|{assignment_label(self.assignment)}|{self.repeat}"


def expand(task_ids: list[str], assignments: list[dict], k: int) -> list[Cell]:
    return [Cell(t, a, r)
            for t in task_ids
            for a in assignments
            for r in range(k)]


def shuffled(cells: list[Cell], seed: int) -> list[Cell]:
    """Deterministic shuffle of the run order. expand() nests repeats innermost, so
    unshuffled sweeps run all K repeats of a cell back-to-back and all tiers of a
    task consecutively — sim/EKF drift and API warm/cold effects get confounded
    with cell identity. Resume is unaffected (done-keys are order-independent)."""
    out = list(cells)
    random.Random(seed).shuffle(out)
    return out


def done_keys(rows: list[dict]) -> set[str]:
    return {f"{r['task_id']}|{r['assignment']}|{r['repeat']}" for r in rows}
