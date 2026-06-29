"""Sweep grid: cross product of {tasks} x {model assignments} x K repeats, plus
resume-skip. Pure (no sim) so the schedule is testable and a killed sweep can be
restarted, skipping cells already in results.jsonl."""
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


def done_keys(rows: list[dict]) -> set[str]:
    return {f"{r['task_id']}|{r['assignment']}|{r['repeat']}" for r in rows}
