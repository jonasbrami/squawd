"""Aggregate eval result rows into per-cell success-rate + latency, render Markdown.

Pure (no sim): reads the results.jsonl rows produced by run_evals. infra_fail rows are
excluded from the success denominator (they're harness noise, not task outcomes) so the
accuracy numbers stay honest. Answers: complexity limit (success vs task), model
trade-off (success + latency per assignment)."""
from collections import defaultdict
from dataclasses import dataclass


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    k = (len(s) - 1) * pct
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


@dataclass
class CellAgg:
    task_id: str
    assignment: str
    k: int
    successes: int
    success_rate: float
    lat_p50: float
    lat_p95: float
    mean_steps: float
    failure_breakdown: dict[str, int]


def aggregate(rows: list[dict]) -> list[CellAgg]:
    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in rows:
        groups[(r["task_id"], r["assignment"])].append(r)

    out: list[CellAgg] = []
    for (task_id, assignment), grp in sorted(groups.items()):
        scored = [r for r in grp if not r.get("infra_fail")]
        k = len(scored)
        successes = sum(1 for r in scored if r.get("passed"))
        lats = [r["latency_s"] for r in scored if r.get("latency_s") is not None]
        steps = [r["steps"] for r in scored]
        fails: dict[str, int] = defaultdict(int)
        for r in scored:
            if not r.get("passed"):
                fails[r.get("failure_reason") or "oracle check failed"] += 1
        out.append(CellAgg(
            task_id=task_id, assignment=assignment, k=k, successes=successes,
            success_rate=(successes / k if k else 0.0),
            lat_p50=_percentile(lats, 0.5), lat_p95=_percentile(lats, 0.95),
            mean_steps=(sum(steps) / k if k else 0.0),
            failure_breakdown=dict(fails)))
    return out


def render_markdown(aggs: list[CellAgg]) -> str:
    lines = ["# Agent Task-Eval Results", "",
             "| task | assignment | k | success_rate | lat_p50 | lat_p95 | mean_steps | failures |",
             "|------|-----------|---|--------------|---------|---------|------------|----------|"]
    for a in aggs:
        fb = ", ".join(f"{kk}×{vv}" for kk, vv in a.failure_breakdown.items()) or "-"
        lines.append(f"| {a.task_id} | {a.assignment} | {a.k} | {a.success_rate:.0%} | "
                     f"{a.lat_p50:.1f}s | {a.lat_p95:.1f}s | {a.mean_steps:.1f} | {fb} |")
    return "\n".join(lines) + "\n"


def render_ladders(rows: list[dict]) -> str:
    """Per-suite pivot: success-rate by rung (difficulty[suite]) x assignment — the knee view.
    Skips infra_fail rows and rows without a suite."""
    suites: dict[str, dict] = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    assigns: set[str] = set()
    for r in rows:
        if r.get("infra_fail"):
            continue
        suite = r.get("suite")
        if not suite:
            continue
        rung = (r.get("difficulty") or {}).get(suite, 0)
        a = r["assignment"]
        assigns.add(a)
        suites[suite][rung][a].append(bool(r.get("passed")))

    cols = sorted(assigns)
    lines = ["# Ladders (success-rate by rung x tier)"]
    for suite in sorted(suites):
        lines += ["", f"## {suite} ladder", "",
                  "| rung | " + " | ".join(cols) + " |",
                  "|------|" + "|".join(["------"] * len(cols)) + "|"]
        for rung in sorted(suites[suite]):
            cells = []
            for a in cols:
                res = suites[suite][rung].get(a, [])
                cells.append(f"{100.0 * sum(res) / len(res):.0f}%" if res else "-")
            lines.append(f"| {rung} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"
