"""Aggregate eval result rows into per-cell statistics, render Markdown.

Pure (no sim): reads the results.jsonl / transcripts.jsonl rows produced by
run_evals. infra_fail rows are excluded from the success denominator (they're
harness noise, not task outcomes) so the accuracy numbers stay honest.

Statistics discipline (K is small and binomial):
- every success rate carries a Wilson 95% interval — at K=3 a bare percentage is
  noise (0/3 vs 3/3 is Fisher p=0.10), and a knee you can't bound isn't localized;
- no tail percentiles at tiny K (a p95 over 3 samples is an interpolation next to
  the max) — first-action latency is reported as median + range;
- steps are conditioned on success (a failed run's step count means "budget burned",
  not efficiency);
- gcs = goal-condition success, the mean fraction of oracle checks passed — graded
  signal near the knee where all-or-nothing pass/fail saturates."""
import math
from collections import Counter, defaultdict
from dataclasses import dataclass, field


def wilson_ci(successes: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score 95% interval for a binomial proportion. (0, 0) -> (0, 1)."""
    if n == 0:
        return (0.0, 1.0)
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    k = (len(s) - 1) * pct
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def _gcs(row: dict) -> float:
    """Fraction of oracle checks passed; falls back to the pass bit for old rows."""
    checks = row.get("checks")
    if not checks:
        return 1.0 if row.get("passed") else 0.0
    return sum(1 for c in checks if c.get("passed")) / len(checks)


@dataclass
class CellAgg:
    task_id: str
    assignment: str
    k: int
    successes: int
    success_rate: float
    ci_lo: float
    ci_hi: float
    gcs: float
    ttfa_p50: float            # time to first tool call (was 'latency'); model-side
    ttfa_min: float
    ttfa_max: float
    steps_to_success: float | None   # mean steps over PASSING repeats (None if none)
    steps_on_fail: float | None
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
        pass_steps = [r["steps"] for r in scored if r.get("passed")]
        fail_steps = [r["steps"] for r in scored if not r.get("passed")]
        fails: dict[str, int] = defaultdict(int)
        for r in scored:
            if not r.get("passed"):
                fails[r.get("failure_reason") or "oracle check failed"] += 1
        lo, hi = wilson_ci(successes, k)
        out.append(CellAgg(
            task_id=task_id, assignment=assignment, k=k, successes=successes,
            success_rate=(successes / k if k else 0.0), ci_lo=lo, ci_hi=hi,
            gcs=(sum(_gcs(r) for r in scored) / k if k else 0.0),
            ttfa_p50=_percentile(lats, 0.5),
            ttfa_min=min(lats, default=0.0), ttfa_max=max(lats, default=0.0),
            steps_to_success=(sum(pass_steps) / len(pass_steps) if pass_steps else None),
            steps_on_fail=(sum(fail_steps) / len(fail_steps) if fail_steps else None),
            failure_breakdown=dict(fails)))
    return out


def _rate_ci(successes: int, k: int, lo: float, hi: float) -> str:
    return f"{successes}/{k} [{lo:.0%}–{hi:.0%}]"


def render_markdown(aggs: list[CellAgg]) -> str:
    lines = ["# Agent Task-Eval Results", "",
             "success_rate carries its Wilson 95% interval; gcs = mean fraction of "
             "oracle checks passed; ttfa = time to first tool call (model-side); "
             "steps✓ = mean steps over passing repeats only.", "",
             "| task | assignment | success_rate | gcs | ttfa_p50 (range) | steps✓ | steps✗ | failures |",
             "|------|-----------|--------------|-----|------------------|--------|--------|----------|"]
    for a in aggs:
        fb = ", ".join(f"{kk}×{vv}" for kk, vv in a.failure_breakdown.items()) or "-"
        s_ok = f"{a.steps_to_success:.1f}" if a.steps_to_success is not None else "-"
        s_ko = f"{a.steps_on_fail:.1f}" if a.steps_on_fail is not None else "-"
        lines.append(
            f"| {a.task_id} | {a.assignment} | {_rate_ci(a.successes, a.k, a.ci_lo, a.ci_hi)} | "
            f"{a.gcs:.0%} | {a.ttfa_p50:.1f}s ({a.ttfa_min:.1f}–{a.ttfa_max:.1f}) | "
            f"{s_ok} | {s_ko} | {fb} |")
    return "\n".join(lines) + "\n"


def render_ladders(rows: list[dict]) -> str:
    """Per-suite pivot: success by rung (difficulty[suite]) x assignment — the knee
    view, each cell bounded by its Wilson interval. Skips infra_fail rows and rows
    without a suite."""
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
    lines = ["# Ladders (success by rung x tier, Wilson 95%)"]
    for suite in sorted(suites):
        lines += ["", f"## {suite} ladder", "",
                  "| rung | " + " | ".join(cols) + " |",
                  "|------|" + "|".join(["------"] * len(cols)) + "|"]
        for rung in sorted(suites[suite]):
            cells = []
            for a in cols:
                res = suites[suite][rung].get(a, [])
                if not res:
                    cells.append("-")
                    continue
                n, k = sum(res), len(res)
                lo, hi = wilson_ci(n, k)
                cells.append(f"{100.0 * n / k:.0f}% [{lo:.0%}–{hi:.0%}]")
            lines.append(f"| {rung} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


# ---- transcript-derived tool metrics --------------------------------------------

GOTO_BURST_WINDOW_S = 5.0   # a goto issued this soon after the previous goto was
                            # overriding an in-flight move (pre-blocking-goto data)


@dataclass
class ToolAgg:
    task_id: str
    assignment: str
    cells: int
    tool_mix: dict[str, int] = field(default_factory=dict)
    goto_burst: int = 0        # max per-cell count of rapid-fire goto follow-ups
    gap_p50: float = 0.0       # median seconds between consecutive tool calls
    out_tokens: float = 0.0    # median output tokens per cell
    cost_usd: float = 0.0      # summed across cells


def _tool_name(ev: dict) -> str:
    return ev.get("name", "").rsplit("__", 1)[-1]


def _cell_burst(calls: list[dict]) -> int:
    gotos = [c["t"] for c in calls if _tool_name(c) == "goto"]
    return sum(1 for a, b in zip(gotos, gotos[1:]) if b - a < GOTO_BURST_WINDOW_S)


def aggregate_transcripts(trows: list[dict]) -> list[ToolAgg]:
    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in trows:
        groups[(r["task_id"], r["assignment"])].append(r)

    out: list[ToolAgg] = []
    for (task_id, assignment), grp in sorted(groups.items()):
        mix: Counter = Counter()
        bursts, gaps, tokens, cost = [], [], [], 0.0
        for r in grp:
            calls = [e for e in r.get("events", []) if e.get("type") == "tool_call"]
            mix.update(_tool_name(c) for c in calls)
            bursts.append(_cell_burst(calls))
            gaps += [b["t"] - a["t"] for a, b in zip(calls, calls[1:])]
            if (r.get("usage") or {}).get("output_tokens") is not None:
                tokens.append(r["usage"]["output_tokens"])
            cost += r.get("cost_usd") or 0.0
        out.append(ToolAgg(
            task_id=task_id, assignment=assignment, cells=len(grp),
            tool_mix=dict(mix), goto_burst=max(bursts, default=0),
            gap_p50=_percentile(gaps, 0.5),
            out_tokens=_percentile(tokens, 0.5), cost_usd=cost))
    return out


def render_tools(aggs: list[ToolAgg]) -> str:
    lines = ["# Tool usage by cell (from transcripts)", "",
             "burst = max per-cell count of gotos issued <5s after the previous goto "
             "(each one overrode an in-flight move on pre-blocking-goto data); "
             "gap_p50 = median seconds between tool calls (patience).", "",
             "| task | assignment | cells | tool mix | burst | gap_p50 | out_tok_p50 | cost |",
             "|------|-----------|-------|----------|-------|---------|-------------|------|"]
    for a in aggs:
        mix = ", ".join(f"{k}×{v}" for k, v in sorted(a.tool_mix.items(),
                                                      key=lambda kv: -kv[1])) or "-"
        lines.append(f"| {a.task_id} | {a.assignment} | {a.cells} | {mix} | "
                     f"{a.goto_burst} | {a.gap_p50:.1f}s | {a.out_tokens:.0f} | "
                     f"${a.cost_usd:.2f} |")
    return "\n".join(lines) + "\n"
