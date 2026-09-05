"""strategy_ab — strategy-snippet A/B infrastructure (design §13 item 6, §7 M5).

A snippet is a hand-written, version-controlled markdown file in
agents/pilot/strategies/<name>.md, validated against the real tool registry.
It is appended to the pilot system prompt ONLY for cells whose assignment
names it — `--assignments "drones=sonnet;drones=sonnet,strategy=intercept-lead"`
produces paired base/snippet cells in one sweep (assignment_label carries the
key, so the lanes stay distinct in results.jsonl).

ACTIVATION GATE: a snippet activates ONLY on measured lift — its Wilson 95%
lower bound must beat the base lane's point success rate, with enough scored
cells on BOTH sides. Runtime self-generation/self-activation stays REJECTED;
this module is observational and changes nothing by itself.
"""
from pathlib import Path

from evals.report import wilson_ci

STRATEGIES_DIR = Path(__file__).resolve().parents[1] / "agents" / "pilot" / "strategies"

# The registry snippets are validated against (mirrors flight/tools.py
# _pilot_server's 12 M1 tools + detect, §0.6 — kept local so evals stays
# import-light; the tool-surface contract test pins the real list).
PILOT_TOOL_NAMES = frozenset({
    "take_off", "fly", "goto", "orbit", "hover", "set_speed", "face", "land",
    "report", "scan", "run_mission", "track", "detect",
})

MIN_K = 3   # below 3 scored cells per lane a Wilson interval bounds nothing


class StrategyError(Exception):
    pass


def load_snippet(name: str, strategies_dir: Path | None = None) -> str:
    """Read + validate a snippet. Raises StrategyError on a missing file or a
    backticked tool reference the real registry doesn't know (a snippet that
    hallucinates tools would corrupt the A/B it rides on)."""
    if not name or "/" in name or name.startswith("."):
        raise StrategyError(f"invalid strategy name {name!r}")
    d = Path(strategies_dir) if strategies_dir else STRATEGIES_DIR
    path = d / f"{name}.md"
    if not path.is_file():
        raise StrategyError(f"no strategy snippet at {path}")
    text = path.read_text().strip()
    if not text:
        raise StrategyError(f"strategy snippet {path} is empty")
    bad = _unknown_tools(text)
    if bad:
        raise StrategyError(
            f"strategy {name!r} references tools outside the pilot registry: "
            f"{sorted(bad)} (have {sorted(PILOT_TOOL_NAMES)})")
    return text


def _unknown_tools(text: str) -> set:
    """Backticked tokens that look like tool calls but aren't pilot tools."""
    import re
    out = set()
    for tok in re.findall(r"`([a-z_]+)\(", text):
        if tok not in PILOT_TOOL_NAMES:
            out.add(tok)
    return out


def scored(rows: list[dict]) -> tuple[int, int]:
    """(successes, k) over non-infra rows — the only honest denominator."""
    sc = [r for r in rows if not r.get("infra_fail")]
    return sum(1 for r in sc if r.get("passed")), len(sc)


def lift_decision(base_rows: list[dict], snip_rows: list[dict],
                  min_k: int = MIN_K) -> dict:
    """The activation gate. activate=True ONLY when the snippet lane's Wilson
    95% lower bound exceeds the base lane's point success rate AND both lanes
    have >= min_k scored cells — 'measured lift', never vibes."""
    bs, bk = scored(base_rows)
    ss, sk = scored(snip_rows)
    base_rate = bs / bk if bk else 0.0
    snip_rate = ss / sk if sk else 0.0
    lo, hi = wilson_ci(ss, sk)
    out = {"activate": False, "base": {"successes": bs, "k": bk,
                                       "rate": base_rate},
           "snippet": {"successes": ss, "k": sk, "rate": snip_rate,
                       "ci_lo": lo, "ci_hi": hi}}
    if bk < min_k or sk < min_k:
        out["reason"] = (f"insufficient data (base k={bk}, snippet k={sk}; "
                         f"need >= {min_k} each)")
        return out
    if lo > base_rate:
        out["activate"] = True
        out["reason"] = (f"measured lift: snippet {ss}/{sk} [{lo:.0%}-{hi:.0%}] "
                         f"beats base {bs}/{bk} ({base_rate:.0%}) at CI-low")
    else:
        out["reason"] = (f"no measured lift: snippet CI-low {lo:.0%} does not "
                         f"beat base rate {base_rate:.0%} — stays inactive")
    return out
