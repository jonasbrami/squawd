"""Pure benchmark math: delivered-FPS from frame-counter deltas, and the
PASS/FAIL verdict. No ROS, Docker, or subprocess imports — unit-tested directly.
"""


def compute_fps(seq_start: dict[int, int], seq_end: dict[int, int], dt: float) -> dict[int, float]:
    """Per-drone delivered FPS = (end_seq - start_seq) / dt. dt<=0 -> 0.0."""
    out: dict[int, float] = {}
    for i, s0 in seq_start.items():
        s1 = seq_end.get(i, s0)
        out[i] = (s1 - s0) / dt if dt > 0 else 0.0
    return out


def _percentile(values: list[float], pct: float) -> float:
    """Linear-interpolated percentile (pct in 0..100). Empty -> 0.0."""
    if not values:
        return 0.0
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    rank = (pct / 100.0) * (len(xs) - 1)
    lo = int(rank)
    frac = rank - lo
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * frac


def fps_summary(fps: dict[int, float]) -> dict:
    vals = list(fps.values())
    if not vals:
        return {"min": 0.0, "mean": 0.0, "p10": 0.0}
    return {
        "min": min(vals),
        "mean": sum(vals) / len(vals),
        "p10": _percentile(vals, 10.0),
    }


def evaluate_verdict(fps_min: float, cam_fps: float, rtf: float, alive: int, n: int,
                     *, fps_frac: float = 0.90, rtf_min: float = 0.90) -> dict:
    """PASS iff min-FPS >= fps_frac*cam_fps AND rtf >= rtf_min AND alive == n."""
    reasons: list[str] = []
    fps_bar = fps_frac * cam_fps
    if fps_min < fps_bar:
        reasons.append(f"fps {fps_min:.2f} < bar {fps_bar:.2f}")
    if rtf < rtf_min:
        reasons.append(f"rtf {rtf:.2f} < {rtf_min:.2f}")
    if alive != n:
        reasons.append(f"alive {alive}/{n}")
    return {"pass": not reasons, "reasons": reasons}
