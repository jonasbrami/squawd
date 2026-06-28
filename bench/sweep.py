"""Capacity-frontier search: exponential-probe-then-bisect for the largest N
that passes, seeded by a neighbouring cell's knee. Assumes pass_fn is monotonic.
"""
from typing import Callable


def find_knee(pass_fn: Callable[[int], bool], n_cap: int, seed: int = 1) -> int:
    cache: dict[int, bool] = {}

    def ok(n: int) -> bool:
        if n not in cache:
            cache[n] = bool(pass_fn(n))
        return cache[n]

    seed = max(1, min(seed, n_cap))

    if not ok(seed):
        # seed fails: find the largest passing N in [1, seed)
        if not ok(1):
            return 0
        lo, hi = 1, seed            # lo passes, hi fails
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return lo

    # seed passes: probe upward by doubling
    last_pass = seed
    while last_pass < n_cap:
        nxt = min(last_pass * 2, n_cap)
        if ok(nxt):
            last_pass = nxt
        else:
            lo, hi = last_pass, nxt  # lo passes, hi fails
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if ok(mid):
                    lo = mid
                else:
                    hi = mid
            return lo
    return last_pass
