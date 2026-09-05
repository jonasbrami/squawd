"""M4 item 4 — look()/pinpoint() END-TO-END latency (tool entry -> text
result), warm, n=20 each, through the pilot's real path: the M2 deep tools
(make_deep_tools) over the injected GzCameras frame source and the env
DeepClient (DEEP_PERCEPTION_URL — the :8101 tap, exactly what the live
pilot uses). world/bridge/pipeline are None: they only gate the optional
ground_intersection suffix (cheap dict lookups), not the sidecar path.

Runs: 20x look("truck") [cached vocab], 20x pinpoint(320,180) [SAM],
then 6x look with ALTERNATING fresh vocabularies (set_classes re-embed
cost note). BUSY texts (slowlane tick collision) are retried, counted,
and excluded from the percentiles.

Run INSIDE pilot-sim:
  cd /workspace && PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project \
      python evals/out/deep_m4/run_latency.py
"""
import json
import os
import statistics
import time

from agents.core.camera import GzCameras
from agents.perception.deep_client import DeepClient
from agents.pilot.deep_tools import make_deep_tools

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "latency_results.json")


def pct(xs, p):
    xs = sorted(xs)
    return round(xs[min(len(xs) - 1, max(0, round(p / 100 * (len(xs) - 1))))],
                 1)


def run(fn, n=20, gap=1.2):
    """-> (times_ms, busy_count, first_text). Times are tool-entry->text."""
    ts, busy, first = [], 0, None
    for _ in range(n):
        t0 = time.monotonic()
        text = fn()
        dt = (time.monotonic() - t0) * 1000.0
        if text.startswith("BUSY"):
            busy += 1
            time.sleep(0.5)
            continue
        if first is None:
            first = text
        ts.append(round(dt, 1))
        time.sleep(gap)
    return ts, busy, first


def main():
    cameras = GzCameras(1)
    client = DeepClient()           # env URL (the tap) + token, like the pilot
    look, pinpoint = make_deep_tools(None, None, None,
                                     lambda: cameras.snapshot(0), client)
    # wait for the first frame, then warm both paths
    for _ in range(50):
        if cameras.snapshot(0) is not None:
            break
        time.sleep(0.2)
    look("truck")
    pinpoint(320, 180)

    look_ts, look_busy, look_first = run(lambda: look("truck"))
    pin_ts, pin_busy, pin_first = run(lambda: pinpoint(320, 180))

    fresh = []
    vocabs = ["bicycle", "bus", "motorcycle", "fire hydrant", "bench",
              "traffic light"]
    for v in vocabs:                # every call a NEW vocab -> re-embed rides
        t0 = time.monotonic()
        text = look(v)
        dt = (time.monotonic() - t0) * 1000.0
        if not text.startswith("BUSY"):
            fresh.append(round(dt, 1))
        time.sleep(0.8)

    out = {
        "look_cached_ms": look_ts, "look_p50": pct(look_ts, 50),
        "look_p95": pct(look_ts, 95), "look_busy": look_busy,
        "pinpoint_ms": pin_ts, "pinpoint_p50": pct(pin_ts, 50),
        "pinpoint_p95": pct(pin_ts, 95), "pinpoint_busy": pin_busy,
        "look_fresh_vocab_ms": fresh,
        "look_fresh_p50": pct(fresh, 50) if fresh else None,
        "look_first_text": look_first, "pinpoint_first_text": pin_first,
        "note": "tool entry->text, n=20 each, via DEEP_PERCEPTION_URL (tap)",
    }
    with open(OUT, "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
