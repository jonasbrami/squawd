"""Agent task-eval orchestrator (single-drone layer).

For each (task x model-assignment x repeat) cell: soft-reset the world, run the drone
agent on the task under its budget, grade the sampled WorldTrack, append a row to
results.jsonl (infra failures retried once, never scored as task fails). Then render
RESULTS.md. Runs SEQUENTIALLY against ONE already-running sim (launch it first with
sim/launch/swarm_sim.sh, SWARM_N=1) — parallel sims would confound the latency metric.

Usage:
  # 1) bring up a single-drone sim in the swarm container (separate shell)
  # 2) inside that container/venv:
  python -m evals.run_evals \\
      --tasks evals/tasks/reach_marker_single.yaml \\
      --assignments "drones=opus;drones=haiku" \\
      --k 5

Note: imports of RosBridge, GzCameras, and World are deferred inside main() so that
the module and its pure helpers (parse_assignments, run_with_retry) are importable
without a live ROS2 environment. This matches the pattern established in runner.py
(DroneAgent deferred inside run_cell). At actual runtime (inside the sim container)
all imports resolve normally.
"""
import argparse
import asyncio
import json
import os
import time

from evals.matrix import expand, done_keys, shuffled
from evals.report import aggregate, render_markdown
from evals.runner import Deps, DroneHarness, run_cell
from evals.spec import load_task


def parse_assignments(spec_str: str) -> list[dict]:
    out = []
    for chunk in spec_str.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        d = {}
        for pair in chunk.split(","):
            role, _, tier = pair.partition("=")
            d[role.strip()] = tier.strip()
        out.append(d)
    return out


class InfraFuse:
    """Trips after `limit` CONSECUTIVE infra failures (post-retry). A persistently
    sick sim would otherwise convert the rest of a sweep into infra_fail rows —
    abort loudly instead; resume skips the completed cells."""

    def __init__(self, limit: int = 2) -> None:
        self.limit = limit
        self._consecutive = 0

    def update(self, infra_fail: bool) -> bool:
        self._consecutive = self._consecutive + 1 if infra_fail else 0
        return self._consecutive >= self.limit


async def run_with_retry(coro_fn, attempts: int = 2):
    """Await coro_fn() up to `attempts` times; retry only while the result is an
    infra failure. A real PASS/FAIL is returned immediately."""
    result = None
    for _ in range(max(1, attempts)):
        result = await coro_fn()
        if result is None or not result.infra_fail:
            return result
    return result


def _load_rows(path: str) -> list[dict]:
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


async def main(args) -> None:
    # Deferred ROS imports: only resolve inside the sim container where rclpy is available.
    from agents.core.bus import RosBridge
    from agents.core.camera import GzCameras
    from agents.world import World

    specs = {os.path.splitext(os.path.basename(p))[0]: load_task(p) for p in args.tasks}
    if args.pilot:
        # Trap gate: fly each task's declared ideal script with NO LLM through the
        # same run_cell/oracle path. Tasks without a pilot are quarantined loudly.
        missing = [t for t, s in specs.items() if not s.pilot]
        for t in missing:
            print(f"pilot: SKIP {t} (no pilot script declared)", flush=True)
        specs = {t: s for t, s in specs.items() if s.pilot}
        assignments = [{"drones": "pilot"}]
    else:
        assignments = parse_assignments(args.assignments)

    out_dir = args.out or os.path.join(
        os.path.dirname(__file__), "out", time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    jsonl = os.path.join(out_dir, "results.jsonl")
    tjsonl = os.path.join(out_dir, "transcripts.jsonl")

    done = done_keys(_load_rows(jsonl))
    cells = shuffled(expand(list(specs), assignments, args.k), seed=args.seed)

    bridge = RosBridge(node_name="evals_runner")
    world = World()
    n_max = max(s.setup.n_drones for s in specs.values())
    cameras = GzCameras(n_max)
    gzposes = None
    if world.movers:
        from agents.core.gzposes import GzPoses
        gz_world = os.environ.get("GZ_WORLD") or os.environ.get("PX4_GZ_WORLD") or "dynamic"
        gzposes = GzPoses(gz_world, [m["name"] for m in world.movers])
    bridge.start()
    deps = Deps(world=world, bridge=bridge, cameras=cameras, gzposes=gzposes)
    harness = DroneHarness(deps)  # shared flight link, fresh Claude client per cell
    if args.pilot:
        from evals.pilot import pilot_client_builder
        harness._client_builder = pilot_client_builder(harness, deps)
    try:
        print(f"evals: {len(cells)} cells (order seed {args.seed}) -> {jsonl}", flush=True)
        fuse = InfraFuse(limit=2)
        # results.jsonl is the resume index; transcripts.jsonl is written in the same
        # iteration keyed by the same triple, so readers join on it (last wins).
        with open(jsonl, "a") as fh, open(tjsonl, "a") as tfh:
            for cell in cells:
                if cell.key() in done:
                    print(f"skip (done): {cell.key()}", flush=True)
                    continue
                spec = specs[cell.task_id]
                if args.pilot:
                    harness.pilot_script = spec.pilot  # per-cell script for the builder
                res = await run_with_retry(
                    lambda c=cell, s=spec: run_cell(s, c.assignment, c.repeat, deps, harness))
                fh.write(json.dumps(res.to_row()) + "\n")
                fh.flush()
                tfh.write(json.dumps(res.to_transcript_row()) + "\n")
                tfh.flush()
                lat = f"{res.latency_s:.1f}s" if res.latency_s is not None else "n/a"
                print(f"{cell.key()}: passed={res.passed} infra_fail={res.infra_fail} "
                      f"steps={res.steps} lat={lat}", flush=True)
                if fuse.update(res.infra_fail):
                    print("ABORT: 2 consecutive infra failures — the sim is likely "
                          "sick. Fix it and re-run the same command (resume skips "
                          "completed cells).", flush=True)
                    break

        rows = _load_rows(jsonl)
        md = render_markdown(aggregate(rows))
        with open(os.path.join(out_dir, "RESULTS.md"), "w") as f:
            f.write(md)
        print(md, flush=True)

        from evals.report import render_ladders
        with open(os.path.join(out_dir, "LADDERS.md"), "w") as f:
            f.write(render_ladders(rows))

        trows = _load_rows(tjsonl)
        if trows:
            from evals.report import aggregate_transcripts, render_tools
            with open(os.path.join(out_dir, "TOOLS.md"), "w") as f:
                f.write(render_tools(aggregate_transcripts(trows)))
    finally:
        bridge.shutdown()


def _cli() -> None:
    ap = argparse.ArgumentParser(description="Agent task-eval harness (single-drone).")
    ap.add_argument("--tasks", nargs="+", required=True, help="task YAML paths")
    ap.add_argument("--assignments", default="drones=opus",
                    help="';'-separated role=tier groups, e.g. 'drones=opus;drones=haiku'")
    ap.add_argument("--k", type=int, default=5, help="repeats per cell")
    ap.add_argument("--out", default=None, help="output dir (default evals/out/<ts>)")
    ap.add_argument("--seed", type=int, default=0,
                    help="cell-order shuffle seed (logged; resume-safe)")
    ap.add_argument("--pilot", action="store_true",
                    help="fly each task's declared ideal script with NO LLM (trap "
                         "gate): a task the pilot can't pass is a harness bug")
    args = ap.parse_args()
    asyncio.run(main(args))


if __name__ == "__main__":
    _cli()
