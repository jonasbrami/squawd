"""Capacity-frontier benchmark orchestrator. For each (backend, resolution)
cell, ramps N to the knee via bench.sweep, driving the existing Docker swarm
stack one run at a time, and writes per-run JSON + a frontier table.

A run: clean container -> bring up (run_swarm_demo.sh with RENDER_BACKEND/CAM_*)
-> wait sim-ready -> send one warm-up command -> settle -> measure FPS+RTF+liveness
+ host headroom -> verdict -> teardown.
"""
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

from bench import metrics, probes, sweep, frontier

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_RES = ["320x180", "640x360", "960x540", "1280x720", "1920x1080"]
DEFAULT_BACKENDS = ["cpu", "intel", "nvidia"]
WARMUP = "everyone take off, climb to 20 m, and orbit"
WORLD = "baylands"
OBS = "http://localhost:8000"


# ---- pure helpers (unit-tested) ----
def slice_samples(samples: list[dict], t0: float, t1: float) -> list[dict]:
    return [s for s in samples if t0 <= s.get("t", 0.0) < t1]


def _load(s: dict) -> float:
    nv = s.get("nvidia") or {}
    return max(s.get("cpu_pct", 0.0), nv.get("util", 0.0),
              (s.get("intel") or {}).get("render_pct", 0.0))


def peak_sample(samples: list[dict]) -> dict:
    return max(samples, key=_load) if samples else {}


# ---- glue ----
def _sh(cmd: str, timeout: float = 60.0) -> str:
    return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                          timeout=timeout, cwd=ROOT).stdout


def _state() -> dict | None:
    try:
        with urllib.request.urlopen(OBS + "/state", timeout=5) as r:
            return json.loads(r.read())
    except Exception:
        return None


def _wait_ready(n: int, deadline: float) -> bool:
    """Sim + observatory ready: /state returns n drones with telemetry + cam_seq>0."""
    while time.time() < deadline:
        st = _state()
        if st and st.get("n") == n:
            ds = st["drones"]
            if all(d.get("cam_seq", 0) > 0 for d in ds):
                return True
        time.sleep(5)
    return False


def _command(text: str) -> None:
    data = json.dumps({"text": text}).encode()
    try:
        urllib.request.urlopen(
            urllib.request.Request(OBS + "/command", data=data,
                                   headers={"Content-Type": "application/json"}),
            timeout=5)
    except Exception:
        pass


def _rtf() -> float:
    out = _sh(f"docker exec swarm-multi bash -lc "
              f"'gz topic -e -t /world/{WORLD}/stats -n 1' 2>/dev/null", timeout=20)
    return probes.parse_gz_rtf(out)


def run_one(backend: str, res: str, n: int, settle: float, measure: float,
            outdir: str, cam_fps: int = 10) -> dict:
    """Bring up, measure, tear down. Returns a result dict with 'verdict'."""
    w, h = res.split("x")
    _sh("docker rm -f swarm-multi 2>/dev/null || true", timeout=60)
    # pristine camera model so a new CAM_* actually applies (the sed is one-shot)
    _sh("git -C PX4-Autopilot checkout -- "
        "Tools/simulation/gz/models/OakD-Lite/model.sdf 2>/dev/null || true", timeout=30)

    sample_path = os.path.join(outdir, f"samples-{backend}-{res}-{n}.jsonl")
    sampler = subprocess.Popen(
        [sys.executable, "-m", "bench.sample_host", "--out", sample_path],
        cwd=ROOT)
    try:
        env = (f"RENDER_BACKEND={backend} CAM_W={w} CAM_H={h} CAM_FPS={cam_fps} "
               f"WORLD={WORLD}")
        _sh(f"{env} timeout 600 ./scripts/run_swarm_demo.sh {n}", timeout=620)
        if not _wait_ready(n, time.time() + 300):
            return {"backend": backend, "resolution": res, "n": n,
                    "verdict": {"pass": False, "reasons": ["infra: not ready"]},
                    "infra_fail": True}
        _command(WARMUP)
        time.sleep(settle)

        st0 = _state() or {"drones": []}
        seq0 = {d["id"]: d.get("cam_seq", 0) for d in st0["drones"]}
        t0 = time.time()
        time.sleep(measure)
        st1 = _state() or {"drones": []}
        seq1 = {d["id"]: d.get("cam_seq", 0) for d in st1["drones"]}
        dt = time.time() - t0

        fps = metrics.compute_fps(seq0, seq1, dt)
        fsum = metrics.fps_summary(fps)
        rtf = _rtf()
        alive = sum(1 for d in st1["drones"] if d.get("armed") is True)

        if os.path.exists(sample_path):
            with open(sample_path) as fh:
                samples = [json.loads(l) for l in fh]
        else:
            samples = []
        window = slice_samples(samples, t0, t0 + dt)
        peak = peak_sample(window)
        limit = probes.limiting_resource(peak) if peak else "none"

        verdict = metrics.evaluate_verdict(fsum["min"], cam_fps, rtf, alive, n)
        return {"backend": backend, "resolution": res, "n": n,
                "fps": fsum, "rtf": rtf, "alive": alive,
                "limiting": limit, "peak": peak, "verdict": verdict}
    except Exception as e:
        return {"backend": backend, "resolution": res, "n": n,
                "verdict": {"pass": False, "reasons": [f"infra: {type(e).__name__}"]},
                "infra_fail": True}
    finally:
        sampler.terminate()
        try:
            sampler.wait(timeout=5)
        except Exception:
            pass
        _sh("docker rm -f swarm-multi 2>/dev/null || true", timeout=60)


def run_sweep(backends, resolutions, n_cap, settle, measure, outdir, cam_fps=10):
    os.makedirs(outdir, exist_ok=True)
    rows = []
    runs_csv = os.path.join(outdir, "runs.csv")
    with open(runs_csv, "w") as f:
        f.write("backend,resolution,n,fps_min,rtf,alive,limiting,verdict\n")
    for backend in backends:
        seed = 1
        for res in resolutions:                    # ascending res -> capacity non-increasing
            def passes(n, _b=backend, _r=res):
                r = run_one(_b, _r, n, settle, measure, outdir, cam_fps)
                with open(os.path.join(outdir, f"run-{_b}-{_r}-{n}.json"), "w") as jf:
                    json.dump(r, jf, indent=2)
                v = r["verdict"]
                fsum = r.get("fps", {})
                with open(runs_csv, "a") as cf:
                    cf.write(f"{_b},{_r},{n},{fsum.get('min',0):.2f},"
                             f"{r.get('rtf',0):.2f},{r.get('alive',0)},"
                             f"{r.get('limiting','')},{'PASS' if v['pass'] else 'FAIL'}\n")
                return v["pass"]
            knee = sweep.find_knee(passes, n_cap=n_cap, seed=seed)
            seed = max(1, knee)                    # seed next (heavier) resolution from here
            # limiting resource at the knee: re-read the knee run if present
            kr_path = os.path.join(outdir, f"run-{backend}-{res}-{knee}.json")
            limiting = "none"
            if os.path.exists(kr_path):
                with open(kr_path) as fh:
                    limiting = json.load(fh).get("limiting", "none")
            rows.append({"backend": backend, "resolution": res,
                         "knee_n": knee, "limiting": limiting})
    table = frontier.build_frontier_table(rows)
    md = frontier.render_markdown(table, backends, resolutions)
    with open(os.path.join(outdir, "frontier.md"), "w") as f:
        f.write("# Capacity frontier — max sustainable drones\n\n" + md)
    frontier.render_heatmap(table, backends, resolutions,
                            os.path.join(outdir, "frontier.png"))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true",
                    help="fixed N=8, sweep resolutions per backend (harness validation)")
    ap.add_argument("--backends", default=",".join(DEFAULT_BACKENDS))
    ap.add_argument("--resolutions", default=",".join(DEFAULT_RES))
    ap.add_argument("--n-cap", type=int, default=32)
    ap.add_argument("--settle", type=float, default=30.0)
    ap.add_argument("--measure", type=float, default=60.0)
    ap.add_argument("--cam-fps", type=int, default=10)
    ap.add_argument("--out", default=os.path.join("docs", "benchmarks", time.strftime("%Y%m%dT%H%M%S")))
    args = ap.parse_args()
    backends = args.backends.split(",")
    resolutions = args.resolutions.split(",")
    os.makedirs(os.path.join(ROOT, args.out), exist_ok=True)
    outdir = os.path.join(ROOT, args.out)

    if args.smoke:
        # one fixed N across the resolution sweep, no knee search
        for backend in backends:
            for res in resolutions:
                r = run_one(backend, res, 8, args.settle, args.measure, outdir, args.cam_fps)
                with open(os.path.join(outdir, f"smoke-{backend}-{res}.json"), "w") as jf:
                    json.dump(r, jf, indent=2)
                print(f"[smoke] {backend} {res} N=8 -> "
                      f"{'PASS' if r['verdict']['pass'] else 'FAIL'} {r.get('fps',{})}")
        return
    run_sweep(backends, resolutions, args.n_cap, args.settle, args.measure,
              outdir, args.cam_fps)


if __name__ == "__main__":
    main()
