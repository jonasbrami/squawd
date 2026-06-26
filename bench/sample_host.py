"""~1 Hz host-resource sampler. Shells out to nvidia-smi / intel_gpu_top /
docker stats, parses with bench.probes, and appends JSONL. Runs on the HOST
(GPUs are host devices), alongside the benchmarked container.
"""
import argparse
import json
import subprocess
import time

from bench import probes


def _run(cmd: list[str], timeout: float = 4.0) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except Exception:
        return ""


def _cpu_ram() -> dict:
    try:
        import psutil
        return {"cpu_pct": psutil.cpu_percent(interval=None),
                "load1": psutil.getloadavg()[0],
                "ram_used_gb": (psutil.virtual_memory().total - psutil.virtual_memory().available) / 1e9}
    except Exception:
        # /proc fallback for load + mem; cpu_pct best-effort 0.
        load1 = 0.0
        try:
            with open("/proc/loadavg") as f:
                load1 = float(f.read().split()[0])
        except Exception:
            pass
        used = 0.0
        try:
            mem = {}
            with open("/proc/meminfo") as f:
                for line in f:
                    k, v = line.split(":")
                    mem[k] = float(v.strip().split()[0])  # kB
            used = (mem.get("MemTotal", 0) - mem.get("MemAvailable", 0)) / 1e6
        except Exception:
            pass
        return {"cpu_pct": 0.0, "load1": load1, "ram_used_gb": used}


def _nvidia() -> dict:
    out = _run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,power.draw",
                "--format=csv,noheader,nounits"])
    line = out.strip().splitlines()[0] if out.strip() else ""
    return probes.parse_nvidia_smi(line) if line else {}


def _intel() -> dict:
    # one ~500ms sample; intel_gpu_top -J streams a JSON array, take the first object.
    out = _run(["sudo", "-n", "intel_gpu_top", "-J", "-s", "500", "-o", "-"], timeout=4.0)
    out = out.strip().lstrip("[").strip()
    if not out:
        return {}
    # take text up to the first top-level closing brace
    depth = 0
    end = -1
    for idx, ch in enumerate(out):
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                end = idx + 1
                break
    if end == -1:
        return {}
    try:
        return probes.parse_intel_gpu_top(out[:end])
    except Exception:
        return {}


def _container(name: str) -> dict:
    out = _run(["docker", "stats", "--no-stream", "--format", "{{.CPUPerc}} {{.MemUsage}}", name])
    line = out.strip().splitlines()[0] if out.strip() else ""
    return probes.parse_docker_stats(line) if line else {}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--container", default="swarm-multi")
    ap.add_argument("--interval", type=float, default=1.0)
    args = ap.parse_args()
    try:
        import psutil
        psutil.cpu_percent(interval=None)  # prime the first delta
    except Exception:
        pass
    with open(args.out, "a") as f:
        while True:
            sample = {"t": time.time(), **_cpu_ram(),
                      "nvidia": _nvidia(), "intel": _intel(),
                      "container": _container(args.container)}
            f.write(json.dumps(sample) + "\n")
            f.flush()
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
