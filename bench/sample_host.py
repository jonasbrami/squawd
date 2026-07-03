"""~1 Hz host-resource sampler. Shells out to nvidia-smi / intel_gpu_top /
docker stats, parses with bench.probes, and appends JSONL. Runs on the HOST
(GPUs are host devices), alongside the benchmarked container.
"""
import argparse
import json
import subprocess
import time

from bench import probes

# Module-level state for /proc/stat CPU delta (Fix 1b)
_cpu_stat_prev: tuple[float, float] | None = None  # (total, idle)


def _run(cmd: list[str], timeout: float = 4.0) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except Exception:
        return ""


def _read_proc_stat_cpu() -> tuple[float, float] | None:
    """Return (total, idle) from the first 'cpu' line of /proc/stat, or None."""
    try:
        with open("/proc/stat") as f:
            for line in f:
                if line.startswith("cpu "):
                    fields = line.split()[1:]  # drop 'cpu' label
                    vals = [float(v) for v in fields]
                    total = sum(vals)
                    idle = vals[3] + (vals[4] if len(vals) > 4 else 0.0)  # idle + iowait
                    return total, idle
    except Exception:
        pass
    return None


def _cpu_ram() -> dict:
    global _cpu_stat_prev
    try:
        import psutil
        return {"cpu_pct": psutil.cpu_percent(interval=None),
                "load1": psutil.getloadavg()[0],
                "ram_used_gb": (psutil.virtual_memory().total - psutil.virtual_memory().available) / 1e9}
    except Exception:
        # /proc fallback for load + mem; cpu_pct computed from /proc/stat delta (Fix 1b).
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

        cpu_pct = 0.0
        cur = _read_proc_stat_cpu()
        if cur is not None:
            if _cpu_stat_prev is not None:
                delta_total = cur[0] - _cpu_stat_prev[0]
                delta_idle = cur[1] - _cpu_stat_prev[1]
                if delta_total > 0:
                    cpu_pct = 100.0 * (delta_total - delta_idle) / delta_total
            _cpu_stat_prev = cur

        return {"cpu_pct": cpu_pct, "load1": load1, "ram_used_gb": used}


def _nvidia() -> dict:
    try:
        out = _run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,power.draw",
                    "--format=csv,noheader,nounits"])
        line = out.strip().splitlines()[0] if out.strip() else ""
        return probes.parse_nvidia_smi(line) if line else {}
    except Exception:
        return {}


def _intel() -> dict:
    # one ~500ms sample; intel_gpu_top -J streams a JSON array, take the first object.
    try:
        out = _run(["sudo", "-n", "intel_gpu_top", "-J", "-s", "500", "-o", "-"], timeout=4.0)
        out = out.strip().lstrip("[").strip()
        if not out:
            return {}
        # Use raw_decode to extract the first complete JSON object (Fix 3).
        decoder = json.JSONDecoder()
        obj, _ = decoder.raw_decode(out)
        return probes.parse_intel_gpu_top(obj)
    except Exception:
        return {}


def _container(name: str) -> dict:
    try:
        out = _run(["docker", "stats", "--no-stream", "--format", "{{.CPUPerc}} {{.MemUsage}}", name])
        line = out.strip().splitlines()[0] if out.strip() else ""
        return probes.parse_docker_stats(line) if line else {}
    except Exception:
        return {}


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
            t0 = time.monotonic()  # Fix 4: measure iteration elapsed time
            sample = {"t": time.time(), **_cpu_ram(),
                      "nvidia": _nvidia(), "intel": _intel(),
                      "container": _container(args.container)}
            f.write(json.dumps(sample) + "\n")
            f.flush()
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, args.interval - elapsed))  # Fix 4: honor --interval


if __name__ == "__main__":
    main()
