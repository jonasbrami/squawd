"""Pure parsers for the host-sampler tool outputs, plus limiting-resource
attribution. No subprocess calls here — Task 6's CLI does the shelling out and
hands these the captured text/objects.
"""
import json
import re


def parse_nvidia_smi(line: str) -> dict:
    """One `nvidia-smi --query-gpu=utilization.gpu,memory.used,power.draw
    --format=csv,noheader,nounits` line, e.g. '42, 1536, 78.50'."""
    parts = [p.strip() for p in line.split(",")]
    util, mem, power = (parts + ["0", "0", "0"])[:3]
    return {"util": float(util), "mem_used_mb": float(mem), "power_w": float(power)}


def parse_intel_gpu_top(obj) -> dict:
    """One `intel_gpu_top -J` sample object (dict or JSON string). Engine keys
    vary by kernel ('Render/3D', 'Render/3D/0', 'Video/0', …); match by prefix.
    'VideoEnhance' is NOT the video engine."""
    if isinstance(obj, str):
        obj = json.loads(obj)
    engines = obj.get("engines", {})

    def busy(prefix: str, exclude: str | None = None) -> float:
        for name, e in engines.items():
            if name.startswith(prefix) and (exclude is None or not name.startswith(exclude)):
                return float(e.get("busy", 0.0))
        return 0.0

    return {"render_pct": busy("Render"), "video_pct": busy("Video", exclude="VideoEnhance")}


def parse_docker_stats(line: str) -> dict:
    """`docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}'`,
    e.g. '231.40% 4.5GiB / 31GiB'."""
    m = re.match(r"\s*([\d.]+)%\s+([\d.]+)\s*([KMGT]?i?B)", line)
    if not m:
        return {"cpu_pct": 0.0, "mem_mb": 0.0}
    cpu = float(m.group(1))
    val = float(m.group(2))
    unit = m.group(3).rstrip("B").rstrip("i")
    scale = {"K": 1 / 1024, "M": 1.0, "G": 1024.0, "T": 1024.0 * 1024, "": 1 / (1024 * 1024)}.get(unit, 1.0)
    return {"cpu_pct": cpu, "mem_mb": val * scale}


def parse_gz_rtf(text: str) -> float:
    """Extract real_time_factor from a `gz topic -e -t /world/<w>/stats` dump."""
    m = re.search(r"real_time_factor:\s*([\d.]+)", text)
    return float(m.group(1)) if m else 0.0


def limiting_resource(sample: dict, *, vram_total_mb: float = 8192,
                      ram_total_gb: float = 31.0, threshold: float = 0.6) -> str:
    """Which resource is closest to saturation. Returns 'none' if the most-loaded
    resource is below `threshold` of its limit (i.e. nothing is actually the
    bottleneck — the FAIL was elsewhere)."""
    nv = sample.get("nvidia") or {}
    ig = sample.get("intel") or {}
    norm = {
        "cpu": sample.get("cpu_pct", 0.0) / 100.0,
        "ram": sample.get("ram_used_gb", 0.0) / ram_total_gb,
        "dgpu": nv.get("util", 0.0) / 100.0,
        "vram": nv.get("mem_used_mb", 0.0) / vram_total_mb,
        "igpu": max(ig.get("render_pct", 0.0), ig.get("video_pct", 0.0)) / 100.0,
    }
    key = max(norm, key=norm.get)
    return key if norm[key] >= threshold else "none"
