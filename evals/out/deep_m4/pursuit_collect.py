#!/usr/bin/env python3
"""M4 item 5 — fast-lane regression + load collector during ONE pursuit
scenario (HOST side). Same metrics as the M3 A/B collector (gz RTF, cockpit
/state detector.latency_ms + cam cadence, PX4 time-jump delta, tap sidecar
wire latencies, VRAM, slowlane counters) PLUS host load average — sampled
over a ~2 min window while a headless click-lock pursuit runs (w3_click).

Usage: python3 evals/out/deep_m4/pursuit_collect.py <label> <duration_s>
Writes evals/out/deep_m4/pursuit_<label>.jsonl + _summary.json.
"""
import json
import re
import statistics
import subprocess
import sys
import time
import urllib.request

OUT = "evals/out/deep_m4"
TAP = "evals/out/deep_m2/tap.log"
SAMPLE_DT = 10.0


def sh(cmd, timeout=30):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                       timeout=timeout)
    return r.stdout


def gz_rtf():
    out = sh("docker exec pilot-sim bash -lc "
             "'timeout 8 gz topic -t /world/demo/stats -e -n 1 2>/dev/null'",
             timeout=20)
    m = re.search(r"real_time_factor:\s*([0-9.]+)", out)
    return float(m.group(1)) if m else None


def cockpit_state():
    try:
        with urllib.request.urlopen("http://localhost:8000/state",
                                    timeout=3) as r:
            return json.load(r)
    except Exception:
        return None


def px4_time_jumps():
    out = sh("docker exec pilot-sim bash -lc "
             "'grep -c \"time jump detected\" /tmp/px4_0.log'")
    try:
        return int(out.strip())
    except ValueError:
        return None


def loadavg():
    with open("/proc/loadavg") as f:
        return float(f.read().split()[0])


def vram_mb():
    out = sh("nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits")
    try:
        return int(out.strip())
    except ValueError:
        return None


def tap_offset():
    try:
        with open(TAP) as f:
            return len(f.read())
    except OSError:
        return 0


def tap_latencies_since(offset):
    det, seg = [], []
    with open(TAP) as f:
        f.seek(offset)
        for line in f:
            m = re.search(r"POST (/v1/\w+) .* -> 200 (\d+)ms", line)
            if not m:
                continue
            (det if m.group(1) == "/v1/detect" else seg).append(int(m.group(2)))
    return det, seg


def pct(xs, p):
    if not xs:
        return None
    xs = sorted(xs)
    return xs[min(len(xs) - 1, max(0, round((p / 100) * (len(xs) - 1))))]


def main():
    label, duration = sys.argv[1], float(sys.argv[2])
    t0 = time.monotonic()
    start_jumps, start_tap = px4_time_jumps(), tap_offset()
    start_vram = vram_mb()
    rows = []
    prev = None
    while time.monotonic() - t0 < duration:
        t = time.monotonic() - t0
        st = cockpit_state()
        rtf = gz_rtf()
        row = {"t": round(t, 1), "rtf": rtf, "load1": loadavg()}
        if st:
            cam_seq = st.get("cam_seq") or 0
            cam_hz = None
            if prev and prev.get("cam_seq") is not None:
                dt = t - prev["t"]
                cam_hz = round((cam_seq - prev["cam_seq"]) / dt, 2) \
                    if dt > 0 else None
            tr = st.get("track") or {}
            row.update({
                "cam_seq": cam_seq, "cam_hz": cam_hz,
                "det_ms": (st.get("detector") or {}).get("latency_ms"),
                "det_ok": (st.get("detector") or {}).get("healthy"),
                "armed": st.get("armed"), "mode": st.get("mode"),
                "track_state": tr.get("state"), "track_target": tr.get("target"),
                "annotations": len(st.get("annotations") or []),
                "slowlane_ticks": (st.get("slowlane") or {}).get("ticks"),
                "slowlane_calls": (st.get("slowlane") or {}).get("calls"),
            })
            prev = {"t": t, "cam_seq": cam_seq}
        rows.append(row)
        print(f"[{label} {t:6.0f}s] rtf={rtf} det_ms={row.get('det_ms')} "
              f"cam_hz={row.get('cam_hz')} load={row.get('load1')} "
              f"track={row.get('track_state')}", flush=True)
        time.sleep(SAMPLE_DT)
    mins = (time.monotonic() - t0) / 60.0
    end_jumps, end_vram = px4_time_jumps(), vram_mb()
    det_lat, seg_lat = tap_latencies_since(start_tap)

    rtfs = [r["rtf"] for r in rows if r.get("rtf") is not None]
    dets = [r["det_ms"] for r in rows if r.get("det_ms") is not None]
    cams = [r["cam_hz"] for r in rows if r.get("cam_hz") is not None]
    loads = [r["load1"] for r in rows if r.get("load1") is not None]
    summary = {
        "label": label, "duration_min": round(mins, 2), "samples": len(rows),
        "rtf_mean": round(statistics.mean(rtfs), 3) if rtfs else None,
        "rtf_min": round(min(rtfs), 3) if rtfs else None,
        "det_ms_mean": round(statistics.mean(dets), 1) if dets else None,
        "det_ms_p95": pct(dets, 95),
        "cam_hz_mean": round(statistics.mean(cams), 2) if cams else None,
        "load1_mean": round(statistics.mean(loads), 1) if loads else None,
        "px4_time_jumps": (end_jumps - start_jumps)
        if None not in (start_jumps, end_jumps) else None,
        "px4_jumps_per_min": round((end_jumps - start_jumps) / mins, 2)
        if None not in (start_jumps, end_jumps) else None,
        "sidecar_detect_n": len(det_lat),
        "sidecar_detect_p50": pct(det_lat, 50),
        "sidecar_detect_p95": pct(det_lat, 95),
        "sidecar_segment_n": len(seg_lat),
        "vram_start_mb": start_vram, "vram_end_mb": end_vram,
        "slowlane_ticks_delta": (rows[-1].get("slowlane_ticks") or 0)
        - (rows[0].get("slowlane_ticks") or 0) if rows else None,
        "slowlane_calls_delta": (rows[-1].get("slowlane_calls") or 0)
        - (rows[0].get("slowlane_calls") or 0) if rows else None,
    }
    base = f"{OUT}/pursuit_{label}"
    with open(base + ".jsonl", "w") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
    with open(base + "_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
