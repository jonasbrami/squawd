"""W5 golden-path POV video recorder (in-container): records the gz camera
feed to mp4 with the cockpit-overlay content burned in (COCO boxes + a
track/beam header — exactly what the cockpit overlay draws, freshness-gated
|frame_stamp - snap_stamp| <= 0.5 s; when no fresh snapshot exists the raw
frame is recorded — honest POV, never staged).

Uses imageio-ffmpeg's own writer (no imageio dep). ~10 fps in, 10 fps out.

  PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project \
      --with imageio-ffmpeg --with pillow --with numpy \
      python evals/out/w5_golden/w5_video.py <out.mp4> [seconds]
"""
import base64
import json
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

from agents.core.bus import STATE_QOS, RosBridge
from agents.core.camera import GzCameras
from agents.observatory.overlay import overlay_fresh
from agents.vision.types import rle_decode

PALETTE = ("#3ddc84", "#ff5252", "#40c4ff", "#ffd740", "#ff6e40", "#e040fb",
           "#64ffda", "#ffff64")


def color_for(cls, cache={}):
    if cls not in cache:
        cache[cls] = PALETTE[len(cache) % len(PALETTE)]
    return cache[cls]


def draw(frame, snap):
    im = Image.frombytes("RGB", (frame.width, frame.height), frame.rgb)
    d = ImageDraw.Draw(im)
    for det in snap.get("dets") or []:
        x1, y1, x2, y2 = det["xyxy"]
        col = color_for(det["cls"])
        m = det.get("mask")
        if m:
            rows = rle_decode(base64.b64decode(m["rle"]), m["w"], m["h"])
            w, h = int(x2) - int(x1), int(y2) - int(y1)
            mask_im = Image.new("L", (m["w"], m["h"]), 0)
            mask_im.putdata([255 if v else 0 for row in rows for v in row])
            if (m["w"], m["h"]) != (w, h):
                mask_im = mask_im.resize((max(1, w), max(1, h)))
            tint = Image.new("RGBA", mask_im.size, col + "70")
            im = im.convert("RGBA")
            im.paste(tint, (int(x1), int(y1)), mask_im)
            im = im.convert("RGB")
            d = ImageDraw.Draw(im)
        d.rectangle([x1, y1, x2, y2], outline=col, width=2)
        d.text((x1 + 2, max(0, y1 - 10)),
               f"{det['cls']} {det['conf']:.2f}", fill=col)
    track, beam = snap.get("track") or {}, snap.get("beam") or {}
    tgt = track.get("target")
    for c in snap.get("contacts") or []:
        if c.get("name") != tgt or not c.get("bbox_xyxy"):
            continue
        x1, y1, x2, y2 = c["bbox_xyxy"]
        L = max(6, int(0.25 * min(x2 - x1, y2 - y1)))
        for cx, cy, dx, dy in ((x1, y1, 1, 1), (x2, y1, -1, 1),
                               (x1, y2, 1, -1), (x2, y2, -1, -1)):
            d.line([cx, cy, cx + dx * L, cy], fill="#ffffff", width=3)
            d.line([cx, cy, cx, cy + dy * L], fill="#ffffff", width=3)
    pill = ("RANGE LOCKED" if beam.get("status") == "LOCKED"
            else "VISION LOCK" if tgt else "NO TRACK")
    rng = beam.get("range_m")
    header = (f"{pill} | track={track.get('state')}/{tgt} "
              f"gap={track.get('gap_m')} | beam={beam.get('status')} "
              f"range={rng} | dets={len(snap.get('dets') or [])} "
              f"sim={snap.get('sim_stamp')}")
    d.rectangle([0, 0, im.width, 16], fill=(0, 0, 0))
    d.text((4, 2), header, fill="#ffffff")
    return im


def main() -> None:
    out = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 420.0
    import imageio_ffmpeg
    from std_msgs.msg import String
    bridge = RosBridge(node_name="w5_video")
    bridge.subscribe("/pilot/detections", String, STATE_QOS)
    cameras = GzCameras(1)
    bridge.start()
    # wait for the first frame so dimensions are known
    f = None
    while f is None:
        f = cameras.snapshot(0)
        time.sleep(0.2)
    w, h = f.width, f.height
    writer = imageio_ffmpeg.write_frames(
        out, (w, h), fps=10, codec="libx264", quality=7,
        macro_block_size=2, ffmpeg_log_level="error",
        output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    writer.send(None)
    t0 = time.monotonic()
    n = 0
    last_seq = -1
    while time.monotonic() - t0 < seconds:
        f = cameras.snapshot(0)
        if f is None or f.seq == last_seq:
            time.sleep(0.03)
            continue
        last_seq = f.seq
        msg = bridge.latest("/pilot/detections")
        im = None
        if msg is not None:
            try:
                snap = json.loads(msg.data)
                if overlay_fresh(f.sim_stamp, snap):
                    im = draw(f, snap)
            except Exception:
                im = None
        if im is None:
            im = Image.frombytes("RGB", (w, h), f.rgb)
        writer.send(np.asarray(im))
        n += 1
        if n % 100 == 0:
            print(f"[video] {n} frames ({time.monotonic() - t0:.0f}s)",
                  flush=True)
    writer.close()
    print(f"[video] wrote {n} frames -> {out}", flush=True)


if __name__ == "__main__":
    main()
