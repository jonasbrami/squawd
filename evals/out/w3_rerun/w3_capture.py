"""W3-integration one-shot cockpit-overlay capture (in-container).

Same render path as W2 (evals/out/w2_coco_live/overlay_capture.py): draws
exactly what the cockpit overlay draws — COCO boxes + wire-RLE masks on the
live gz frame, freshness-gated (|frame_stamp - snap_stamp| <= 0.5 s) — plus
the W3 additions the demo is about: beam/track state in the header and a
reticle on the tracked contact. One shot per invocation, saved as
evals/out/w3_integration/<tag>.png. --require locked|contact waits (bounded)
for that condition before grabbing the frame.

Run INSIDE the container:
  PYTHONPATH=/workspace uv run --no-project python \
      evals/out/w3_integration/w3_capture.py <tag> [--require locked] \
      [--wait 60]
"""
import argparse
import base64
import json
import os
import time

from PIL import Image, ImageDraw

from agents.core.bus import STATE_QOS, RosBridge
from agents.core.camera import GzCameras
from agents.observatory.overlay import overlay_fresh
from agents.vision.types import rle_decode

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PALETTE = ("#3ddc84", "#ff5252", "#40c4ff", "#ffd740", "#ff6e40", "#e040fb",
           "#64ffda", "#ffff64")


def color_for(cls, cache={}):
    if cls not in cache:
        cache[cls] = PALETTE[len(cache) % len(PALETTE)]
    return cache[cls]


def draw(frame, snap, out_path):
    im = Image.frombytes("RGB", (frame.width, frame.height), frame.rgb)
    overlay = Image.new("RGBA", im.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
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
            overlay.paste(tint, (int(x1), int(y1)), mask_im)
        d.rectangle([x1, y1, x2, y2], outline=col, width=2)
        d.text((x1 + 2, max(0, y1 - 10)),
               f"{det['cls']} {det['conf']:.2f}", fill=col)
    im = Image.alpha_composite(im.convert("RGBA"), overlay).convert("RGB")
    d = ImageDraw.Draw(im)
    track, beam = snap.get("track") or {}, snap.get("beam") or {}
    tgt = track.get("target")
    for c in snap.get("contacts") or []:
        if c.get("name") != tgt or not c.get("bbox_xyxy"):
            continue
        x1, y1, x2, y2 = c["bbox_xyxy"]          # corner-bracket reticle
        L = max(6, int(0.25 * min(x2 - x1, y2 - y1)))
        for cx, cy, dx, dy in ((x1, y1, 1, 1), (x2, y1, -1, 1),
                               (x1, y2, 1, -1), (x2, y2, -1, -1)):
            d.line([cx, cy, cx + dx * L, cy], fill="#ffffff", width=3)
            d.line([cx, cy, cx, cy + dy * L], fill="#ffffff", width=3)
    contacts = snap.get("contacts") or []
    names = ", ".join(f"{c['name']}[{c.get('health', '?')}]" for c in contacts)
    det = snap.get("detector") or {}
    rng = beam.get("range_m")
    header = (f"sim={snap.get('sim_stamp')} dets={len(snap.get('dets') or [])} "
              f"latency={det.get('latency_ms')}ms "
              f"track={track.get('state')}/{tgt} gap={track.get('gap_m')} "
              f"beam={beam.get('status')} range={rng}")
    d.rectangle([0, 0, im.width, 26], fill=(0, 0, 0))
    d.text((4, 2), header, fill="#ffffff")
    d.text((4, 14), f"contacts: {names or '(none)'}", fill="#3ddc84")
    im.save(out_path)
    return header, names


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--require", choices=["locked", "contact"], default=None)
    ap.add_argument("--wait", type=float, default=60.0)
    args = ap.parse_args()

    from std_msgs.msg import String
    bridge = RosBridge(node_name="w3_capture")
    bridge.subscribe("/pilot/detections", String, STATE_QOS)
    cameras = GzCameras(1)
    bridge.start()

    deadline = time.monotonic() + args.wait
    last_seq = -1
    while time.monotonic() < deadline:
        msg = bridge.latest("/pilot/detections")
        f = cameras.snapshot(0)
        if msg is None or f is None or f.seq == last_seq:
            time.sleep(0.3)
            continue
        snap = json.loads(msg.data)
        if not overlay_fresh(f.sim_stamp, snap):
            time.sleep(0.3)
            continue
        if args.require == "locked" and \
                (snap.get("beam") or {}).get("status") != "LOCKED":
            time.sleep(0.5)
            continue
        if args.require == "contact" and not (snap.get("contacts") or []):
            time.sleep(0.5)
            continue
        last_seq = f.seq
        path = os.path.join(OUT_DIR, f"{args.tag}.png")
        header, names = draw(f, snap, path)
        print(f"{path}: {header}", flush=True)
        print(f"    {names}", flush=True)
        return
    raise SystemExit(f"no fresh frame meeting --require {args.require!r} "
                     f"within {args.wait:.0f}s")


if __name__ == "__main__":
    main()
