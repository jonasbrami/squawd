"""W2 visual proof: server-side cockpit-overlay render (design §4 milestone
W2: "overlay screenshot"). No browser in the container — this draws exactly
what the cockpit overlay draws (boxes + labels from dets, masks decoded from
the WIRE base64 RLE) on the live gz camera frame, honoring the overlay
freshness rule (|frame_stamp - snap_stamp| <= 0.5 s, observatory/overlay.py).

Run INSIDE the container (ROS env sourced, perception harness running):
  PYTHONPATH=/workspace uv run --no-project \
      python evals/out/w2_coco_live/overlay_capture.py [n_captures] [out_dir]
"""
import base64
import json
import sys
import time

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
    contacts = snap.get("contacts") or []
    names = ", ".join(f"{c['name']}[{c.get('health', '?')}]" for c in contacts)
    det = snap.get("detector") or {}
    header = (f"sim={snap.get('sim_stamp')} dets={len(snap.get('dets') or [])} "
              f"latency={det.get('latency_ms')}ms healthy={det.get('healthy')}")
    d.rectangle([0, 0, im.width, 26], fill=(0, 0, 0))
    d.text((4, 2), header, fill="#ffffff")
    d.text((4, 14), f"contacts: {names or '(none)'}", fill="#3ddc84")
    im.save(out_path)
    return header, names


def main() -> None:
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "evals/out/w2_coco_live"
    min_dets = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    from std_msgs.msg import String
    bridge = RosBridge(node_name="w2_capture")
    bridge.subscribe("/pilot/detections", String, STATE_QOS)
    cameras = GzCameras(1)
    bridge.start()
    got = 0
    deadline = time.monotonic() + 300
    last_seq = -1
    while got < n and time.monotonic() < deadline:
        msg = bridge.latest("/pilot/detections")
        f = cameras.snapshot(0)
        if msg is None or f is None or f.seq == last_seq:
            time.sleep(0.5)
            continue
        snap = json.loads(msg.data)
        if not overlay_fresh(f.sim_stamp, snap):
            time.sleep(0.5)
            continue
        if len(snap.get("dets") or []) < min_dets:
            time.sleep(1.0)                # hunting a detection frame
            continue
        last_seq = f.seq
        path = f"{out_dir}/overlay_{got}.png"
        header, names = draw(f, snap, path)
        print(f"[{got}] {path}: {header}", flush=True)
        print(f"    {names}", flush=True)
        got += 1
        time.sleep(6)                      # spread captures over mover loops
    if got == 0:
        raise SystemExit("no fresh frame+snapshot pair captured")


if __name__ == "__main__":
    main()
