"""M3 cockpit-overlay capture (in-container) — draws exactly what the M3
cockpit draws (the w3_capture.py render path): COCO det boxes + wire-RLE
masks, PLUS the deep layer — slowlane annotation boxes (magenta family,
frame-age-gated ≤0.5 s) and the pinpoint mask silhouette (translucent,
box-local RLE) — on the live gz frame, with slowlane/fp state in the header.

  PYTHONPATH=/workspace uv run --no-project python \
      evals/out/deep_m3/m3_capture.py <tag> [--require annotations] [--wait 90]
"""
import argparse
import base64
import json
import os
import time

from PIL import Image, ImageDraw

from agents.core.bus import STATE_QOS, RosBridge
from agents.core.camera import GzCameras
from agents.observatory.overlay import (annotations_for, overlay_fresh,
                                        pinpoint_mask_for)
from agents.vision.types import rle_decode

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PALETTE = ("#3ddc84", "#ff5252", "#40c4ff", "#ffd740", "#ff6e40", "#e040fb",
           "#64ffda", "#ffff64")
ANN_COLOR = "#e040fb"          # magenta family — distinct from det boxes
MASK_RGBA = (76, 195, 255, 90)  # translucent accent silhouette


def color_for(cls, cache={}):
    if cls not in cache:
        cache[cls] = PALETTE[len(cache) % len(PALETTE)]
    return cache[cls]


def draw_mask(im, payload):
    x1, y1, x2, y2 = payload["xyxy"]
    m = payload["mask"]
    rows = rle_decode(base64.b64decode(m["rle"]), m["w"], m["h"])
    mask_im = Image.new("L", (m["w"], m["h"]), 0)
    mask_im.putdata([255 if v else 0 for row in rows for v in row])
    w, h = int(x2) - int(x1), int(y2) - int(y1)
    if (m["w"], m["h"]) != (w, h):
        mask_im = mask_im.resize((max(1, w), max(1, h)))
    tint = Image.new("RGBA", mask_im.size, MASK_RGBA)
    out = Image.alpha_composite(im.convert("RGBA"),
                                Image.new("RGBA", im.size, (0, 0, 0, 0)))
    out.paste(tint, (int(x1), int(y1)), mask_im)
    return out


def draw(frame, snap, slow, deep, out_path):
    im = Image.frombytes("RGB", (frame.width, frame.height), frame.rgb)
    mask = pinpoint_mask_for(deep, frame.sim_stamp)
    if mask:
        im = draw_mask(im, mask)
    im = im.convert("RGB")
    d = ImageDraw.Draw(im)
    for det in snap.get("dets") or []:
        x1, y1, x2, y2 = det["xyxy"]
        col = color_for(det["cls"])
        d.rectangle([x1, y1, x2, y2], outline=col, width=2)
        d.text((x1 + 2, max(0, y1 - 10)),
               f"{det['cls']} {det['conf']:.2f}", fill=col)
    anns = annotations_for(slow, frame.sim_stamp)
    for a in anns:                               # magenta slow-lane boxes
        x1, y1, x2, y2 = a["xyxy"]
        d.rectangle([x1, y1, x2, y2], outline=ANN_COLOR, width=2)
        d.text((x1 + 2, y2 + 2), f"{a['cls']} {a['conf']:.2f} deep",
               fill=ANN_COLOR)
    fps = (slow or {}).get("fp_suspects") or [] if overlay_fresh(
        frame.sim_stamp, slow) else []
    health = (slow or {}).get("health") or {}
    det = snap.get("detector") or {}
    header = (f"sim={snap.get('sim_stamp')} dets={len(snap.get('dets') or [])} "
              f"latency={det.get('latency_ms')}ms | slowlane ann={len(anns)} "
              f"fp_suspects={len(fps)} hz={health.get('hz')} "
              f"note={health.get('note')!r}")
    line2 = (f"ann: " + ", ".join(f"{a['cls']}@{a['conf']:.2f}" for a in anns)
             if anns else "ann: (none fresh)")
    if mask:
        line2 += (f" | mask {mask.get('cls') or 'unlabeled'} "
                  f"area={mask.get('area_px')}px")
    d.rectangle([0, 0, im.width, 26], fill=(0, 0, 0))
    d.text((4, 2), header, fill="#ffffff")
    d.text((4, 14), line2, fill=ANN_COLOR)
    im.save(out_path)
    return header, line2


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--require", choices=["annotations", "mask"], default=None)
    ap.add_argument("--wait", type=float, default=90.0)
    args = ap.parse_args()

    from std_msgs.msg import String
    bridge = RosBridge(node_name="m3_capture")
    bridge.subscribe("/pilot/detections", String, STATE_QOS)
    bridge.subscribe("/pilot/slowlane", String, STATE_QOS)
    bridge.subscribe("/pilot/deep", String, STATE_QOS)
    cameras = GzCameras(1)
    bridge.start()

    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline:
        f = cameras.snapshot(0)
        msg = bridge.latest("/pilot/detections")
        slow_m = bridge.latest("/pilot/slowlane")
        deep_m = bridge.latest("/pilot/deep")
        if f is None or msg is None:
            time.sleep(0.3)
            continue
        snap = json.loads(msg.data)
        if not overlay_fresh(f.sim_stamp, snap):
            time.sleep(0.3)
            continue
        slow = json.loads(slow_m.data) if slow_m else None
        deep = json.loads(deep_m.data) if deep_m else None
        if args.require == "annotations" and \
                not annotations_for(slow, f.sim_stamp):
            time.sleep(0.4)
            continue
        if args.require == "mask" and \
                not pinpoint_mask_for(deep, f.sim_stamp):
            time.sleep(0.4)
            continue
        path = os.path.join(OUT_DIR, f"{args.tag}.png")
        header, line2 = draw(f, snap, slow, deep, path)
        print(f"{path}: {header}", flush=True)
        print(f"    {line2}", flush=True)
        return
    raise SystemExit(f"no fresh frame meeting --require {args.require!r} "
                     f"within {args.wait:.0f}s")


if __name__ == "__main__":
    main()
