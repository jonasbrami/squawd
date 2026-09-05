"""M4 probe (b) evidence: render the latched /pilot/deep pinpoint mask
over the CURRENT gz frame (the drone held the pinpoint viewpoint —
mask frame_seq expired for the live join, so this re-draws it). Payload
read from probe_b_mask_payload.json (captured via ros2 topic echo
--full-length; a fresh subscriber didn't get the latch in time)."""
import base64
import json
import time

from PIL import Image, ImageDraw

from agents.core.camera import GzCameras
from agents.vision.types import rle_decode

p = json.load(open("/workspace/evals/out/deep_m4/probe_b_mask_payload.json"))
cameras = GzCameras(1)
f = None
for _ in range(30):
    f = cameras.snapshot(0)
    if f:
        break
    time.sleep(0.2)
im = Image.frombytes("RGB", (f.width, f.height), f.rgb).convert("RGBA")
x1, y1, x2, y2 = [int(v) for v in p["xyxy"]]
rows = rle_decode(base64.b64decode(p["mask"]["rle"]),
                  p["mask"]["w"], p["mask"]["h"])
m = Image.new("L", (p["mask"]["w"], p["mask"]["h"]), 0)
m.putdata([255 if v else 0 for row in rows for v in row])
im.paste(Image.new("RGBA", m.size, (224, 64, 251, 90)), (x1, y1), m)
d = ImageDraw.Draw(im)
d.rectangle([x1, y1, x2, y2], outline="#e040fb", width=2)
d.text((x1 + 2, max(0, y1 - 12)),
       f"pinpoint {p['cls']} score={p['score']:.2f} "
       f"area={p['area_px']}px seq={p['frame_seq']}", fill="#e040fb")
im.convert("RGB").save("/workspace/evals/out/deep_m4/probe_b_mask_render.png")
print("saved; current frame seq", f.seq, "vs mask seq", p["frame_seq"])
