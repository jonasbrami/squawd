#!/usr/bin/env python3
"""M4 item 2 helper — render detect dets over the recorded frames for the
eyeball mislabel audit (M1b's judgment-column discipline). Reads dets.json
(run_recall.py) + frames/*.png -> overlays/<tag>_c<conf>.png with boxes +
cls/conf labels.

  python3 evals/out/deep_m4/render_dets.py [conf]
"""
import json
import os
import sys

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, "frames")
OUT = os.path.join(HERE, "overlays")
PALETTE = {"building": "#40c4ff", "house": "#3ddc84", "tree": "#8dff57",
           "pole": "#ffd740", "tower": "#ffab40", "car": "#ff5252",
           "truck": "#ff6e40", "person": "#e040fb"}


def main():
    conf = sys.argv[1] if len(sys.argv) > 1 else "0.05"
    os.makedirs(OUT, exist_ok=True)
    dets = json.load(open(os.path.join(HERE, "dets.json")))
    for tag, per in sorted(dets.items()):
        im = Image.open(os.path.join(FRAMES, f"{tag}.png")).convert("RGB")
        d = ImageDraw.Draw(im)
        for det in per[conf]["dets"]:
            x1, y1, x2, y2 = det["xyxy"]
            col = PALETTE.get(det["cls"], "#ffffff")
            d.rectangle([x1, y1, x2, y2], outline=col, width=2)
            d.text((x1 + 2, max(0, y1 - 10)),
                   f"{det['cls']} {det['conf']:.2f}", fill=col)
        path = os.path.join(OUT, f"{tag}_c{conf}.png")
        im.save(path)
        print(path, flush=True)


if __name__ == "__main__":
    main()
