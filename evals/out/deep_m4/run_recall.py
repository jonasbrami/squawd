#!/usr/bin/env python3
"""M4 item 2 — per-concept presence-level recall/FP over the recorded set.

Runs YOLO-World vocab detect (slowlane vocab + vehicle/person prompts:
building,house,tree,pole,tower,car,truck,person) at conf 0.05 AND 0.10 over
evals/out/deep_m4/frames/*.png, through the repo DeepClient DIRECT to the
sidecar on :8100 (not the :8101 tap — keeps the tap log clean, M3 pattern).
Retries typed BUSY (the live slowlane owns the one-flight lock a few % of
the time). Labels come from labels.json (hand-labeled by eye, presence per
concept; convention: house present => building present).

Writes dets.json (raw per-frame dets both confs) and prints the recall/FP
tables as markdown rows.

  PYTHONPATH=. python3 evals/out/deep_m4/run_recall.py
"""
import json
import os
import sys
import time

from agents.core.contact import Frame
from agents.perception.deep_client import DeepClient

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, "frames")
VOCAB = ["building", "house", "tree", "pole", "tower", "car", "truck",
         "person"]
CONCEPTS = ["building", "house", "tree", "pole", "car", "truck", "person"]
CONFS = [0.05, 0.10]
TOKEN = open(".deep_token").read().strip()
URL = "http://172.17.0.1:8100"


def detect_with_retry(client, frame, conf, tries=8):
    for _ in range(tries):
        r = client.detect(frame, VOCAB, conf)
        if r.status == "OK":
            return r
        if r.status == "BUSY":
            time.sleep(0.4)
            continue
        raise SystemExit(f"detect failed: {r.status} {r.detail}")
    raise SystemExit("detect BUSY x8 — slowlane hogging?")


def main():
    with open(os.path.join(FRAMES, "labels.json")) as f:
        labels = {k: v for k, v in json.load(f).items()
                  if not k.startswith("_")}
    client = DeepClient(URL, TOKEN)
    out_path = os.path.join(HERE, "dets.json")
    dets = {}
    if os.path.exists(out_path) and "--rerun" not in sys.argv:
        dets = json.load(open(out_path))
    for tag, lab in sorted(labels.items()):
        if tag in dets:
            continue
        meta = json.load(open(os.path.join(FRAMES, f"{tag}.json")))
        from PIL import Image
        im = Image.open(os.path.join(FRAMES, f"{tag}.png")).convert("RGB")
        frame = Frame(meta["seq"], meta["sim_stamp"], im.width, im.height,
                      im.tobytes())
        dets[tag] = {}
        for conf in CONFS:
            r = detect_with_retry(client, frame, conf)
            dets[tag][str(conf)] = {"dets": r.data.get("dets"),
                                    "latency_ms": r.data.get("latency_ms")}
            print(f"[{tag} conf={conf}] "
                  f"{[(d['cls'], round(d['conf'],2)) for d in r.data['dets']]}",
                  flush=True)
        with open(out_path, "w") as f:
            json.dump(dets, f)

    # ---- presence-level scoring ----
    for conf in CONFS:
        print(f"\n### conf {conf}")
        print("| concept | recall (present) | FP rate (absent) | n_present | n_absent |")
        print("|---|---|---|---|---|")
        for c in CONCEPTS:
            present = [t for t, l in labels.items()
                       if l.get("presence", {}).get(c)
                       and c not in l.get("exclude_from_recall", [])]
            absent = [t for t, l in labels.items()
                      if not l.get("presence", {}).get(c)]
            hits = sum(1 for t in present
                       if any(d["cls"] == c
                              for d in dets[t][str(conf)]["dets"]))
            fps = sum(1 for t in absent
                      if any(d["cls"] == c
                             for d in dets[t][str(conf)]["dets"]))
            rec = f"{hits}/{len(present)}"
            fpr = f"{fps}/{len(absent)}"
            rp = f"{hits/len(present)*100:.0f}%" if present else "—"
            fp = f"{fps/len(absent)*100:.0f}%" if absent else "—"
            print(f"| {c} | {rec} = {rp} | {fpr} = {fp} | "
                  f"{len(present)} | {len(absent)} |")


if __name__ == "__main__":
    main()
