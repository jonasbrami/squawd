#!/usr/bin/env python3
"""demo_coco_replay — W2.5b anti-forgetting replay set (codex R5): 5,000
stratified COCO-seg train2017 images (person/car/truck/bus >= 500 each,
bicycle/motorcycle emphasized, all 80 classes covered) with their YOLO-seg
labels, placed under <out>/replay/ and hardlinked into the train split.

train2017 (not val2017) so the phase-2 COCO-val acceptance metric stays
uncontaminated. Images are fetched individually from images.cocodataset.org
(the full 19 GB train2017 zip is not needed); seg labels come from
ultralytics' coco2017labels-segments.zip (YOLO-seg format already).

  .venv/bin/python scripts/demo_coco_replay.py \
      --out evals/out/w25b_dataset --count 5000

Idempotent: the parsed label index and downloaded images are cached under
<out>/replay/_cache/ — reruns resume where they stopped.
"""
import argparse
import io
import json
import os
import random
import subprocess
import sys
import urllib.request
import zipfile

LABELS_URL = ("https://github.com/ultralytics/assets/releases/download/"
              "v0.0.0/coco2017labels-segments.zip")
IMG_URL = "http://images.cocodataset.org/train2017/{name}.jpg"

# codex R5 emphasis: >=500 each containing person/car/truck/bus; bicycle/
# motorcycle emphasized (pursuit-adjacent classes); all 80 covered >= 25.
MIN_MAIN = {"person": 600, "car": 600, "truck": 600, "bus": 600}
MIN_EMPH = {"bicycle": 400, "motorcycle": 400}
MIN_OTHER = 25
MAIN_CLASSES = list(MIN_MAIN) + list(MIN_EMPH)


def load_labels_zip(cache: str) -> zipfile.ZipFile:
    if not os.path.exists(cache):
        print(f"downloading {LABELS_URL} …", flush=True)
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        tmp = cache + ".part"
        urllib.request.urlretrieve(LABELS_URL, tmp)
        os.replace(tmp, cache)
    return zipfile.ZipFile(cache)


def build_index(zf: zipfile.ZipFile, index_path: str) -> dict:
    """image stem -> sorted class ids (from the train2017 seg labels)."""
    if os.path.exists(index_path):
        return json.load(open(index_path))
    idx = {}
    names = [n for n in zf.namelist()
             if n.startswith("coco/labels/train2017/") and n.endswith(".txt")]
    for i, n in enumerate(names):
        stem = os.path.splitext(os.path.basename(n))[0]
        classes = set()
        with zf.open(n) as fh:
            for line in io.TextIOWrapper(fh):
                parts = line.split()
                if parts:
                    classes.add(int(parts[0]))
        idx[stem] = sorted(classes)
        if i % 20000 == 0:
            print(f"index: {i}/{len(names)}", flush=True)
    json.dump(idx, open(index_path, "w"))
    return idx


def select(idx: dict, count: int, seed: int) -> list[str]:
    from collections import defaultdict
    by_class = defaultdict(list)
    for stem, classes in idx.items():
        for c in classes:
            by_class[c].append(stem)
    names = COCO_NAMES
    rng = random.Random(seed)
    chosen: set[str] = set()

    def take(cls_id: int, n: int) -> None:
        pool = [s for s in by_class[cls_id] if s not in chosen]
        rng.shuffle(pool)
        chosen.update(pool[:n])

    for cname, n in {**MIN_MAIN, **MIN_EMPH}.items():
        take(names.index(cname), n)
    for cls_id in range(80):
        if names[cls_id] in MIN_MAIN or names[cls_id] in MIN_EMPH:
            continue
        have = sum(1 for s in by_class[cls_id] if s in chosen)
        if have < MIN_OTHER:
            take(cls_id, MIN_OTHER - have)
    rest = [s for s in idx if s not in chosen]
    rng.shuffle(rest)
    chosen.update(rest[: max(0, count - len(chosen))])
    return sorted(chosen)


def fetch(stems: list[str], img_dir: str, workers: int = 16) -> list[str]:
    todo = [s for s in stems
            if not os.path.exists(os.path.join(img_dir, s + ".jpg"))]
    print(f"fetching {len(todo)} images ({len(stems) - len(todo)} cached)",
          flush=True)
    if todo:
        with open(os.path.join(img_dir, "_todo.txt"), "w") as fh:
            fh.write("\n".join(todo))
        cmd = (f"cd {img_dir} && xargs -a _todo.txt -P {workers} -n 1 "
               f"bash -c 'curl -sf --max-time 60 -o \"$0.jpg\" "
               f"\"{IMG_URL.replace('{name}', '$0')}\" || echo \"FAIL $0\"'")
        out = subprocess.run(["bash", "-c", cmd], capture_output=True,
                             text=True)
        fails = [l.split()[1] for l in out.stdout.splitlines()
                 if l.startswith("FAIL ")]
        if fails:
            print(f"WARNING: {len(fails)} images failed to download",
                  flush=True)
    return [s for s in stems
            if os.path.exists(os.path.join(img_dir, s + ".jpg"))]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="evals/out/w25b_dataset")
    ap.add_argument("--count", type=int, default=5000)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--workers", type=int, default=16)
    args = ap.parse_args()

    replay = os.path.join(args.out, "replay")
    cache_dir = os.path.join(replay, "_cache")
    img_dir = os.path.join(replay, "images")
    lab_dir = os.path.join(replay, "labels")
    for d in (cache_dir, img_dir, lab_dir,
              os.path.join(args.out, "images", "train"),
              os.path.join(args.out, "labels", "train")):
        os.makedirs(d, exist_ok=True)

    zf = load_labels_zip(os.path.join(cache_dir, "coco2017labels-segments.zip"))
    idx = build_index(zf, os.path.join(cache_dir, "train2017_index.json"))
    print(f"index: {len(idx)} train2017 label files", flush=True)

    stems = select(idx, args.count, args.seed)
    got = fetch(stems, img_dir, args.workers)
    print(f"{len(got)}/{len(stems)} images present", flush=True)

    names = COCO_NAMES
    class_hist = {n: 0 for n in names}
    linked = 0
    for s in got:
        src = f"coco/labels/train2017/{s}.txt"
        txt = zf.read(src).decode()
        with open(os.path.join(lab_dir, s + ".txt"), "w") as fh:
            fh.write(txt)
        for line in txt.splitlines():
            if line.split():
                class_hist[names[int(line.split()[0])]] += 1
        for a, b in ((os.path.join(img_dir, s + ".jpg"),
                      os.path.join(args.out, "images", "train", s + ".jpg")),
                     (os.path.join(lab_dir, s + ".txt"),
                      os.path.join(args.out, "labels", "train", s + ".txt"))):
            if not os.path.exists(b):
                try:
                    os.link(a, b)
                except OSError:
                    import shutil
                    shutil.copyfile(a, b)
                linked += 1
    stats = {"count": len(got), "seed": args.seed,
             "class_image_counts": class_hist,
             "shortfalls": {c: n for c, n in
                            {**MIN_MAIN, **MIN_EMPH}.items()
                            if class_hist[c] < n}}
    with open(os.path.join(replay, "replay_stats.json"), "w") as fh:
        json.dump(stats, fh, indent=2)
    print(f"linked {linked} files into the train split", flush=True)
    print("main-class coverage:",
          {c: class_hist[c] for c in MAIN_CLASSES}, flush=True)
    covered = sum(1 for v in class_hist.values() if v > 0)
    print(f"classes covered: {covered}/80; shortfalls: "
          f"{stats['shortfalls'] or 'none'}", flush=True)
    return 0


COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"]


if __name__ == "__main__":
    sys.exit(main())
