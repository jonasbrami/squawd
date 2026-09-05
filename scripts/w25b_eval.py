#!/usr/bin/env python3
"""w25b_eval — W2.5b phase-2 acceptance eval on the gz splits (codex R5 bar,
spec docs/benchmarks/w3-detector-codex-r5.md §4; task items 1/2/5).

Runs the exported ONNX directly with onnxruntime — preprocess/decode adapted
from scripts/w0_assets_eval.py (letterbox 114-pad, class-aware NMS iou=0.45,
conf 0.25) so numbers are comparable to the production OnnxBackend precedent.
Ground truth comes from the QA'd dataset itself: labels/<split>/<stem>.txt
seg polygons (bbox of polygon) paired with the per-instance cell metadata in
frames.jsonl.

  uv run --no-project --with onnxruntime --with numpy --with pillow \
      python scripts/w25b_eval.py --model models/coco-nano-seg-v2-640

Modes (--mode):
  recall   gz val+test per-class recall, overall and per cell
           (vehicle: mesh x aspect x range; person: aspect x range)
  streaks  gz test clips (frames.jsonl time-ordered per cam): consecutive-miss
           streaks in seconds + miss-gap p95
  fp       admitted-class false positives on the 1,100 negative frames,
           incl. explicit "chair"-on-house-roof accounting
  all      everything (default), one JSON report

Bars (R5/task): vehicle recall >=0.90 overall & >=0.80 every mesh cell;
person >=0.90 & >=0.80 every aspect x range cell; no miss streak >1.0 s,
miss-gap p95 <=0.5 s; admitted-class FP <=0.5% of negative frames and zero
house-roof vehicle/person contacts.
"""
import argparse
import json
import os
import sys
import time
from collections import defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DATASET = os.path.join(ROOT, "evals/out/w25b_dataset")
ADMITTED = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle",
            5: "bus", 7: "truck"}
VEHICLE = {2, 7}
PERSON = {0}
CHAIR = 56
HOUSE_ROOF_CAMS = {"neg_house1_obl", "neg_house2_roof"}


# ---- preprocess/decode (adapted from scripts/w0_assets_eval.py, boxes only)

def letterbox(img, size):
    import numpy as np
    w, h = img.size
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    a = np.asarray(img.resize((nw, nh))) if (nw, nh) != (w, h) else np.asarray(img)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas[py:py + nh, px:px + nw] = a
    x = canvas.transpose(2, 0, 1)[None].astype("float32") / 255.0
    return x, scale, (px, py)


def nms(boxes, scores, iou=0.45):
    import numpy as np
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_j = (boxes[order[1:], 2] - boxes[order[1:], 0]) * \
                 (boxes[order[1:], 3] - boxes[order[1:], 1])
        order = order[1:][inter / (area_i + area_j - inter + 1e-9) <= iou]
    return keep


def detect(sess, img, size, conf):
    """-> [(cls_id, score, xyxy)] in original-image pixels."""
    import numpy as np
    x, scale, pad = letterbox(img, size)
    outs = sess.run(None, {sess.get_inputs()[0].name: x})
    det = outs[0][0]                                 # (C, A)
    protos_channels = outs[1].shape[1]
    nc = det.shape[0] - 4 - protos_channels
    boxes_cxcywh = det[:4].T
    cls_scores = det[4:4 + nc].T
    cls_id = cls_scores.argmax(1)
    scores = cls_scores.max(1)
    keep = scores >= conf
    if not keep.any():
        return []
    boxes_cxcywh, scores, cls_id = (boxes_cxcywh[keep], scores[keep],
                                    cls_id[keep])
    x1 = (boxes_cxcywh[:, 0] - boxes_cxcywh[:, 2] / 2 - pad[0]) / scale
    y1 = (boxes_cxcywh[:, 1] - boxes_cxcywh[:, 3] / 2 - pad[1]) / scale
    x2 = (boxes_cxcywh[:, 0] + boxes_cxcywh[:, 2] / 2 - pad[0]) / scale
    y2 = (boxes_cxcywh[:, 1] + boxes_cxcywh[:, 3] / 2 - pad[1]) / scale
    boxes = np.stack([x1, y1, x2, y2], 1)
    out = []
    for c in np.unique(cls_id):
        idx = np.where(cls_id == c)[0]
        for i in nms(boxes[idx], scores[idx]):
            j = idx[i]
            out.append((int(c), float(scores[j]),
                        tuple(float(v) for v in boxes[j])))
    return out


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    aa = (a[2] - a[0]) * (a[3] - a[1])
    bb = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (aa + bb - inter + 1e-9)


def center_in(pt, box):
    return box[0] <= pt[0] <= box[2] and box[1] <= pt[1] <= box[3]


def overlaps(det_box, exp_box):
    """w0_assets_eval.py:248 rule: IoU>0.15 or either center inside."""
    if box_iou(det_box, exp_box) > 0.15:
        return True
    ec = ((exp_box[0] + exp_box[2]) / 2, (exp_box[1] + exp_box[3]) / 2)
    dc = ((det_box[0] + det_box[2]) / 2, (det_box[1] + det_box[3]) / 2)
    return center_in(ec, det_box) or center_in(dc, exp_box)


def p95(vals):
    vals = sorted(vals)
    return vals[int(0.95 * (len(vals) - 1))] if vals else None


# ---- dataset access ----------------------------------------------------------

def load_frames():
    recs = {}
    with open(os.path.join(DATASET, "frames.jsonl")) as f:
        for line in f:
            r = json.loads(line)
            recs[r["stem"]] = r
    return recs


def gt_boxes(split, stem, w, h):
    """labels/<split>/<stem>.txt seg polygons -> [(cls, xyxy px)]."""
    out = []
    fp = os.path.join(DATASET, "labels", split, stem + ".txt")
    if not os.path.exists(fp):
        return out
    with open(fp) as f:
        for ln in f:
            p = ln.split()
            if len(p) < 7:
                continue
            xy = [float(v) for v in p[1:]]
            xs, ys = xy[0::2], xy[1::2]
            out.append((int(p[0]),
                        (min(xs) * w, min(ys) * h, max(xs) * w, max(ys) * h)))
    return out


def pair_labels(gt, labels, anomalies):
    """Align frames.jsonl label metadata with label-file instances.

    Capture wrote both together, so order normally matches 1:1; fall back to
    greedy same-class pairing (counted) if a frame disagrees."""
    if len(gt) == len(labels) and \
            all(g[0] == lab["cls"] for g, lab in zip(gt, labels)):
        return list(zip(range(len(gt)), labels))
    anomalies[0] += 1
    used = [False] * len(gt)
    out = []
    for lab in labels:
        gi = next((i for i, g in enumerate(gt)
                   if not used[i] and g[0] == lab["cls"]), None)
        if gi is not None:
            used[gi] = True
        out.append((gi, lab))
    return out


def match_frame(dets, gt, labels, anomalies):
    """-> hit flag (True/False) or None (unpaired) per frames.jsonl label."""
    hits = []
    for gi, lab in pair_labels(gt, labels, anomalies):
        if gi is None:
            hits.append(None)
            continue
        cls, box = gt[gi]
        hits.append(any(c == cls and overlaps(b, box) for c, s, b in dets))
    return hits


def run_frames(sess, size, recs, conf, filter_fn, progress_tag):
    """Detect over selected frames -> {stem: dets}. Single pass shared by
    the recall/streak/fp modes (they partition the dataset)."""
    from PIL import Image
    stems = sorted(s for s, r in recs.items() if filter_fn(r))
    out = {}
    t0 = time.time()
    for k, stem in enumerate(stems):
        r = recs[stem]
        fp = os.path.join(DATASET, "images", r["split"], stem + ".png")
        img = Image.open(fp).convert("RGB")
        out[stem] = detect(sess, img, size, conf)
        if (k + 1) % 200 == 0:
            print(f"  {progress_tag}: {k + 1}/{len(stems)} "
                  f"({(time.time() - t0) / (k + 1) * 1000:.0f} ms/frame)",
                  flush=True)
    print(f"  {progress_tag}: {len(stems)} frames in "
          f"{time.time() - t0:.0f}s", flush=True)
    return out


# ---- mode: recall ------------------------------------------------------------

def mode_recall(recs, dets_by_stem, conf):
    anomalies = [0]
    veh_cells = defaultdict(lambda: [0, 0])    # (mover,aspect,range) -> [hit,tot]
    veh_bands = defaultdict(lambda: [0, 0])    # (aspect,range)
    per_cells = defaultdict(lambda: [0, 0])    # (aspect,range)
    overall = {"vehicle": [0, 0], "person": [0, 0]}
    misses = []
    for stem, dets in sorted(dets_by_stem.items()):
        r = recs[stem]
        if r["negative"] or r["split"] not in ("val", "test"):
            continue
        gt = gt_boxes(r["split"], stem, 640, 360)
        for lab, hit in zip(r["labels"],
                            match_frame(dets, gt, r["labels"], anomalies)):
            if hit is None:
                continue
            mover, aspect, rng = lab["cell"]
            if lab["cls"] in VEHICLE:
                grp = overall["vehicle"]
            elif lab["cls"] in PERSON:
                grp = overall["person"]
            else:
                continue                       # no other classes exist in gz
            grp[1] += 1
            grp[0] += int(hit)
            if rng is None:
                continue                       # byproduct: overall only
            cell = veh_cells if lab["cls"] in VEHICLE else per_cells
            key = (mover, aspect, rng) if lab["cls"] in VEHICLE \
                else (aspect, rng)
            cell[key][1] += 1
            cell[key][0] += int(hit)
            if lab["cls"] in VEHICLE:
                veh_bands[(aspect, rng)][1] += 1
                veh_bands[(aspect, rng)][0] += int(hit)
            if not hit:
                misses.append({"stem": stem, "split": r["split"],
                               "cam": r["cam"], "cls": lab["cls"],
                               "cell": lab["cell"], "clip": lab["clip"],
                               "slant": lab["slant"], "alt": lab["alt"]})

    def cell_table(cells):
        return {"|".join(str(k) for k in key):
                {"instances": v[1], "hits": v[0],
                 "recall": round(v[0] / v[1], 4)}
                for key, v in sorted(cells.items(), key=lambda kv: str(kv[0]))}

    def fails(cells, bar):
        return {k: v for k, v in cell_table(cells).items()
                if v["recall"] < bar}

    res = {
        "frames": sum(1 for s, r in recs.items()
                      if not r["negative"] and r["split"] in ("val", "test")),
        "label_pairing_anomalies": anomalies[0],
        "overall": {
            g: {"instances": v[1], "hits": v[0],
                "recall": round(v[0] / v[1], 4) if v[1] else None}
            for g, v in overall.items()},
        "vehicle_cells_mesh_aspect_range": cell_table(veh_cells),
        "vehicle_bands_aspect_range": cell_table(veh_bands),
        "person_cells_aspect_range": cell_table(per_cells),
        "vehicle_cells_below_0.80": fails(veh_cells, 0.80),
        "person_cells_below_0.80": fails(per_cells, 0.80),
        "verdict": {
            "vehicle_overall_>=0.90": overall["vehicle"][1] > 0 and
                overall["vehicle"][0] / overall["vehicle"][1] >= 0.90,
            "vehicle_every_cell_>=0.80": not fails(veh_cells, 0.80),
            "person_overall_>=0.90": overall["person"][1] > 0 and
                overall["person"][0] / overall["person"][1] >= 0.90,
            "person_every_cell_>=0.80": not fails(per_cells, 0.80),
        },
        "missed_instances": misses,
    }
    ov = res["overall"]
    print(f"  recall: vehicle {ov['vehicle']['recall']} "
          f"({ov['vehicle']['hits']}/{ov['vehicle']['instances']}), "
          f"person {ov['person']['recall']} "
          f"({ov['person']['hits']}/{ov['person']['instances']})")
    print(f"  vehicle cells <0.80: {len(res['vehicle_cells_below_0.80'])}, "
          f"person cells <0.80: {len(res['person_cells_below_0.80'])}")
    return res


# ---- mode: streaks -----------------------------------------------------------

def mode_streaks(recs, dets_by_stem, conf):
    anomalies = [0]
    by_cam = defaultdict(list)
    for stem, r in recs.items():
        if r["negative"] or r["split"] != "test":
            continue
        by_cam[r["cam"]].append((stem, r))

    def runs_of(flags):
        """-> [(start_idx, length)] runs of True."""
        runs, i = [], 0
        while i < len(flags):
            if flags[i]:
                j = i
                while j + 1 < len(flags) and flags[j + 1]:
                    j += 1
                runs.append((i, j - i + 1))
                i = j + 1
            else:
                i += 1
        return runs

    strict_runs, lenient_runs, gaps = [], [], []
    for cam, items in sorted(by_cam.items()):
        items.sort(key=lambda t: t[1]["stamp"])
        stamps = [r["stamp"] for _, r in items]
        dts = [b - a for a, b in zip(stamps, stamps[1:])]
        dt = sorted(dts)[len(dts) // 2] if dts else 0.2
        strict, lenient, evaluated = [], [], []
        for stem, r in items:
            targets = [i for i, lab in enumerate(r["labels"])
                       if lab["cell"][2] is not None]
            if not targets:
                continue                       # byproduct-only frame: skip
            gt = gt_boxes("test", stem, 640, 360)
            hits = match_frame(dets_by_stem[stem], gt, r["labels"], anomalies)
            strict.append(any(hits[i] is False for i in targets))
            lenient.append(all(hits[i] is False for i in targets))
            evaluated.append(r["stamp"])
        for i, n in runs_of(strict):
            strict_runs.append({"cam": cam, "frames": n,
                                "duration_s": round(n * dt, 3),
                                "start_stamp": evaluated[i]})
        for i, n in runs_of(lenient):
            lenient_runs.append({"cam": cam, "frames": n,
                                 "duration_s": round(n * dt, 3)})
        for i, n in runs_of(strict):           # miss-gap between hits
            if i > 0 and i + n < len(evaluated):
                gaps.append(evaluated[i + n] - evaluated[i - 1])

    max_strict = max((r["duration_s"] for r in strict_runs), default=0.0)
    max_strict_frames = max((r["frames"] for r in strict_runs), default=0)
    max_lenient = max((r["duration_s"] for r in lenient_runs), default=0.0)
    gap_p95 = p95(gaps)
    res = {
        "label_pairing_anomalies": anomalies[0],
        "cams": len(by_cam),
        "strict_miss_definition": "any in-quota label unmatched in frame",
        "max_miss_streak_frames": max_strict_frames,
        "max_miss_streak_s": max_strict,
        "max_miss_streak_s_lenient": max_lenient,
        "miss_gap_p95_s": (round(gap_p95, 3) if gap_p95 is not None else None),
        "n_miss_runs": len(strict_runs),
        "miss_runs": sorted(strict_runs, key=lambda r: -r["duration_s"])[:20],
        "verdict": {"no_streak_>1.0s": max_strict <= 1.0,
                    "miss_gap_p95_<=0.5s": (gap_p95 is not None
                                            and gap_p95 <= 0.5)},
    }
    print(f"  streaks: max {max_strict_frames} frames "
          f"({max_strict:.2f}s) strict / {max_lenient:.2f}s lenient, "
          f"miss-gap p95 {res['miss_gap_p95_s']}s over {len(gaps)} gaps")
    return res


# ---- mode: fp ----------------------------------------------------------------

def mode_fp(recs, dets_by_stem, conf, classes):
    neg = [s for s, r in recs.items() if r["negative"]]
    fp_frames, per_cam = 0, defaultdict(int)
    per_class = defaultdict(int)
    roof_contacts, chair_roof = [], []
    fp_detail = []
    for stem in sorted(neg):
        r = recs[stem]
        dets = dets_by_stem[stem]
        bad = [(c, s, b) for c, s, b in dets if c in ADMITTED]
        for c, s, b in dets:
            per_class[classes[c]] += 1
        if bad:
            fp_frames += 1
            per_cam[r["cam"]] += 1
            fp_detail.append({"stem": stem, "cam": r["cam"],
                              "dets": [(classes[c], round(s, 3))
                                       for c, s, b in bad]})
        if r["cam"] in HOUSE_ROOF_CAMS:
            for c, s, b in bad:
                roof_contacts.append({"stem": stem, "cam": r["cam"],
                                      "cls": classes[c],
                                      "conf": round(s, 3)})
            for c, s, b in dets:
                if c == CHAIR:
                    chair_roof.append({"stem": stem, "cam": r["cam"],
                                       "conf": round(s, 3)})
    rate = fp_frames / len(neg) if neg else None
    res = {
        "negative_frames": len(neg),
        "admitted_classes": sorted(ADMITTED.values()),
        "fp_frames": fp_frames,
        "fp_rate": round(rate, 5) if rate is not None else None,
        "fp_frames_by_cam": dict(sorted(per_cam.items())),
        "detections_by_class_on_negatives": dict(sorted(per_class.items())),
        "house_roof_vehicle_person_contacts": roof_contacts,
        "chair_on_house_roof": chair_roof,
        "fp_detail": fp_detail[:50],
        "verdict": {
            "fp_rate_<=0.005": rate is not None and rate <= 0.005,
            "zero_house_roof_vehicle_person": not roof_contacts,
        },
    }
    print(f"  fp: {fp_frames}/{len(neg)} negative frames "
          f"(rate {res['fp_rate']}), roof contacts "
          f"{len(roof_contacts)}, chair-on-roof {len(chair_roof)}")
    return res


# ---- main --------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/coco-nano-seg-v2-640",
                    help="path WITHOUT extension (needs .onnx + .json)")
    ap.add_argument("--mode", choices=["recall", "streaks", "fp", "all"],
                    default="all")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--report", default="evals/out/w25b_eval/w25b_report.json")
    args = ap.parse_args()

    import onnxruntime as ort

    manifest = json.load(open(args.model + ".json"))
    classes = manifest["classes"]
    sess = ort.InferenceSession(args.model + ".onnx",
                                providers=["CPUExecutionProvider"])
    size = sess.get_inputs()[0].shape[2]
    print(f"model: {args.model}.onnx imgsz={size} sha256={manifest['sha256'][:16]}…",
          flush=True)

    recs = load_frames()
    want = {"recall", "streaks", "fp"} if args.mode == "all" else {args.mode}
    pos_sel = lambda r: not r["negative"] and r["split"] in ("val", "test")
    neg_sel = lambda r: r["negative"]
    dets_pos, dets_neg = {}, {}
    if want & {"recall", "streaks"}:
        dets_pos = run_frames(sess, size, recs, args.conf, pos_sel, "positives")
    if "fp" in want:
        dets_neg = run_frames(sess, size, recs, args.conf, neg_sel, "negatives")

    report = {
        "gate": "W2.5b phase-2 fine-tune acceptance (codex R5)",
        "date_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "model": {"path": args.model + ".onnx",
                  "sha256": manifest["sha256"], "imgsz": size,
                  "finetuned_from": manifest.get("finetuned_from")},
        "conf": args.conf, "nms_iou": 0.45,
    }
    if "recall" in want:
        report["recall"] = mode_recall(recs, dets_pos, args.conf)
    if "streaks" in want:
        report["streaks"] = mode_streaks(recs, dets_pos, args.conf)
    if "fp" in want:
        report["fp"] = mode_fp(recs, dets_neg, args.conf, classes)

    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w") as f:
        json.dump(report, f, indent=1)
    print("report:", args.report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
