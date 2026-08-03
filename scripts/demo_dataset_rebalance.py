#!/usr/bin/env python3
"""demo_dataset_rebalance — W2.5b whole-block split rebalance (phase-1a
discipline): reassign gz/neg frames between train/val/test so val and test
each hold ~15% of the gz total, moving ONLY whole capture blocks — one block
= a 20 s sim-time window (demo_dataset.SPLIT_BLOCK_S) within a single boot
(boots are segmented by stamp resets in frames.jsonl; every boot restarts
sim time at 0). Existing val/test membership is preserved (blocks already
there stay put); replay (COCO) images are not in frames.jsonl and are never
touched — val/test stay gz-only.

frames.jsonl is rewritten with the corrected per-record "split" so the QA
renderer (demo_dataset.py --qa) and downstream tooling keep resolving image
paths; a .bak of the pre-rebalance jsonl is kept.

  .venv/bin/python scripts/demo_dataset_rebalance.py \
      --out evals/out/w25b_dataset [--val-frac 0.15 --test-frac 0.15] [--dry]
"""
import argparse
import json
import os
import shutil

BLOCK_S = 20.0


def load(out):
    recs = [json.loads(l) for l in open(os.path.join(out, "frames.jsonl"))]
    for r in recs:
        p = os.path.join(out, "images", r["split"], r["stem"] + ".png")
        if not os.path.exists(p):
            raise SystemExit(f"frames.jsonl/fs mismatch: {p} missing — "
                             "rebalance would corrupt the split; aborting")
    return recs


def segment_runs(recs):
    """Boot runs: a new run starts when the sim stamp resets backwards."""
    run = -1
    prev = -1e18
    for r in recs:
        if run == -1 or r["stamp"] < prev - 1.0:
            run += 1
        prev = r["stamp"]
        r["_run"] = run
    return recs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="evals/out/w25b_dataset")
    ap.add_argument("--val-frac", type=float, default=0.15)
    ap.add_argument("--test-frac", type=float, default=0.15)
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()
    out = args.out

    recs = segment_runs(load(out))
    total = len(recs)
    blocks = {}                       # (run, block) -> list[rec]
    for r in recs:
        blocks.setdefault((r["_run"], int(r["stamp"] // BLOCK_S)), []).append(r)

    def census():
        c = {"train": 0, "val": 0, "test": 0}
        for r in recs:
            c[r["split"]] += 1
        return c

    have = census()
    goals = (("val", args.val_frac), ("test", args.test_frac))
    moves = []
    chosen = set()
    for split, frac in goals:
        target = frac * total
        while True:
            err_now = abs(have[split] - target)
            best, best_err = None, err_now
            for bk, rs in blocks.items():
                if bk in chosen or any(r["split"] != "train" for r in rs):
                    continue
                err = abs(have[split] + len(rs) - target)
                if err < best_err:
                    best, best_err = bk, err
            if best is None:
                break
            chosen.add(best)
            moves.append((best, split))
            have[split] += len(blocks[best])
            have["train"] -= len(blocks[best])

    print(f"total gz/neg={total} before={census()}")
    for bk, sp in moves:
        print(f"  move block run{bk[0]}/t{bk[1] * 20:.0f}s "
              f"({len(blocks[bk])} frames) -> {sp}")
    for bk, sp in moves:
        for r in blocks[bk]:
            r["split"] = sp
    after = census()
    print(f"after={after} val={after['val'] / total:.1%} "
          f"test={after['test'] / total:.1%}")
    if args.dry:
        print("--dry: no files moved")
        return
    for bk, sp in moves:
        old = "train"
        for r in blocks[bk]:
            for kind, ext in (("images", ".png"), ("labels", ".txt")):
                src = os.path.join(out, kind, old, r["stem"] + ext)
                dst = os.path.join(out, kind, sp, r["stem"] + ext)
                if os.path.exists(src):
                    os.replace(src, dst)
    jl = os.path.join(out, "frames.jsonl")
    shutil.copy2(jl, jl + ".bak")
    with open(jl, "w") as fh:
        for r in recs:
            r.pop("_run", None)
            fh.write(json.dumps(r) + "\n")
    print(f"moved {sum(len(blocks[bk]) for bk, _ in moves)} frames; "
          "frames.jsonl rewritten (.bak kept)")


if __name__ == "__main__":
    main()
