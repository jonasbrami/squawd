"""FrameDump: record POV + overhead cinecam frames from INSIDE the eval runner.

A standalone recorder process's gz-transport subscriptions silently died the
moment run_evals joined the transport (both streams cut at the same instant,
process healthy, gz healthy) — so recording lives in the runner process, where
GzCameras/GzPoses nodes already coexist. Enabled by `run_evals --record DIR`;
frames land as numbered JPEGs per stream + index.csv (frame, mono_ms) so
assembly can compute each stream's true fps.
"""
import io
import os
import threading
import time


class FrameDump:
    def __init__(self, out_dir: str, world: str, over_topic: str = "cinecam",
                 deps=None) -> None:
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image

        self._deps = deps
        self._stop = threading.Event()
        self._poses_f = None
        os.makedirs(out_dir, exist_ok=True)
        if deps is not None:
            # 10 Hz pose log (epoch, drone e/n/alt, mover name:x:y triples) so
            # assembly can draw tracking rings — the actors are ~1 m boxes,
            # 3 px from a 300 m cinecam, invisible without markers.
            self._poses_f = open(os.path.join(out_dir, "poses.csv"), "w", buffering=1)
            threading.Thread(target=self._pose_loop, daemon=True).start()
        self._streams = {
            "pov": (f"/world/{world}/model/x500_depth_0/link/OakD-Lite/base_link"
                    f"/sensor/IMX214/image"),
            "over": over_topic,
        }
        self._node = Node()
        self._state = {}
        self._cbs = []
        for name, topic in self._streams.items():
            d = os.path.join(out_dir, name)
            os.makedirs(d, exist_ok=True)
            idx = open(os.path.join(d, "index.csv"), "w", buffering=1)
            self._state[name] = [0, idx, d, threading.Lock()]
            cb = self._make_cb(name)
            self._cbs.append(cb)
            self._node.subscribe(Image, topic, cb)

    def _make_cb(self, name: str):
        from PIL import Image as PILImage

        def cb(msg):
            if self._stop.is_set():
                return
            n, idx, d, lock = self._state[name]
            with lock:
                self._state[name][0] += 1
            try:
                img = PILImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data))
                buf = io.BytesIO()
                img.save(buf, "JPEG", quality=90)
                with open(os.path.join(d, f"{n:06d}.jpg"), "wb") as f:
                    f.write(buf.getvalue())
                idx.write(f"{n},{time.monotonic() * 1000:.0f}\n")
            except Exception:
                pass
        return cb

    def _pose_loop(self) -> None:
        while not self._stop.is_set():
            try:
                st = self._deps.world.drone_state(self._deps.bridge, 0)
                movers = (self._deps.oracle_truth.poses()
                          if self._deps.oracle_truth else {})
                d = f"{st[0]:.1f},{st[1]:.1f},{st[2]:.1f}" if st else ",,"
                mv = ";".join(f"{n}:{p[0]:.1f}:{p[1]:.1f}"
                              for n, p in sorted(movers.items()))
                self._poses_f.write(f"{time.time():.2f},{d},{mv}\n")
            except Exception:
                pass
            time.sleep(0.1)

    def stop(self) -> str:
        self._stop.set()
        time.sleep(0.3)
        if self._poses_f is not None:
            self._poses_f.close()
        parts = []
        for name, (n, idx, _d, _l) in self._state.items():
            idx.close()
            parts.append(f"{name}={n}")
        return "frames: " + " ".join(parts)
