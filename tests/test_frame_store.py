"""Frame atomicity (C1): a reader must never observe fields mixed across
generations, under a hammered writer."""
import threading

from agents.core.contact import LatestFrame


def test_seq_increases_and_snapshot_consistent():
    h = LatestFrame()
    assert h.seq() == 0 and h.get() is None
    for k in range(1, 5):
        h.set(float(k), 640, 360, bytes([k]) * 10)
        f = h.get()
        assert f.seq == k and f.sim_stamp == float(k)
        assert f.width == 640 and f.rgb == bytes([k]) * 10


def test_atomicity_under_hammering():
    h = LatestFrame()
    stop = False
    bad = []

    def writer():
        k = 0
        while not stop:
            k += 1
            # stamp, dims, and payload all carry the generation marker k
            h.set(float(k), k, k, k.to_bytes(2, "little") * 100)

    def reader():
        while not stop:
            f = h.get()
            if f is None:
                continue
            if f.seq != int(f.sim_stamp) or f.width != f.seq \
                    or f.height != f.seq or f.rgb[:2] != f.seq.to_bytes(2, "little"):
                bad.append(f)

    threads = [threading.Thread(target=writer)] + \
              [threading.Thread(target=reader) for _ in range(4)]
    for t in threads:
        t.start()
    threading.Event().wait(0.5)
    stop = True
    for t in threads:
        t.join()
    assert not bad, f"torn frames observed: {bad[:3]}"
