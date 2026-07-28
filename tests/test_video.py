"""Unit tests for the observatory H.264 video path (agents/observatory/video.py).

Browser-free: encode synthetic RGB frames, assert keyframe/delta structure and
that the stream decodes back at the downscaled size, then exercise the VideoHub
pump's fan-out, forced-keyframe-on-subscribe behaviour, and the per-access-unit
(seq, sim_stamp) stamping the overlay match depends on (ICD §8.2/§8.3).
"""
import asyncio

import av
import pytest
from PIL import Image

from agents.core.contact import Frame
from agents.observatory.video import H264Encoder, VideoHub, _target_size


def _rgb(w: int, h: int, val: int) -> bytes:
    return Image.new("RGB", (w, h), (val, val, val)).tobytes()


def test_target_size_downscales_to_even_dims():
    assert _target_size(640, 360, 480) == (480, 270)
    assert _target_size(641, 361, 480) == (480, 270)   # rounded even
    assert _target_size(320, 180, 480) == (320, 180)   # no upscale


def test_first_packet_is_keyframe_then_deltas():
    enc = H264Encoder(maxpx=320)
    first = enc.encode(640, 360, _rgb(640, 360, 10))
    assert first and first[0][0] is True                # keyframe
    deltas = []
    for v in (20, 30, 40):
        deltas += enc.encode(640, 360, _rgb(640, 360, v))
    assert deltas and all(is_key is False for is_key, _ in deltas)


def test_force_key_emits_idr_mid_stream():
    enc = H264Encoder(maxpx=320)
    enc.encode(640, 360, _rgb(640, 360, 10))            # natural first IDR
    enc.encode(640, 360, _rgb(640, 360, 20))            # delta
    forced = enc.encode(640, 360, _rgb(640, 360, 30), force_key=True)
    assert any(is_key for is_key, _ in forced)


def test_codec_string_is_baseline():
    enc = H264Encoder(maxpx=320)
    enc.encode(640, 360, _rgb(640, 360, 10))
    assert enc.codec_string is not None
    assert enc.codec_string.startswith("avc1.42")       # baseline profile_idc=0x42


def test_stream_decodes_at_downscaled_size():
    enc = H264Encoder(maxpx=320)
    chunks = []
    for v in (10, 20, 30, 40):
        chunks += [b for _, b in enc.encode(640, 360, _rgb(640, 360, v))]
    dec = av.CodecContext.create("h264", "r")
    sizes = []
    for b in chunks:
        for fr in dec.decode(av.packet.Packet(b)):
            sizes.append((fr.width, fr.height))
    for fr in dec.decode(av.packet.Packet(b"")):         # flush
        sizes.append((fr.width, fr.height))
    assert sizes and all(s == (320, 180) for s in sizes)


class _FakeCameras:
    """snapshot()-only camera stand-in (the ICD §8.3 consumer API) with a
    bumpable frame; sim_stamp advances 0.1 s per frame like the 10 Hz feed."""

    def __init__(self):
        self._f: Frame | None = None

    def bump(self):
        seq = (self._f.seq + 1) if self._f else 1
        self._f = Frame(seq, seq * 0.1, 640, 360, _rgb(640, 360, 10 + seq))

    def snapshot(self, i):
        return self._f


def test_hub_subscriber_gets_stamped_keyframe_first():
    async def run():
        cams = _FakeCameras()
        cams.bump()                                      # a frame is available
        hub = VideoHub(cams, 0, maxpx=320, interval=0.01)
        q = hub.subscribe()                              # self-starts the pump
        try:
            return await asyncio.wait_for(q.get(), timeout=2.0)
        finally:
            hub._pump_task.cancel()
            try:
                await hub._pump_task
            except asyncio.CancelledError:
                pass

    seq, stamp, is_key, codec, data = asyncio.run(run())
    assert (seq, stamp) == (1, 0.1)                      # the frame's own stamps
    assert is_key is True                                # first frame is an IDR
    assert codec and codec.startswith("avc1.42")
    assert len(data) > 0


def test_hub_pumps_each_new_frame_with_its_sim_stamp():
    async def run():
        cams = _FakeCameras()
        cams.bump()
        hub = VideoHub(cams, 0, maxpx=320, interval=0.01)
        q = hub.subscribe()
        got = [await asyncio.wait_for(q.get(), timeout=2.0)]
        cams.bump()                                      # seq 2, stamp 0.2
        got.append(await asyncio.wait_for(q.get(), timeout=2.0))
        hub._pump_task.cancel()
        try:
            await hub._pump_task
        except asyncio.CancelledError:
            pass
        return got

    first, second = asyncio.run(run())
    assert (first[0], first[1]) == (1, 0.1)
    assert (second[0], second[1]) == (2, 0.2)
    assert second[2] is False                            # delta after the IDR


def test_hub_skips_encoding_with_no_subscribers():
    async def run():
        cams = _FakeCameras()
        cams.bump()
        hub = VideoHub(cams, 0, maxpx=320, interval=0.01)
        task = asyncio.create_task(hub.pump())
        await asyncio.sleep(0.1)                          # no subscribers
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        return hub._last                                  # nothing encoded => 0

    assert asyncio.run(run()) == 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
