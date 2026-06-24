"""Unit tests for the observatory H.264 video path (agents/observatory/video.py).

Browser-free: encode synthetic RGB frames, assert keyframe/delta structure and
that the stream decodes back at the downscaled size, then exercise the VideoHub
pump's fan-out and forced-keyframe-on-subscribe behaviour.
"""
import asyncio

import av
import pytest
from PIL import Image

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
    """Minimal seq()/raw() camera stand-in with a bumpable frame."""

    def __init__(self, n):
        self._n = n
        self._seq = {i: 0 for i in range(n)}

    def bump(self, i):
        self._seq[i] += 1

    def seq(self, i):
        return self._seq[i]

    def raw(self, i):
        if self._seq[i] == 0:
            return None
        return (640, 360, _rgb(640, 360, 10 + self._seq[i]))


def test_hub_subscriber_gets_keyframe_first_with_codec():
    async def run():
        cams = _FakeCameras(2)
        for i in range(2):
            cams.bump(i)                                 # a frame is available
        hub = VideoHub(cams, 2, maxpx=320, interval=0.01)
        q = hub.subscribe()                              # self-starts the pump
        msgs = []
        try:
            for _ in range(2):                           # one keyframe per drone
                msgs.append(await asyncio.wait_for(q.get(), timeout=2.0))
        finally:
            hub._pump_task.cancel()
            try:
                await hub._pump_task
            except asyncio.CancelledError:
                pass
        return msgs

    msgs = asyncio.run(run())
    seen = {i: (is_key, codec) for i, is_key, codec, _ in msgs}
    assert set(seen) == {0, 1}
    for i, (is_key, codec) in seen.items():
        assert is_key is True                            # first frame per drone is an IDR
        assert codec and codec.startswith("avc1.42")


def test_hub_skips_encoding_with_no_subscribers():
    async def run():
        cams = _FakeCameras(1)
        cams.bump(0)
        hub = VideoHub(cams, 1, maxpx=320, interval=0.01)
        task = asyncio.create_task(hub.pump())
        await asyncio.sleep(0.1)                          # no subscribers
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        return hub._last                                 # nothing encoded => empty

    assert asyncio.run(run()) == {}


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
