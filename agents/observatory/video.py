"""H.264 video for the observatory tiles.

Replaces per-frame JPEG with a real codec. One libx264 encoder per drone
(baseline / zero-latency / Annex-B, with SPS+PPS carried in every keyframe so the
browser's WebCodecs ``VideoDecoder`` is self-sufficient), driven by a single
background pump that encodes each new frame exactly once and fans the NAL units
out to every connected WebSocket. The browser decodes per tile onto a ``<canvas>``.

Only the observatory streaming path lives here; the VLM still gets JPEG straight
from ``core.camera`` (``jpeg_b64``), untouched.
"""
import asyncio
import os
from fractions import Fraction

import av
from av.video.frame import PictureType
from PIL import Image

MAXPX = int(os.environ.get("OBS_VID_MAXPX", "640"))      # longest encoded edge (>= 640 = full native 640x360)
BITRATE = int(os.environ.get("OBS_VID_BITRATE", "800000"))
FPS = int(os.environ.get("OBS_VID_FPS", "12"))           # nominal; pump is event-driven
GOP = max(1, FPS * 2)                                    # keyframe at least every ~2s


def _even(x: int) -> int:
    """H.264 / yuv420p needs even dimensions."""
    return x - (x % 2)


def _target_size(w: int, h: int, maxpx: int) -> tuple[int, int]:
    if max(w, h) <= maxpx:
        return _even(w), _even(h)
    s = maxpx / max(w, h)
    return _even(int(w * s)), _even(int(h * s))


def _codec_string(annexb: bytes) -> str:
    """Derive the WebCodecs ``codec`` (``avc1.PPCCLL``) from an Annex-B keyframe's
    SPS NAL (profile_idc, constraint flags, level_idc). Falls back to constrained
    baseline 3.1 if no SPS is found."""
    i, n = 0, len(annexb)
    while i < n - 4:
        if annexb[i] == 0 and annexb[i + 1] == 0:
            if annexb[i + 2] == 1:
                hdr = i + 3
            elif annexb[i + 2] == 0 and annexb[i + 3] == 1:
                hdr = i + 4
            else:
                i += 1
                continue
            if hdr + 3 < n and (annexb[hdr] & 0x1F) == 7:  # SPS
                return "avc1.%02X%02X%02X" % (
                    annexb[hdr + 1], annexb[hdr + 2], annexb[hdr + 3])
            i = hdr
        else:
            i += 1
    return "avc1.42C01F"


class H264Encoder:
    """One drone's libx264 encoder, lazily sized from its first frame.

    ``encode`` returns a list of ``(is_keyframe, annexb_bytes)`` (possibly empty).
    ``codec_string`` is filled in from the first keyframe for the browser's
    ``VideoDecoder.configure``.
    """

    def __init__(self, maxpx: int = MAXPX, bitrate: int = BITRATE, fps: int = FPS) -> None:
        self._maxpx, self._bitrate, self._fps = maxpx, bitrate, fps
        self._cc: av.CodecContext | None = None
        self._tw = self._th = 0
        self._pts = 0
        self.codec_string: str | None = None

    def _ensure(self, w: int, h: int) -> None:
        if self._cc is not None:
            return
        self._tw, self._th = _target_size(w, h, self._maxpx)
        cc = av.CodecContext.create("libx264", "w")
        cc.width, cc.height = self._tw, self._th
        cc.pix_fmt = "yuv420p"
        cc.framerate = Fraction(self._fps, 1)
        cc.time_base = Fraction(1, self._fps)
        cc.bit_rate = self._bitrate
        # ultrafast + zerolatency => no B-frames, no lookahead: one packet out per
        # frame in, minimal CPU and latency. baseline keeps SPS/PPS in-stream and
        # matches the avc1.42.. codec string the browser configures with.
        cc.options = {
            "preset": "ultrafast", "tune": "zerolatency",
            "profile": "baseline", "level": "31", "g": str(GOP),
        }
        self._cc = cc

    def encode(self, w: int, h: int, rgb: bytes, force_key: bool = False
               ) -> list[tuple[bool, bytes]]:
        self._ensure(w, h)
        frame = av.VideoFrame.from_image(Image.frombytes("RGB", (w, h), rgb))
        frame = frame.reformat(width=self._tw, height=self._th, format="yuv420p")
        frame.pts = self._pts
        self._pts += 1
        if force_key:
            frame.pict_type = PictureType.I
        out: list[tuple[bool, bytes]] = []
        for pkt in self._cc.encode(frame):
            b = bytes(pkt)
            if self.codec_string is None and pkt.is_keyframe:
                self.codec_string = _codec_string(b)
            out.append((bool(pkt.is_keyframe), b))
        return out


class VideoHub:
    """Background encode pump + pub/sub fan-out over a camera feed.

    ``pump`` encodes each drone's newest frame once (regardless of how many
    clients are watching) and pushes ``(id, is_key, codec, data)`` to every
    subscriber queue. A new subscriber asks the pump to force an IDR on every
    drone, so its tiles light up within a frame rather than waiting up to a GOP.
    Encoding is skipped entirely while nobody is connected.

    ``cameras`` is duck-typed: it needs ``seq(i) -> int`` and
    ``raw(i) -> (w, h, rgb_bytes) | None``.
    """

    def __init__(self, cameras, n: int, *, maxpx: int = MAXPX,
                 bitrate: int = BITRATE, fps: int = FPS, interval: float = 0.05) -> None:
        self._cameras = cameras
        self._n = n
        self._interval = interval
        self._encoders = {i: H264Encoder(maxpx, bitrate, fps) for i in range(n)}
        self._last: dict[int, int] = {}        # i -> last seq encoded
        self._subs: set[asyncio.Queue] = set()
        self._force: set[int] = set()          # drones owed a forced IDR
        self._pump_task: asyncio.Task | None = None

    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=240)
        self._subs.add(q)
        self._force.update(range(self._n))     # prime keyframes for the newcomer
        # Self-start the pump on the first viewer (no startup hook needed, and no
        # encode cost until someone is watching).
        if self._pump_task is None or self._pump_task.done():
            self._pump_task = asyncio.ensure_future(self.pump())
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        self._subs.discard(q)

    async def pump(self) -> None:
        while True:
            if not self._subs:                 # nobody watching => no encode cost
                await asyncio.sleep(self._interval)
                continue
            for i in range(self._n):
                seq = self._cameras.seq(i)
                if not seq:
                    continue
                force = i in self._force
                if self._last.get(i) == seq and not force:
                    continue
                raw = self._cameras.raw(i)
                if raw is None:
                    continue
                w, h, data = raw
                try:
                    packets = await asyncio.to_thread(
                        self._encoders[i].encode, w, h, data, force)
                except Exception:
                    continue
                self._last[i] = seq
                self._force.discard(i)
                codec = self._encoders[i].codec_string
                for is_key, b in packets:
                    self._broadcast(i, is_key, codec, b)
            await asyncio.sleep(self._interval)

    def _broadcast(self, i: int, is_key: bool, codec: str | None, data: bytes) -> None:
        msg = (i, is_key, codec, data)
        for q in list(self._subs):
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                try:                            # live video: drop oldest, keep newest
                    q.get_nowait()
                    q.put_nowait(msg)
                except Exception:
                    pass
