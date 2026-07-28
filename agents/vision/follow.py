"""vision/follow.py — lock lifecycle skeleton (ICD §6.8), ADAPTED from
perception-lab's FollowTarget @ 26e9431 (not vendored whole):

- sim time injected (never time.time());
- deadlines derived from TrackerConfig (coast_s/lost_s ÷ dt_nominal_s — ONE
  owner of the lost constants);
- hits pass to the EKF RAW (the lab's 0.5-EMA output is display-only);
- LOST persists until the owner drops/rebinds (no auto-expire to IDLE);
- bearing math uses full pinhole (perception/projection), not the lab's 75deg
  linear webcam approximation.
"""
from agents.perception.projection import pixel_to_angles

IDLE, TRACKING, COAST, LOST = "IDLE", "TRACKING", "COAST", "LOST"

# health mapping onto VisionContacts states (ICD §6.8)
HEALTH = {TRACKING: "MEASURED", COAST: "COASTING", LOST: "LOST", IDLE: "LOST"}


class FollowTarget:
    def __init__(self, *, dt_nominal_s: float = 0.2, coast_s: float = 1.0,
                 lost_s: float = 2.0) -> None:
        self.coast_frames = max(1, round(coast_s / dt_nominal_s))
        self.lost_frames = max(self.coast_frames + 1,
                               round(lost_s / dt_nominal_s))
        self.clear()

    def clear(self) -> None:
        self.locked = False
        self.cls = None
        self.x = self.y = 0.0
        self.conf = 0.0
        self.misses = 0
        self.status = IDLE

    def lock(self, cls: str, x: float, y: float, conf: float = 0.0) -> None:
        self.locked = True
        self.cls, self.x, self.y, self.conf = cls, x, y, conf
        self.misses = 0
        self.status = TRACKING

    def step(self, hit, frame_w: int, frame_h: int):
        """Advance one inference. `hit` is (cx, cy, conf) RAW (no smoothing
        here — the EKF owns filtering) or None."""
        if not self.locked:
            return
        if hit:
            self.x, self.y, self.conf = hit[0], hit[1], hit[2]
            self.misses = 0
            self.status = TRACKING
        else:
            self.misses += 1
            if self.misses > self.lost_frames:
                self.status = LOST
                return
            if self.misses > self.coast_frames:
                self.status = COAST

    def health(self) -> str:
        return HEALTH[self.status]

    def bearing_elevation(self, frame_w: int, frame_h: int) -> tuple[float, float]:
        ax, ay = pixel_to_angles(self.x, self.y, frame_w, frame_h)
        return ax, ay

    def snapshot(self) -> dict:
        return {"status": self.status, "health": self.health(),
                "cls": self.cls, "misses": self.misses,
                "x": self.x, "y": self.y, "conf": self.conf}
