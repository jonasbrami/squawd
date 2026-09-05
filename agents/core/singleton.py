"""Single-instance guard for the pilot process."""
import fcntl
import os

DEFAULT_LOCK_PATH = os.environ.get("PILOT_LOCK", "/tmp/squawd_pilot.lock")


def acquire_singleton_lock(path: str = DEFAULT_LOCK_PATH):
    """Take an exclusive, non-blocking lock; raise SystemExit if another process
    already holds it. Returns the open file object, which the CALLER MUST keep
    referenced for the process lifetime — the lock releases when it is closed or
    the process exits (so a crashed/killed process frees it automatically)."""
    f = open(path, "w")
    try:
        fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.close()
        raise SystemExit(
            f"pilot already running (lock held: {path}); stop it before starting another")
    f.truncate(0)
    f.write(str(os.getpid()))
    f.flush()
    return f
