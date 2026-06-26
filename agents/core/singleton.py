"""Single-instance guard for the swarm agents process.

Running two `agents/swarm/run.py` processes at once is a foot-gun: each starts
its own Commander, and BOTH react to every drone report on /swarm/report/*, so
they amplify each other into a storm of dispatches with no human input. An
exclusive advisory file lock makes a second agents process refuse to start
instead of silently stacking another swarm.
"""
import fcntl
import os

DEFAULT_LOCK_PATH = os.environ.get("SWARM_LOCK", "/tmp/dronebot_swarm_agents.lock")


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
            f"swarm agents already running (lock held: {path}); refusing to start a "
            "second Commander. Stop the other process first "
            "(pkill -f '[a]gents/swarm/run.py').")
    f.truncate(0)
    f.write(str(os.getpid()))
    f.flush()
    return f
