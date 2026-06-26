import pytest

from agents.core.singleton import acquire_singleton_lock


def test_second_acquire_is_refused(tmp_path):
    p = str(tmp_path / "swarm.lock")
    f1 = acquire_singleton_lock(p)
    try:
        with pytest.raises(SystemExit):
            acquire_singleton_lock(p)       # second process/handle is blocked
    finally:
        f1.close()


def test_lock_reusable_after_release(tmp_path):
    p = str(tmp_path / "swarm.lock")
    f1 = acquire_singleton_lock(p)
    f1.close()                              # release (mimics the holder exiting)
    f2 = acquire_singleton_lock(p)          # now a fresh process can take it
    f2.close()


def test_lock_file_records_pid(tmp_path):
    import os
    p = str(tmp_path / "swarm.lock")
    f = acquire_singleton_lock(p)
    try:
        with open(p) as r:
            assert r.read().strip() == str(os.getpid())
    finally:
        f.close()
