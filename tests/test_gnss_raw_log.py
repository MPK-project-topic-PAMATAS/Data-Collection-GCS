"""Regression test for a confirmed deadlock: GNSSReceiver.start_raw_log()
acquired self._raw_lock and then called self.stop_raw_log(), which tried to
acquire the same non-reentrant threading.Lock again — a permanent
self-deadlock. This froze the Qt main thread solid on every REC press
(confirmed via a live py-spy dump showing MainThread stuck at
gnss/receiver.py:168 inside stop_raw_log(), called from start_raw_log()),
forcing the user to force-quit the whole application. Fixed by using an
RLock. This test must complete quickly, not hang, on both the very first
call (no prior log open) and a second call that rotates an existing one.

Uses the real GNSSReceiver() constructor (not __new__ + manual attribute
patching) so it actually exercises whatever lock type __init__ creates —
a test that substitutes its own lock would pass even with the bug present.
"""

import threading

from gnss.receiver import GNSSReceiver


def test_start_raw_log_does_not_deadlock_on_first_call(tmp_path):
    gnss = GNSSReceiver()  # no serial hardware touched by __init__ itself

    assert _call_with_timeout(
        lambda: gnss.start_raw_log(str(tmp_path / "gnss.nmea.log"))
    ), "start_raw_log() hung — deadlock regression"
    assert gnss._raw_log_file is not None
    gnss.stop_raw_log()


def _call_with_timeout(fn, timeout_s=2.0):
    """Run fn() on a daemon thread and report whether it finished in time —
    never blocks the calling (test) thread itself, even if fn deadlocks."""
    done = threading.Event()

    def call():
        fn()
        done.set()

    t = threading.Thread(target=call, daemon=True)
    t.start()
    t.join(timeout=timeout_s)
    return done.is_set()


def test_start_raw_log_does_not_deadlock_when_rotating(tmp_path):
    """The exact call shape from the crash: start_raw_log() called while a
    log from a previous session is still open, forcing it through its own
    internal stop_raw_log() call under the same lock. Both calls run on
    bounded background threads — if either deadlocks, this test fails
    instead of hanging the whole suite."""
    gnss = GNSSReceiver()

    assert _call_with_timeout(
        lambda: gnss.start_raw_log(str(tmp_path / "old.log"))
    ), "first start_raw_log() hung — deadlock regression"
    assert gnss._raw_log_file is not None

    assert _call_with_timeout(
        lambda: gnss.start_raw_log(str(tmp_path / "new.log"))
    ), "start_raw_log() hung while rotating — deadlock regression"

    gnss.stop_raw_log()
