"""Native-rate GNSS structured recorder — independent of camera sync.

Polls GNSSReceiver.get_new_since() and pushes every parsed GGA/RMC fix into
DatasetWriter.write_gnss_fix(), which appends to a metadata/gnss Parquet
index. Previously GNSS was only persisted as whatever GNSSReceiver.latest()
happened to return at the moment a camera frame matched — not buffered, not
independently timed, and lost entirely between matched camera frames. GNSS
has no independent recorder for the native/C++ backend today (NativeGNSSReceiver
exposes only latest(), no buffer) — this poller falls back to dedup-by-timestamp
polling of latest() in that case, matching the same best-effort caveat as the
MTi/radar structured pollers.
"""

from __future__ import annotations

import threading
import time

from dataset.clock import capture_wall_offset_ns, mono_to_wall_ns


class GNSSStructuredPoller:
    def __init__(self, gnss, writer, poll_interval_s: float = 0.05):
        self._gnss = gnss
        self._writer = writer
        self._poll = poll_interval_s
        self._last_ns = 0
        self._running = False
        self._thread = None
        self._use_buffer = hasattr(gnss, "get_new_since")
        self._wall_offset_ns = 0

    def start(self):
        self._running = True
        self._wall_offset_ns = capture_wall_offset_ns()
        # GNSSReceiver runs continuously from app startup, not just while
        # recording — skip whatever is already buffered/latest at this
        # moment so a pre-recording backlog never leaks into the session's
        # structured output. See the identical fix/comment in
        # mti_structured_recorder.py.
        if self._use_buffer:
            self._last_ns = time.monotonic_ns()
        else:
            latest = self._gnss.latest()
            self._last_ns = latest["timestamp_ns"] if latest is not None else 0
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="gnss-struct"
        )
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                if self._use_buffer:
                    for fix in self._gnss.get_new_since(self._last_ns):
                        self._writer.write_gnss_fix(
                            fix,
                            wall_timestamp_ns=mono_to_wall_ns(
                                fix["timestamp_ns"], self._wall_offset_ns
                            ),
                        )
                        self._last_ns = fix["timestamp_ns"]
                else:
                    fix = self._gnss.latest()
                    if fix is not None and fix.get("timestamp_ns", 0) != self._last_ns:
                        self._last_ns = fix["timestamp_ns"]
                        self._writer.write_gnss_fix(
                            fix,
                            wall_timestamp_ns=mono_to_wall_ns(
                                fix["timestamp_ns"], self._wall_offset_ns
                            ),
                        )
            except Exception:
                pass
            time.sleep(self._poll)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
