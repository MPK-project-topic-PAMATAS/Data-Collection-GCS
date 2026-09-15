"""Native-rate radar structured (decoded) recorder — independent of camera sync.

Polls RadarReceiver.get_new_since() and pushes every native radar frame
(points + range-Doppler heatmap + range-profile, at the radar's own ~6.6 Hz
frame rate) into DatasetWriter.write_radar_frame(), which persists the JSON
points file, heatmap NPY, range-profile NPY, and a metadata/radar Parquet
index row. This is in addition to RadarBagPoller (dataset/radar_bag_recorder.py),
which already records points-only to a ROS .bag independently of camera —
this poller extends that same independence to the richer structured output
(heatmap/range-profile) that used to only be written for camera-matched
frames.

See dataset/mti_structured_recorder.py for the identical Python-buffer vs.
native-backend-fallback discussion; the same caveat applies here.
"""

from __future__ import annotations

import threading
import time

from dataset.clock import capture_wall_offset_ns, mono_to_wall_ns


class RadarStructuredPoller:
    def __init__(self, radar, writer, poll_interval_s: float = 0.02):
        self._radar = radar
        self._writer = writer
        self._poll = poll_interval_s
        self._last_ns = 0
        self._running = False
        self._thread = None
        self._use_buffer = hasattr(radar, "get_new_since")
        self._wall_offset_ns = 0

    def start(self):
        self._running = True
        self._wall_offset_ns = capture_wall_offset_ns()
        # RadarReceiver runs continuously from app startup, not just while
        # recording — skip whatever is already buffered/latest at this
        # moment so a pre-recording backlog never leaks into the session's
        # structured output (confirmed by an external dataset audit: ~57
        # radar rows timestamped before the session start). See the
        # identical fix/comment in mti_structured_recorder.py.
        if self._use_buffer:
            self._last_ns = time.monotonic_ns()
        else:
            latest = self._radar.get_frame()
            self._last_ns = latest.timestamp_ns if latest is not None else 0
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="radar-struct"
        )
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                if self._use_buffer:
                    for frame in self._radar.get_new_since(self._last_ns):
                        self._writer.write_radar_frame(
                            frame,
                            wall_timestamp_ns=mono_to_wall_ns(
                                frame.timestamp_ns, self._wall_offset_ns
                            ),
                        )
                        self._last_ns = frame.timestamp_ns
                else:
                    frame = self._radar.get_frame()
                    if frame is not None and frame.timestamp_ns != self._last_ns:
                        self._last_ns = frame.timestamp_ns
                        self._writer.write_radar_frame(
                            frame,
                            wall_timestamp_ns=mono_to_wall_ns(
                                frame.timestamp_ns, self._wall_offset_ns
                            ),
                        )
            except Exception:
                pass
            time.sleep(self._poll)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
