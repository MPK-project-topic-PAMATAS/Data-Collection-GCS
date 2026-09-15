"""Native-rate MTi structured (decoded) recorder — independent of camera sync.

Polls MTiReceiver.get_new_since() and pushes every native MTi packet
(~200 Hz) straight into DatasetWriter.write_mti_sample(), which appends to
the HDF5 sensor stream. This replaces the old behaviour where MTi HDF5
samples were only written once per matched camera frame (downsampled to
~camera FPS, and dropped entirely whenever camera<->IMU sync failed).

Python backend (MTiReceiver): loss-free. get_new_since() drains the
receiver's own ring buffer, so no sample is skipped between polls as long as
the poll interval keeps the buffer from wrapping (buffer_size=300 at ~200 Hz
gives >1s of headroom against a 20 ms poll).

Native (C++) backend (NativeMTiReceiver via nativeadapter.py): no
buffer-drain call exists in the pybind11 surface, so this poller falls back
to dedup-by-timestamp polling of get_frame() — the same pattern already used
by RadarBagPoller. This fallback is best-effort only: a sample can be missed
if two native packets arrive within one poll interval. Do not treat native
MTi HDF5 output as loss-free without verifying the C++ manager's own
buffering.
"""

from __future__ import annotations

import threading
import time

from dataset.clock import capture_wall_offset_ns, mono_to_wall_ns


class MTiStructuredPoller:
    def __init__(self, mti, writer, poll_interval_s: float = 0.02):
        self._mti = mti
        self._writer = writer
        self._poll = poll_interval_s
        self._last_ns = 0
        self._running = False
        self._thread = None
        self._use_buffer = hasattr(mti, "get_new_since")
        self._wall_offset_ns = 0

    def start(self):
        self._running = True
        self._wall_offset_ns = capture_wall_offset_ns()
        # MTiReceiver runs continuously from app startup, not just while
        # recording — its ring buffer can already hold up to buffer_size
        # (300) samples from before this poller ever starts. Without this,
        # the first get_new_since(0) call drains that whole pre-recording
        # backlog straight into the session's HDF5 file (confirmed by an
        # external dataset audit: ~128 rows timestamped before the session
        # start). Skip everything already buffered/latest at this moment —
        # only samples arriving from here on belong to this session.
        if self._use_buffer:
            self._last_ns = time.monotonic_ns()
        else:
            latest = self._mti.get_frame()
            self._last_ns = latest.timestamp_ns if latest is not None else 0
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="mti-struct"
        )
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                if self._use_buffer:
                    for frame in self._mti.get_new_since(self._last_ns):
                        self._writer.write_mti_sample(
                            frame,
                            host_rx_timestamp_ns=mono_to_wall_ns(
                                frame.timestamp_ns, self._wall_offset_ns
                            ),
                        )
                        self._last_ns = frame.timestamp_ns
                else:
                    frame = self._mti.get_frame()
                    if frame is not None and frame.timestamp_ns != self._last_ns:
                        self._last_ns = frame.timestamp_ns
                        self._writer.write_mti_sample(
                            frame,
                            host_rx_timestamp_ns=mono_to_wall_ns(
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
