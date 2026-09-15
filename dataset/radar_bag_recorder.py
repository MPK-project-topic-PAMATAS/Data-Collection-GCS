"""ROS1 ``.bag`` recorder for the TI AWR1843BOOST radar (RoadBag-compatible).

Uses the pure-Python ``rosbags`` library (already installed in the ``mpk`` venv)
to write a standard ROS1 bag containing, per radar frame:
  * ``/radar/points`` -> ``sensor_msgs/PointCloud2``   (x, y, z, velocity_mps)
  * ``/clock``        -> ``rosgraph_msgs/Clock``

The bag is written alongside the existing per-frame radar JSON/NPY files during
a recording session, so the radar stream is available in a ROS ingestion tool
(e.g. Autoware RoadBag) in addition to the native JSON format.

Timestamps: radar frames carry host *monotonic* nanosecond timestamps.  ROS bag
time is wall/epoch time, so ``open()`` captures ``wall_offset = time.time_ns() -
time.monotonic_ns()`` at session start and applies it to every frame.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from rosbags.rosbag1 import Writer as _RosbagWriter
from rosbags.typesys import Stores, get_typestore

# PointCloud2 field layout: x, y, z (float32, 0/4/8) + velocity (float32, 12).
_POINT_STEP = 16
_FIELDS = (
    ("x", 0, 7, 1),
    ("y", 4, 7, 1),
    ("z", 8, 7, 1),
    ("velocity_mps", 12, 7, 1),
)


class RadarBagRecorder:
    """Writes every radar frame it is given to a ROS1 ``.bag`` (not thread-safe;
    call from a single producer thread)."""

    def __init__(self):
        self._writer = None
        self._clock_conn = None
        self._pc_conn = None
        self._ts = None
        self._wall_offset_ns = 0
        self._frame_count = 0

    def open(self, path: str) -> None:
        """Open a bag and register the radar connections."""
        self._ts = get_typestore(Stores.ROS1_NOETIC)
        self._pc_conn = None
        self._clock_conn = None
        self._wall_offset_ns = time.time_ns() - time.monotonic_ns()
        self._frame_count = 0

        self._writer = _RosbagWriter(path)
        self._writer.open()
        self._pc_conn = self._writer.add_connection(
            "/radar/points", "sensor_msgs/msg/PointCloud2",
            typestore=self._ts,
        )
        self._clock_conn = self._writer.add_connection(
            "/clock", "rosgraph_msgs/msg/Clock",
            typestore=self._ts,
        )

    def record(self, frame) -> None:
        """Write one radar frame to the bag.

        ``frame`` is either a ``radar.RadarFrame`` (Python) or a native radar
        frame exposing ``points`` (iterable of ``.x/.y/.z/.velocity_mps``),
        ``timestamp_ns``, ``frame_id``.
        """
        if self._writer is None:
            return
        T = self._ts.types
        wall = frame.timestamp_ns + self._wall_offset_ns
        sec, nsec = wall // 1_000_000_000, wall % 1_000_000_000
        stamp = T["builtin_interfaces/msg/Time"](
            sec=int(sec), nanosec=int(nsec)
        )

        points = getattr(frame, "points", None) or []
        n = len(points)
        if n:
            xyz = np.empty((n, 4), dtype=np.float32)
            for i, p in enumerate(points):
                xyz[i, 0] = getattr(p, "x", 0.0)
                xyz[i, 1] = getattr(p, "y", 0.0)
                xyz[i, 2] = getattr(p, "z", 0.0)
                xyz[i, 3] = getattr(p, "velocity_mps", 0.0)
            data = xyz.tobytes()
        else:
            data = b""
        data = np.frombuffer(data, dtype=np.uint8)

        header = T["std_msgs/msg/Header"](seq=0, stamp=stamp, frame_id="radar")
        fields = [
            T["sensor_msgs/msg/PointField"](
                name=name, offset=off, datatype=dt, count=cnt
            )
            for name, off, dt, cnt in _FIELDS
        ]
        pc = T["sensor_msgs/msg/PointCloud2"](
            header=header, height=1, width=n, fields=fields,
            is_bigendian=False, point_step=_POINT_STEP,
            row_step=n * _POINT_STEP, data=data, is_dense=True,
        )
        self._writer.write(
            self._pc_conn, wall, self._ts.serialize_ros1(pc, "sensor_msgs/msg/PointCloud2")
        )

        clock = T["rosgraph_msgs/msg/Clock"](clock=stamp)
        self._writer.write(
            self._clock_conn, wall, self._ts.serialize_ros1(clock, "rosgraph_msgs/msg/Clock")
        )
        self._frame_count += 1

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def close(self) -> None:
        if self._writer is not None:
            try:
                self._writer.close()
            finally:
                self._writer = None


class RadarBagPoller:
    """Records every radar frame independently of camera sync.

    A daemon thread drains ``radar.get_new_since()`` when available (the
    Python ``RadarReceiver``'s ring buffer) so every frame that arrived
    between polls is recorded, not just whichever one happened to be latest
    at poll time — the native ``NativeRadarReceiver`` facade has no such
    buffer, so it falls back to ``get_frame()`` deduped by ``timestamp_ns``
    (same fallback used by ``RadarStructuredPoller``/``GNSSStructuredPoller``).

    Using ``get_frame()`` unconditionally used to silently drop frames
    whenever this thread was starved for longer than one radar frame period
    (150 ms) by the camera/LiDAR/HDF5 write load elsewhere in the process —
    ``get_frame()`` only ever returns the single newest frame, so anything
    produced during the stall was gone by the next poll. That showed up as
    the ``.bag`` having noticeably fewer messages than the structured JSON
    recorder over the same session (which already used ``get_new_since()``).

    This guarantees a ``.bag`` even when no camera frame matches (radar runs at
    ~6.6 Hz vs camera ~30 Hz, so the independent stream captures every frame).
    """

    def __init__(self, radar, recorder: RadarBagRecorder, poll_interval_s: float = 0.05):
        self._radar = radar
        self._recorder = recorder
        self._poll = poll_interval_s
        self._last_ns = 0
        self._running = False
        self._thread = None
        self._use_buffer = hasattr(radar, "get_new_since")

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="radar-bag")
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                if self._use_buffer:
                    frames = self._radar.get_new_since(self._last_ns)
                else:
                    latest = self._radar.get_frame()
                    frames = (
                        [latest]
                        if latest is not None and latest.timestamp_ns != self._last_ns
                        else []
                    )
            except Exception:
                frames = []
            for frame in frames:
                self._last_ns = frame.timestamp_ns
                try:
                    self._recorder.record(frame)
                except Exception:
                    self._running = False
                    raise
            time.sleep(self._poll)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    def close(self):
        self.stop()
        self._recorder.close()
