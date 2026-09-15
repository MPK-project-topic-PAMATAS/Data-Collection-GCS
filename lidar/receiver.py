"""Ouster OS-0-64 lidar receiver.

A background thread opens a live sensor via the ouster SDK and:
  * maintains the latest + a nearest-match ring buffer of LidarFrames,
  * computes a downsampled XYZ point cloud for dashboard visualization,
  * writes raw UDP packets to a single pcap (with a single companion
    metadata JSON) while a recording is active.

Exactly one pcap and one JSON are produced per recording session — never one
JSON per frame/scan/packet. The pcap is the primary raw Ouster dataset; the
JSON is session-level sensor configuration/metadata only (beam angles,
calibration, lidar mode), required to make the pcap replayable. Individual
scans/point clouds are derived from this pcap + JSON offline, on demand, by
``dataset.reader.iter_lidar_frames`` — never pre-generated per-frame.

Recording follows the SDK pattern: open_source collates LidarFrames, and each
frame is expanded back into its raw packets (frame_to_packets) which are
appended to a pcap file opened with ``record_initialize``.  A pcap alone is
not self-describing, so the sensor metadata is saved as ``<name>_0.json``
next to it (matching ouster-cli output that pcap readers, and
``open_source``'s own auto-discovery, expect).
"""

from __future__ import annotations

import os
import threading
import time
from collections import deque

import numpy as np

from lidar.frame import LidarFrame
from config.settings import LIDAR_HOSTNAME, LIDAR_STREAM_TIMEOUT_NS


class LidarReceiver:
    """Live OS-0-64 streamer with nearest-match lookup and pcap recorder."""

    def __init__(self, hostname: str | None = None, buffer_size: int = 200):
        self._hostname = hostname or LIDAR_HOSTNAME
        self._buffer: deque[LidarFrame] = deque(maxlen=buffer_size)
        self._latest: LidarFrame | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False
        self._frame_count = 0
        self._rate = 0.0
        self._last_frame_ns = 0
        self._start_time = 0.0
        self._error_count = 0
        self._last_error = ""
        self._prev_frame_id: int | None = None
        self._dropped_frames = 0

        self._source = None
        self._info = None
        self._xyz_lut = None

        # pcap recording state — exactly one pcap + one config JSON per session
        self._pcap_handle = None
        self._pcap_path = None
        self._config_json_path = None
        self._pcap_packets = 0
        self._sensor_name = hostname or LIDAR_HOSTNAME
        self._diagnosed = False

    # ── lifecycle ────────────────────────────────────────────────────

    def start(self):
        from ouster import sdk
        from ouster.sdk.core import XYZLut

        # A single live URL returns the SDK's ``Singler`` wrapper.  Its
        # ``single_iter(0)`` method is the supported iterable interface.
        self._source = sdk.open_source([self._hostname], collate=True)
        self._info = self._source.sensor_info[0]
        try:
            self._sensor_name = self._info.prod_line
        except Exception:
            pass
        self._xyz_lut = XYZLut(self._info, use_extrinsics=False)
        self._running = True
        self._start_time = time.monotonic_ns()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        try:
            # A collated source (collate=True) is iterated directly to yield
            # FrameSets; single_iter is only for the uncollated source.
            for frameset in self._source:
                if not self._running:
                    break
                for frame in frameset.valid_frames():
                    if frame is None:
                        continue
                    try:
                        self._ingest_frame(frame)
                    except Exception as exc:
                        self._record_error(str(exc))
        except Exception as exc:
            self._record_error(f"source read failed: {exc}")

    def _record_error(self, error: str):
        with self._lock:
            self._error_count += 1
            self._last_error = error

    def _print_diag(self, lf, now):
        """One-time startup diagnostic of the organized scan layout.

        Confirms the sensor is exposing all 64 channels as a (H, W) image and
        how many valid (non-zero) XYZ points that yields after flattening, so
        channel loss vs. physical no-returns can be told apart.
        """
        try:
            from ouster.sdk.core import ChanField
            rng = lf.field(ChanField.RANGE) if lf.has_field(ChanField.RANGE) else None
        except Exception:
            rng = None
        organized = tuple(rng.shape) if rng is not None else None
        xyz = self._xyz_lut(rng).astype(np.float32) if rng is not None else None
        valid = 0
        if xyz is not None:
            m = np.isfinite(xyz).all(axis=-1) & (np.abs(xyz).sum(axis=-1) > 0)
            valid = int(m.sum())
        print(
            f"[lidar] {self._sensor_name}: organized range shape={organized} "
            f"(H={organized[0]} channels, W={organized[1]} columns) -> "
            f"{valid} valid XYZ points"
        )
        print(
            f"[lidar] expected 64 channels; all valid points from every channel "
            f"are flattened into a single Nx3 cloud for display/recording."
        )

    def _ingest_frame(self, lf):
        now = time.monotonic_ns()
        # Report the organized scan dimensions once on startup so we can
        # confirm all 64 channels (organized H) survive the XY Z flatten.
        if not self._diagnosed:
            self._diagnosed = True
            self._print_diag(lf, now)
        self._track_frame_gap(lf.frame_id)

        points_full, reflect = _xyzi_from_frame(lf, self._xyz_lut)
        # Downsample for dashboard rendering (strided columns).
        cols = points_full.shape[0]
        step = max(1, cols // 512)
        points = points_full[::step] if cols > 0 else points_full

        lf_out = LidarFrame(
            frame_id=lf.frame_id,
            timestamp_ns=now,
            points=points,
            points_full=points_full,
            reflectivity=reflect,
            num_points=int(points_full.shape[0]),
            complete=bool(lf.complete()),
        )

        with self._lock:
            self._latest = lf_out
            self._buffer.append(lf_out)
            self._frame_count += 1
            self._last_frame_ns = now
            elapsed = now - self._start_time
            self._rate = (
                self._frame_count / (elapsed / 1e9) if elapsed > 0 else 0.0
            )

        self._write_pcap_frame(lf)

    def _track_frame_gap(self, frame_id: int):
        """Update the dropped-frame count from the sensor's own frame_id
        sequence. Unit-testable independent of the SDK ingest thread."""
        if self._prev_frame_id is not None:
            gap = frame_id - self._prev_frame_id - 1
            if gap > 0:
                self._dropped_frames += gap
        self._prev_frame_id = frame_id

    # ── recording ────────────────────────────────────────────────────

    def start_pcap(self, path: str) -> str:
        """Open a pcap file and begin appending raw lidar packets.

        Exactly one pcap and one companion metadata JSON (``<stem>_0.json``)
        are produced for the whole session — never one JSON per frame.
        Capture begins with the next live frame.
        """
        try:
            self.stop_pcap()
        except Exception:
            pass
        import ouster.sdk._bindings.pcap as _pcap
        self._pcap_path = path
        self._pcap_handle = _pcap.record_initialize(path, 9000, False)
        self._pcap_packets = 0
        stem = os.path.splitext(path)[0]
        md_path = f"{stem}_0.json"
        with open(md_path, "w") as f:
            f.write(self._info.to_json_string())
        self._config_json_path = md_path
        return path

    def _write_pcap_frame(self, lf):
        if not self._pcap_handle or self._info is None:
            return
        try:
            import ouster.sdk._bindings.pcap as _pcap
            from ouster.sdk.util import frame_to_packets
            packets = frame_to_packets(lf, self._info)
            port = self._info.config.udp_port_lidar
            for p in packets:
                ts = (p.host_timestamp / 1e9) if p.host_timestamp else time.time()
                _pcap.record_packet(
                    self._pcap_handle, "127.0.0.1", "127.0.0.1", port, port,
                    p.buf, ts,
                )
                self._pcap_packets += 1
        except Exception:
            pass  # recording must never crash the acquisition thread

    def stop_pcap(self):
        if self._pcap_handle is not None:
            try:
                import ouster.sdk._bindings.pcap as _pcap
                _pcap.record_uninitialize(self._pcap_handle)
            except Exception:
                pass
            self._pcap_handle = None
        self._pcap_path = None

    @property
    def pcap_path(self) -> str | None:
        """Path to the single session pcap, or None before recording."""
        return self._pcap_path

    @property
    def config_json_path(self) -> str | None:
        """Path to the single session-level metadata/configuration JSON
        written alongside the pcap by start_pcap(), or None before a
        recording has started."""
        return self._config_json_path

    # ── accessors ────────────────────────────────────────────────────

    def get_frame(self) -> LidarFrame | None:
        with self._lock:
            return self._latest

    def get_nearest(self, target_ts: int) -> LidarFrame | None:
        with self._lock:
            if not self._buffer:
                return None
            best = None
            best_delta = float("inf")
            for f in reversed(self._buffer):
                delta = abs(f.timestamp_ns - target_ts)
                if delta >= best_delta:
                    break
                best = f
                best_delta = delta
            return best

    @property
    def rate(self) -> float:
        with self._lock:
            return self._rate

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._frame_count

    @property
    def dropped_frames(self) -> int:
        """Frames missing per gaps in the sensor's own frame_id sequence."""
        with self._lock:
            return self._dropped_frames

    @property
    def pcap_packets(self) -> int:
        return self._pcap_packets

    @property
    def sensor_info(self):
        return self._info

    @property
    def sensor_name(self) -> str:
        return self._sensor_name

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    @property
    def error_count(self) -> int:
        with self._lock:
            return self._error_count

    @property
    def streaming(self) -> bool:
        with self._lock:
            last = self._last_frame_ns
        return last > 0 and (
            time.monotonic_ns() - last <= LIDAR_STREAM_TIMEOUT_NS
        )

    def stop(self):
        self._running = False
        # Closing first unblocks an idle SDK iterator so the worker can exit.
        if self._source is not None:
            try:
                self._source.close()
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=3)
        self.stop_pcap()


def _xyzi_from_frame(lf, xyz_lut):
    """Return (Nx3 XYZ in metres, Nx1 reflectivity) for a LidarFrame."""
    from ouster.sdk.core import ChanField
    if lf is None:
        return np.zeros((0, 3), dtype=np.float32), None
    try:
        rng = lf.field(ChanField.RANGE) if lf.has_field(ChanField.RANGE) else None
    except Exception:
        rng = None
    if rng is None:
        return np.zeros((0, 3), dtype=np.float32), None
    xyz = xyz_lut(rng).astype(np.float32)
    valid = np.isfinite(xyz).all(axis=-1) & (np.abs(xyz).sum(axis=-1) > 0)
    xyz = xyz[valid].reshape(-1, 3)
    refl = None
    if lf.has_field(ChanField.REFLECTIVITY):
        try:
            refl = lf.field(ChanField.REFLECTIVITY).astype(np.float32)[valid]
        except Exception:
            refl = None
    return xyz, refl
