"""Dataset writer — continuous namespace with HDF5/Parquet/PNG/NPY.

Directory structure (continuous dataset root):
    dataset/
    ├── left/
    ├── right/
    ├── depth/
    ├── disparity/
    ├── confidence/
    ├── sensors/
    │   └── YYYYMMDD_HHMMSS_mti.h5
    ├── metadata/
    │   ├── frames/      (partitioned Parquet, one row per camera frame)
    │   ├── radar/       (partitioned Parquet, one row per native radar frame)
    │   ├── gnss/        (partitioned Parquet, one row per native GNSS fix)
    │   ├── sessions/    (partitioned Parquet)
    │   └── events/      (partitioned Parquet)
    ├── calibration/
    ├── manifests/
    └── raw/

Recording is per-sensor and native-rate, not camera-gated: write_camera_frame(),
write_mti_sample(), write_radar_frame(), write_gnss_fix() are independent entry
points, each callable regardless of whether any other sensor produced data at
the same moment. A missing camera frame never removes an MTi/radar/GNSS
sample, and vice versa. All I/O still happens in a single writer thread —
callers only enqueue tagged items onto a bounded queue; synchronization
(nearest-neighbour matching across sensors) is a separate, later, offline
step (see fusion/offline_sync.py), never a precondition for writing.
"""

from __future__ import annotations
import hashlib
import json
import threading
import queue
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from config.settings import (
    SAVE_DEPTH as CFG_SAVE_DEPTH,
    SAVE_DISPARITY,
    SAVE_CONFIDENCE,
    ENABLE_ZED_IMU_LOGGING,
)
from dataset.hdf5_sensor_writer import (
    HDF5SensorWriter, SensorSample,
    mti_frame_to_sample,
)
from dataset.parquet_index import FrameIndexWriter, RadarIndexWriter, GNSSIndexWriter
from dataset.session import SessionWriter
from dataset.events import EventWriter


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _frame_prefix(timestamp_ns: int, frame_id: int) -> str:
    """Generate filename prefix: YYYYMMDD_HHMMSS_nnnnnnnnn_FFFFFFFFFF"""
    dt = datetime.fromtimestamp(timestamp_ns / 1e9, tz=timezone.utc)
    date_str = dt.strftime("%Y%m%d_%H%M%S")
    nano_frac = timestamp_ns % 1_000_000_000
    return f"{date_str}_{nano_frac:09d}_{frame_id:010d}"


class DatasetWriter:
    def __init__(self, root="dataset", save_depth=CFG_SAVE_DEPTH,
                 enable_hdf5=True, enable_parquet=True,
                 enable_radar=True,
                 session_id=""):
        # No enable_lidar: DatasetWriter owns no lidar path at all — lidar
        # recording (one pcap + one config JSON) is entirely owned by
        # LidarReceiver, independent of this writer (see lidar/receiver.py).
        #
        # _dataset_root is the top-level folder passed in (e.g. "output/")
        # and never changes. self.root is the *currently active* write
        # root — until begin_session() is first called it's the same as
        # _dataset_root (so constructing a DatasetWriter and writing
        # straight away, with no session, still works — most tests do
        # exactly this). Every begin_session() switches self.root to a
        # fresh _dataset_root/<session_id>/ subfolder and recreates every
        # per-sensor directory and backend writer inside it, so each
        # session is a single, self-contained, immutable directory — no
        # session's files can ever land in, or collide with, another's.
        self._dataset_root = Path(root)
        self._save_depth = save_depth
        self._enable_radar = enable_radar
        self._save_disparity = save_depth and SAVE_DISPARITY
        self._save_confidence = save_depth and SAVE_CONFIDENCE
        self._session_id = session_id or datetime.now(
            tz=timezone.utc
        ).strftime("%Y%m%d_%H%M%S")
        # Every begin_session() must produce a session_id distinct from any
        # used before in this process — otherwise a second REC press reuses
        # the first session's folder name and its files collide with the
        # first session's (this is exactly what happened before session_id
        # was regenerated per call: three REC presses shared one ID).
        self._used_session_ids = {self._session_id}

        self._enable_hdf5 = enable_hdf5
        self._enable_parquet = enable_parquet
        self._hdf5_writer = None
        self._frame_index = None
        self._radar_index = None
        self._gnss_index = None
        self._session_writer = None
        self._event_writer = None
        self._mti_index = 0  # running MTi sample count for the active session
        # True once _open_backends() has actually run. Previously __init__
        # unconditionally created the full sensors/metadata/calibration/
        # manifests/raw/radar scaffold directly under _dataset_root and
        # opened every backend writer at GCS startup — before any REC
        # press — leaving a pile of empty folders behind whenever a session
        # was never recorded. Now that only happens the first time a write
        # is actually enqueued (_ensure_backends(), called from every
        # write_*()/add_event()) or begin_session() runs, whichever comes
        # first — so writing straight to a DatasetWriter with no session
        # still works (most tests do exactly this), but an idle GCS that
        # never presses REC touches disk only for save_calibration().
        self._backends_open = False

        # Path attributes still need to exist before the first write/session
        # (save_calibration()/set_*() and status panels read them), so this
        # only computes them — no mkdir yet.
        self._configure_dirs(self._dataset_root, create=False)

        # ── Async writer queue ────────────────────────────────────────
        self._queue = queue.Queue(maxsize=128)
        self._thread = threading.Thread(
            target=self._write_loop, daemon=True
        )
        self._thread.start()

        # ── Periodic flush ──────────────────────────────────────────────
        # FrameIndexWriter/RadarIndexWriter/GNSSIndexWriter/EventWriter only
        # write a Parquet part when their in-memory row buffer fills
        # (hundreds to a thousand rows) or on an explicit flush() — a crash
        # (native SDK segfault, power loss, kill -9) between those points
        # loses every buffered row with it. Flushing on a short timer bounds
        # that loss to a few seconds of data instead of the whole session.
        self._flush_interval_s = 5.0
        self._flush_stop = threading.Event()
        self._flush_thread = threading.Thread(
            target=self._periodic_flush_loop, daemon=True
        )
        self._flush_thread.start()

        # ── Counters ──────────────────────────────────────────────────
        self._frame_count = 0
        self._dropped = 0
        self._write_errors = 0
        self._bytes_written = 0
        self._lock = threading.Lock()
        self._calib = None

        # Statistics
        self._imu_drops = 0
        self._sync_failures = 0
        self._zed_sdk = ""
        self._mti_firmware = ""

        # Backends (HDF5/Parquet writers) are opened lazily by
        # begin_session() on the first REC press — see the comment above
        # the _configure_dirs() call in __init__.

    # ── Directory / backend (re)configuration ──────────────────────────

    def _configure_dirs(self, root: Path, create: bool = True):
        """Point every directory attribute at `root` and, unless
        create=False, create what's needed. Called once in __init__
        (root = the dataset root, before any session exists, create=False
        so no empty scaffold is left behind if REC is never pressed) and
        again by begin_session() (root = that session's own subfolder,
        create=True)."""
        self.root = root
        self.depth_dir = root / "depth"
        self.disparity_dir = root / "disparity"
        self.conf_dir = root / "confidence"
        self.sensors_dir = root / "sensors"
        self.meta_dir = root / "metadata"
        self.calib_dir = root / "calibration"
        self.manifests_dir = root / "manifests"
        self.raw_dir = root / "raw"
        self.radar_dir = root / "radar"
        self.radar_points_dir = root / "radar" / "points"
        self.radar_heatmap_dir = root / "radar" / "heatmap"
        self.radar_range_profile_dir = root / "radar" / "range_profile"
        self.extrinsics_dir = root / "calibration" / "extrinsics"
        # Lidar has no DatasetWriter-owned directory: LidarReceiver writes
        # its single pcap + single config JSON directly under raw_dir,
        # independent of this writer entirely (see lidar/receiver.py).

        dirs = [
            self.sensors_dir, self.meta_dir, self.calib_dir,
            self.manifests_dir, self.raw_dir,
        ]
        if self._enable_radar:
            dirs += [
                self.radar_dir, self.radar_points_dir, self.radar_heatmap_dir,
                self.radar_range_profile_dir,
            ]
        if self._save_depth:
            dirs.append(self.depth_dir)
        if self._save_disparity:
            dirs.append(self.disparity_dir)
        if self._save_confidence:
            dirs.append(self.conf_dir)
        if create:
            for d in dirs:
                d.mkdir(parents=True, exist_ok=True)

    def _close_backends(self):
        """Finalize whatever backends are currently open — HDF5 fully
        closed (not just flushed), Parquet writers flushed — before
        switching to a new directory. Safe to call when nothing is open."""
        if self._hdf5_writer is not None:
            self._hdf5_writer.stop()
            self._hdf5_writer = None
        if self._frame_index:
            self._frame_index.flush()
        if self._radar_index:
            self._radar_index.flush()
        if self._gnss_index:
            self._gnss_index.flush()
        if self._event_writer:
            self._event_writer.flush()
        self._backends_open = False

    def _ensure_backends(self):
        """Open backends (and create their directories) on the first write,
        if nothing has opened them yet — e.g. a caller writing directly
        without ever calling begin_session(). A no-op once backends are
        open, and a no-op inside begin_session() itself, which manages
        _backends_open via _close_backends()/_open_backends() directly."""
        if self._backends_open:
            return
        self._configure_dirs(self.root, create=True)
        self._open_backends()

    def _open_backends(self):
        """Create fresh backend writer objects pointed at the *current*
        self.sensors_dir / self.meta_dir (set by _configure_dirs first)."""
        if self._enable_hdf5:
            self._hdf5_writer = HDF5SensorWriter(
                sensors_dir=self.sensors_dir,
                enable_zed_imu=ENABLE_ZED_IMU_LOGGING,
                rotation_mode="daily",
            )
            self._hdf5_writer.start()

        if self._enable_parquet:
            self._frame_index = FrameIndexWriter(
                metadata_dir=self.meta_dir, flush_interval=1000
            )
            if self._enable_radar:
                self._radar_index = RadarIndexWriter(
                    metadata_dir=self.meta_dir, flush_interval=200
                )
            self._gnss_index = GNSSIndexWriter(
                metadata_dir=self.meta_dir, flush_interval=200
            )

        self._session_writer = SessionWriter(metadata_dir=self.meta_dir)
        self._event_writer = EventWriter(metadata_dir=self.meta_dir)
        self._backends_open = True

    # ── Public API ──────────────────────────────────────────────────

    def set_zed_sdk(self, version: str):
        self._zed_sdk = version

    def set_mti_firmware(self, firmware: str):
        self._mti_firmware = firmware

    def save_calibration(self, calib: dict):
        self._calib = calib
        calib_dir = self.calib_dir
        calib_dir.mkdir(parents=True, exist_ok=True)

        # ZED intrinsics YAML
        with open(calib_dir / "zed_intrinsics.yaml", "w", encoding="utf-8") as f:
            f.write("%YAML:1.0\n")
            f.write(f"camera_model: {calib['model']}\n")
            f.write(f"image_width: {calib['width']}\n")
            f.write(f"image_height: {calib['height']}\n")
            f.write(f"fx: {calib['fx']}\n")
            f.write(f"fy: {calib['fy']}\n")
            f.write(f"cx: {calib['cx']}\n")
            f.write(f"cy: {calib['cy']}\n")
            f.write(f"k1: {calib['k1']}\n")
            f.write(f"k2: {calib['k2']}\n")
            f.write(f"p1: {calib['p1']}\n")
            f.write(f"p2: {calib['p2']}\n")
            f.write(f"baseline: {calib['baseline']}\n")

        with open(calib_dir / "camera_info.json", "w", encoding="utf-8") as f:
            json.dump(calib, f, indent=2)

        # Camera-to-IMU placeholder
        with open(calib_dir / "camera_imu.yaml", "w", encoding="utf-8") as f:
            f.write(
                "rotation: !!python/tuple "
                "[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n"
            )
            f.write(
                "translation: !!python/tuple [0.0, 0.0, 0.0]\n"
            )
            f.write("status: not_calibrated\n")

        # Extrinsics placeholders
        ext_dir = self.extrinsics_dir
        ext_dir.mkdir(parents=True, exist_ok=True)
        for name in [
            "camera_to_mti.yaml", "camera_to_vehicle.yaml",
            "gnss_to_vehicle.yaml",
        ]:
            path = ext_dir / name
            if not path.exists():
                with open(path, "w", encoding="utf-8") as f:
                    f.write(
                        "# Extrinsics — fill after calibration\n"
                        "rotation: !!python/tuple "
                        "[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n"
                        "translation: !!python/tuple [0.0, 0.0, 0.0]\n"
                    )

        # Radar->camera extrinsic — unlike the placeholders above, this one
        # is actually measured (see config.settings.RADAR_TO_CAMERA_OFFSET_M
        # and docs/RADAR_CAMERA_OVERLAY.md). Always (re)written, not just
        # when missing, so a dataset always reflects the offset in effect
        # when it was recorded, in case the rig gets remeasured later.
        from config.settings import RADAR_TO_CAMERA_OFFSET_M
        dx, dy, dz = RADAR_TO_CAMERA_OFFSET_M
        with open(ext_dir / "radar_to_camera.yaml", "w", encoding="utf-8") as f:
            f.write(
                "# Radar->camera lever arm, in the radar's own frame\n"
                "# (x=right, y=forward, z=up), meters — see\n"
                "# radar/projection.py and docs/RADAR_CAMERA_OVERLAY.md.\n"
                "rotation: !!python/tuple "
                "[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n"
                f"translation: !!python/tuple [{dx}, {dy}, {dz}]\n"
            )

    def _enqueue(self, tag: str, payload) -> bool:
        """Enqueue a tagged item for async writing. Non-blocking."""
        self._ensure_backends()
        if self._queue.full():
            with self._lock:
                self._dropped += 1
            return False
        self._queue.put_nowait((tag, payload))
        return True

    def write_camera_frame(self, frame, synced_preview=None):
        """Enqueue one camera frame. Always recorded when self._recording,
        regardless of whether synced_preview (a live Synchronizer.sync()
        result, or None) matched any other sensor. synced_preview is used
        only to fill informational nearest_mti_ts/sync_delta_ns columns in
        the frame index — its absence never blocks this write.
        """
        if self._enqueue("camera", (frame, synced_preview)):
            with self._lock:
                self._frame_count += 1

    def write_mti_sample(self, mti_frame, host_rx_timestamp_ns: int):
        """Enqueue one native MTi packet, independent of camera timing.

        host_rx_timestamp_ns MUST be a UTC-epoch nanosecond timestamp (e.g.
        mti_frame.timestamp_ns + a captured mono->wall offset), not the raw
        host CLOCK_MONOTONIC value — it is used for HDF5 file-rotation dates
        and, unconverted, produces 1970-01-01 filenames. See
        dataset/mti_structured_recorder.py for the conversion.
        """
        self._enqueue("mti", (mti_frame, host_rx_timestamp_ns))

    def write_radar_frame(self, radar_frame, wall_timestamp_ns: int | None = None):
        """Enqueue one native radar frame, independent of camera timing.

        wall_timestamp_ns should be the UTC-epoch equivalent of
        radar_frame.timestamp_ns (which is host CLOCK_MONOTONIC) — callers
        (RadarStructuredPoller) compute this via a captured mono->wall
        offset. Filenames/Parquet partitioning need real dates; passing the
        raw monotonic value here previously produced 1970-01-01 filenames.
        """
        if not self._enable_radar:
            return
        self._enqueue("radar", (radar_frame, wall_timestamp_ns))

    def write_gnss_fix(self, fix: dict, wall_timestamp_ns: int | None = None):
        """Enqueue one native GNSS fix, independent of camera timing.

        wall_timestamp_ns should be the UTC-epoch equivalent of
        fix["timestamp_ns"] (host CLOCK_MONOTONIC) — see write_radar_frame.
        """
        self._enqueue("gnss", (fix, wall_timestamp_ns))

    def add_event(self, timestamp_ns: int, event_type: str,
                  description: str = ""):
        """Buffer an event marker."""
        self._ensure_backends()
        self._event_writer.add_event(
            timestamp_ns, event_type, description,
            session_id=self._session_id,
        )

    def _make_session_id(self, timestamp_ns: int) -> str:
        """A fresh, unique session_id for this REC press.

        Base format is unchanged (YYYYMMDD_HHMMSS, matching
        config_snapshot_*.json / session_manifest_*.json / HDF5 file
        naming) — a numeric suffix is only appended on an actual collision
        (two sessions starting within the same second in one app run).
        """
        dt = datetime.fromtimestamp(timestamp_ns / 1e9, tz=timezone.utc)
        base = dt.strftime("%Y%m%d_%H%M%S")
        sid = base
        n = 1
        while sid in self._used_session_ids:
            n += 1
            sid = f"{base}_{n}"
        self._used_session_ids.add(sid)
        return sid

    def begin_session(self, timestamp_ns: int):
        """Begin a new logical recording session in its own subfolder.

        Every session gets a fresh session_id (see _make_session_id) and a
        dedicated, self-contained directory tree at
        <dataset_root>/<session_id>/ — depth/, sensors/, radar/, metadata/,
        calibration/, manifests/, raw/, all freshly created there. Nothing
        from a previous or future session ever shares a file with this one.
        Calibration is re-saved into the new folder from the cached dict
        (save_calibration() is normally only called once, at app startup,
        before the first session exists).
        """
        # Finalize whatever the previous session (or the no-session startup
        # state) had open before switching directories out from under it.
        self._close_backends()

        self._session_id = self._make_session_id(timestamp_ns)
        session_root = self._dataset_root / self._session_id
        self._configure_dirs(session_root)
        self._open_backends()

        if self._calib is not None:
            self.save_calibration(self._calib)

        with self._lock:
            self._frame_count = 0
            self._dropped = 0
            self._bytes_written = 0
        self._mti_index = 0

        self._session_writer.begin_session(
            session_id=self._session_id,
            timestamp_ns=timestamp_ns,
            calibration_id="",
        )
        self.add_event(timestamp_ns, "RECORDING_START",
                       f"session={self._session_id}")

    def end_session(self, timestamp_ns: int):
        """Close the current logical recording session.

        Drains the write queue first: _write_camera/_write_mti/_write_radar/
        _write_gnss read self._session_id at dequeue time, not enqueue time,
        so any item still sitting in the queue when a *new* session's
        begin_session() next runs would otherwise get stamped with the
        wrong (new) session_id in its Parquet row. Also fully closes this
        session's HDF5 file (not just flushes it) and writes its manifest —
        once this returns, the session's directory is complete and
        immutable; nothing more will be written to it.
        """
        self._queue.join()
        self.add_event(timestamp_ns, "RECORDING_STOP",
                       f"session={self._session_id}")
        self._session_writer.end_session(
            session_id=self._session_id,
            end_timestamp_ns=timestamp_ns,
            status="COMPLETED",
            frames_end=self._frame_count,
            mti_end_index=self._mti_index,
        )
        self._flush_all()
        if self._hdf5_writer is not None:
            self._hdf5_writer.stop()
            self._hdf5_writer = None
        self.save_manifest()

    def record_sync_failure(self):
        with self._lock:
            self._sync_failures += 1

    def record_imu_drop(self):
        with self._lock:
            self._imu_drops += 1

    # ── Writer thread ───────────────────────────────────────────────

    def _write_loop(self):
        while True:
            item = self._queue.get()
            if item is None:
                self._queue.task_done()
                break
            tag, payload = item
            try:
                if tag == "camera":
                    self._write_camera(*payload)
                elif tag == "mti":
                    self._write_mti(*payload)
                elif tag == "radar":
                    self._write_radar(*payload)
                elif tag == "gnss":
                    self._write_gnss(*payload)
            except Exception as e:
                # A single bad sample (corrupt data, disk hiccup, a timestamp
                # that fails to convert, etc.) must never take down the
                # writer thread — that would silently stop ALL recording for
                # every sensor with no crash and no error visible to the
                # user. Log it and keep draining the queue.
                with self._lock:
                    self._write_errors += 1
                print(f"[DatasetWriter] write error (tag={tag}): {e}")
            finally:
                self._queue.task_done()

    def _write_camera(self, cam, synced_preview=None):
        """Persist one camera frame's depth/disparity/confidence and its
        Parquet frame-index row. Always runs for every recorded camera frame
        — never skipped for lack of an IMU/radar/lidar match. synced_preview
        (optional, from the live Synchronizer) only fills informational
        nearest_mti_ts/sync_delta_ns columns.
        """
        capture_timestamp_ns = cam.wall_timestamp_ns or cam.timestamp_ns
        prefix = _frame_prefix(capture_timestamp_ns, cam.frame_id)
        frame_bytes = 0

        # ── Images ────────────────────────────────────────────────
        # All ZED2 video and IMU is recorded inside the SVO2 file (started and
        # stopped by the camera around the session).  No per-frame colour
        # images are written; only the dense depth maps below are saved.
        left_path = ""
        right_path = ""

        # ── Depth / disparity / confidence ──────────────────────────
        depth_path = ""
        disp_path = ""
        if self._save_depth:
            depth_path = str(self.depth_dir / f"{prefix}.npy")
            np.save(depth_path, cam.depth.astype(np.float32))
            frame_bytes += Path(depth_path).stat().st_size

            if self._save_disparity and cam.disparity is not None:
                disp_path = str(self.disparity_dir / f"{prefix}.npy")
                np.save(disp_path, cam.disparity.astype(np.float32))
                frame_bytes += Path(disp_path).stat().st_size

            if self._save_confidence and cam.confidence is not None:
                conf_path = str(self.conf_dir / f"{prefix}.npy")
                np.save(conf_path, cam.confidence.astype(np.float32))
                frame_bytes += Path(conf_path).stat().st_size

        # ── Optional live-sync preview columns ───────────────────────
        # These are informational only (dashboard-preview provenance); their
        # absence never prevents this row from being written.
        #
        # synced_preview.imu_timestamp_ns comes straight from the live
        # Synchronizer (host CLOCK_MONOTONIC), while this row's own
        # timestamp_ns is wall-clock (capture_timestamp_ns, above) — stored
        # side by side unconverted, they look related but aren't: an
        # external dataset audit found "subtracting the two stored
        # timestamps does not produce the reported delta". Convert the IMU
        # timestamp to the same wall-clock basis using this camera frame's
        # own simultaneous mono/wall pair, so nearest_mti_timestamp_ns and
        # timestamp_ns are on one consistent clock and their difference
        # matches sync_delta_ns.
        nearest_mti_ts = 0
        sync_delta_ns = -1
        if synced_preview is not None and getattr(synced_preview, "imu", None):
            mono_to_wall_offset = capture_timestamp_ns - cam.timestamp_ns
            nearest_mti_ts = synced_preview.imu_timestamp_ns + mono_to_wall_offset
            sync_delta_ns = synced_preview.delta_ns

        # ── Frame index → Parquet ───────────────────────────────────
        if self._enable_parquet:
            self._frame_index.add_frame(
                frame_id=cam.frame_id,
                timestamp_ns=capture_timestamp_ns,
                session_id=self._session_id,
                left_path=left_path,
                right_path=right_path,
                depth_path=f"depth/{prefix}.npy" if depth_path else "",
                disparity_path=f"disparity/{prefix}.npy" if disp_path else "",
                nearest_mti_ts=nearest_mti_ts,
                sync_delta_ns=sync_delta_ns,
                sensor_hdf5_file="",
                nearest_mti_index=-1,
                exposure=cam.exposure,
                gain=cam.gain,
                temperature=cam.temperature,
            )

        with self._lock:
            self._bytes_written += frame_bytes

    def _write_mti(self, mti_frame, host_rx_timestamp_ns: int):
        """Persist one native MTi packet to HDF5 — independent of camera.

        Not every decoded Xbus packet carries inertial data — some Xsens
        output configurations emit GNSS-PVT-only messages with no
        acceleration/gyroscope fields at all. mti_frame_to_sample() falls
        back to [0, 0, 0] for an absent field (needed for the dense
        fixed-width HDF5 schema), which is indistinguishable from a real
        zero reading. An external dataset audit found exactly this:
        GNSS-only packets recorded as physically-zero IMU samples. Skip
        writing an HDF5 row for a packet with no inertial content at all —
        the raw Xbus stream still preserves it losslessly regardless.
        """
        if not self._enable_hdf5:
            return
        if mti_frame.acceleration is None and mti_frame.gyroscope is None:
            return
        sample = mti_frame_to_sample(
            mti_frame, host_timestamp_ns=host_rx_timestamp_ns
        )
        sample.timestamp_ns = host_rx_timestamp_ns
        self._hdf5_writer.put_mti_sample(sample)
        self._mti_index += 1

    def _write_radar(self, radar, wall_timestamp_ns: int | None = None):
        """Persist one native radar frame — independent of camera.

        radar.timestamp_ns is host CLOCK_MONOTONIC (arbitrary epoch — e.g.
        seconds since boot). Everything persisted to disk (filenames, Parquet
        partition dates) must use a real UTC timestamp instead, or every
        recording lands under a 1970-01-01 date. wall_timestamp_ns is that
        UTC-epoch equivalent; fall back to the raw value only if a caller
        didn't supply one (better a wrong-but-plausible date than a crash).
        """
        if not (
            radar.points or radar.range_doppler is not None
            or radar.range_profile is not None
        ):
            return
        ts_ns = wall_timestamp_ns if wall_timestamp_ns is not None else radar.timestamp_ns
        prefix = _frame_prefix(ts_ns, radar.frame_id)
        frame_bytes = 0
        radar_points_path = ""
        radar_heatmap_path = ""
        radar_range_profile_path = ""

        if radar.points:
            pts_data = [
                {"x": p.x, "y": p.y, "z": p.z, "velocity_mps": p.velocity_mps}
                for p in radar.points
            ]
            pts_path = self.radar_points_dir / f"{prefix}.json"
            with open(pts_path, "w", encoding="utf-8") as f:
                json.dump({
                    "timestamp_ns": ts_ns,
                    "frame_id": radar.frame_id,
                    "num_objects": radar.num_objects,
                    "points": pts_data,
                }, f)
            radar_points_path = f"radar/points/{prefix}.json"
            frame_bytes += pts_path.stat().st_size

        if radar.range_doppler is not None and radar.range_doppler.size > 0:
            rd_path = self.radar_heatmap_dir / f"{prefix}.npy"
            np.save(str(rd_path), radar.range_doppler.astype(np.uint16))
            radar_heatmap_path = f"radar/heatmap/{prefix}.npy"
            frame_bytes += rd_path.stat().st_size

        if radar.range_profile is not None and radar.range_profile.size > 0:
            rp_path = self.radar_range_profile_dir / f"{prefix}.npy"
            np.save(str(rp_path), radar.range_profile.astype(np.uint16))
            radar_range_profile_path = f"radar/range_profile/{prefix}.npy"
            frame_bytes += rp_path.stat().st_size

        if self._radar_index is not None:
            self._radar_index.add_radar_frame(
                frame_id=radar.frame_id,
                timestamp_ns=ts_ns,
                session_id=self._session_id,
                points_path=radar_points_path,
                heatmap_path=radar_heatmap_path,
                range_profile_path=radar_range_profile_path,
                num_objects=radar.num_objects,
                num_range_bins=radar.num_range_bins,
                num_doppler_bins=radar.num_doppler_bins,
            )

        with self._lock:
            self._bytes_written += frame_bytes

    def _write_gnss(self, fix: dict, wall_timestamp_ns: int | None = None):
        """Persist one native GNSS fix — independent of camera.

        See _write_radar's docstring: fix["timestamp_ns"] is host
        CLOCK_MONOTONIC, not a UTC epoch — wall_timestamp_ns is required to
        get a real Parquet partition date.
        """
        if self._gnss_index is None:
            return
        ts = wall_timestamp_ns if wall_timestamp_ns is not None else fix.get("timestamp_ns", 0)
        ts = int(ts)
        if ts <= 0:
            return
        self._gnss_index.add_fix(
            timestamp_ns=ts, session_id=self._session_id, fix=fix
        )

    # ── Manifest ────────────────────────────────────────────────────

    def save_manifest(self):
        calib_sha = ""
        calib_path = self.calib_dir / "zed_intrinsics.yaml"
        if calib_path.exists():
            calib_sha = _sha256_file(calib_path)

        with self._lock:
            cam_drops = self._dropped
            imu_drops = self._imu_drops
            sync_fails = self._sync_failures

        manifest = {
            "schema_version": "2.0",
            "dataset_version": "2.0",
            "vehicle": "",
            "operator": "",
            "route": "",
            "weather": "",
            "road_type": "",
            "camera": (
                self._calib.get("model", "unknown")
                if self._calib else "unknown"
            ),
            "imu": "MTi-8",
            "gnss": "u-blox F9P",
            "camera_fps": (
                self._calib.get("fps", 0) if self._calib else 0
            ),
            "imu_rate": 200,
            "resolution": (
                [self._calib["width"], self._calib["height"]]
                if self._calib else []
            ),
            "depth_mode": str(self._calib.get("depth_mode", "NEURAL_LIGHT"))
            if self._calib else "NEURAL_LIGHT",
            "coordinate_system": "RIGHT_HANDED_Z_UP",
            "image_format": "svo2",
            "record_date": datetime.now(tz=timezone.utc).isoformat(),
            "calibration_version": "v1.0",
            "camera_calibration_sha": calib_sha,
            "zed_sdk": self._zed_sdk,
            "mti_firmware": self._mti_firmware,
            "radar": "TI AWR1843BOOST",
            "lidar": "Ouster OS-0-64",
            "frames": self._frame_count,
            "dropped": cam_drops,
            "mti_samples": (
                self._hdf5_writer.mti_samples_written
                if self._hdf5_writer else 0
            ),
            "mti_drops": (
                self._hdf5_writer.mti_drops
                if self._hdf5_writer else 0
            ),
            "hdf5_files": (
                self._hdf5_writer.manifest_records
                if self._hdf5_writer else []
            ),
            "session_id": self._session_id,
            "directory_layout": {
                "left": "",
                "right": "",
                "depth": "depth/",
                "disparity": "disparity/" if self._save_disparity else "",
                "confidence": "confidence/" if self._save_confidence else "",
                "sensors": "sensors/",
                "metadata": "metadata/",
                "calibration": "calibration/",
                "radar": "radar/",
                "radar_points": "radar/points/",
                "radar_heatmap": "radar/heatmap/",
                "radar_range_profile": "radar/range_profile/",
                "raw": "raw/",
                "lidar_pcap": "raw/lidar.pcap",
                "lidar_config_json": "raw/lidar_0.json",
            },
        }
        self.manifests_dir.mkdir(parents=True, exist_ok=True)
        with open(self.manifests_dir / "dataset_manifest.json", "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)

    # ── Helpers ─────────────────────────────────────────────────────

    def _flush_all(self):
        """Flush all writers."""
        if self._frame_index:
            self._frame_index.flush()
        if self._radar_index:
            self._radar_index.flush()
        if self._gnss_index:
            self._gnss_index.flush()
        if self._event_writer:
            self._event_writer.flush()
        if self._hdf5_writer:
            self._hdf5_writer.flush()

    def _periodic_flush_loop(self):
        while not self._flush_stop.wait(self._flush_interval_s):
            try:
                self._flush_all()
            except Exception as e:
                # Never let a flush failure kill this background thread —
                # it would silently widen the crash data-loss window back
                # to "the whole session" with no visible symptom.
                print(f"[DatasetWriter] periodic flush failed: {e}")

    def save_config_snapshot(self, snapshot: dict):
        """Write an immutable per-session configuration snapshot.

        Never call this with a dict containing NTRIP credentials or other
        secrets — build_config_snapshot() (dataset/config_snapshot.py)
        already excludes them.
        """
        self.meta_dir.mkdir(parents=True, exist_ok=True)
        path = self.meta_dir / f"config_snapshot_{self._session_id}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(snapshot, f, indent=2, default=str)

    def save_session_manifest(self, manifest: dict):
        """Write the per-session file manifest (see dataset/session_manifest.py
        for what it contains and why it's distinct from the config snapshot
        and the cumulative dataset manifest)."""
        self.meta_dir.mkdir(parents=True, exist_ok=True)
        path = self.meta_dir / f"session_manifest_{self._session_id}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2, default=str)

    @property
    def frame_count(self):
        return self._frame_count

    @property
    def bytes_written(self):
        return self._bytes_written

    @property
    def session_id(self):
        return self._session_id

    @property
    def write_queue_size(self):
        return self._queue.qsize()

    @property
    def image_writer_queue_peak(self):
        return self._queue.maxsize

    @property
    def sensor_writer_queue_peak(self):
        if self._hdf5_writer:
            return self._hdf5_writer._mti_queue.maxsize
        return 0

    @property
    def mti_samples_written(self):
        if self._hdf5_writer:
            return self._hdf5_writer.mti_samples_written
        return 0

    @property
    def mti_drops(self):
        if self._hdf5_writer:
            return self._hdf5_writer.mti_drops
        return 0

    @property
    def dropped(self):
        return self._dropped

    @property
    def write_errors(self):
        """Count of per-item write exceptions the writer thread absorbed
        (and logged) instead of dying. Non-zero means something is silently
        failing to persist — check the process log."""
        return self._write_errors

    def close(self):
        """Signal writer thread to stop and flush all pending data."""
        self._flush_stop.set()
        self._flush_thread.join(timeout=self._flush_interval_s + 2)

        self._queue.put(None)
        self._queue.join()
        self._thread.join(timeout=10)

        # Flush Parquet and events
        self._flush_all()

        # Close HDF5
        if self._hdf5_writer:
            self._hdf5_writer.stop()

        # Write final manifest
        self.save_manifest()

        print(
            f"Saved {self._frame_count} frames, "
            f"dropped {self._dropped} to {self.root}"
        )
