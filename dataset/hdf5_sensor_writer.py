"""HDF5 sensor writer — rotating, append-only, thread-safe via queue.

Architecture:
    MTi serial thread / ZED IMU grab
            |
    SensorWriterQueue (thread-safe put)
            |
    HDF5 writer thread (owns h5py file)
            |
    Rotating HDF5 files (daily or size-based)

Do NOT access the same h5py File from multiple threads.
This module owns the file exclusively in the writer thread.
"""

from __future__ import annotations
import json
import queue
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import h5py
import numpy as np

from config.settings import HDF5_WRITE_BATCH_SIZE, HDF5_BATCH_MAX_LATENCY_S


class SensorSample:
    """Single MTi or ZED IMU sample — flat dataclass for queue transport."""

    __slots__ = (
        "timestamp_ns", "host_timestamp_ns", "device_timestamp",
        "packet_counter",
        "acceleration", "angular_velocity", "magnetometer",
        "quaternion_raw",
        "latitude", "longitude", "altitude_msl", "altitude_ellipsoid",
        "velocity",
        "temperature", "baro_pressure",
        "status_word", "status_byte",
        "delta_velocity", "delta_quaternion",
        "sensor_type",  # "mti" or "zed_imu"
        # ZED IMU fields
        "zed_orientation",
    )

    def __init__(self, **kwargs):
        for slot in self.__slots__:
            setattr(self, slot, kwargs.get(slot, None))
        if self.sensor_type is None:
            self.sensor_type = "mti"


class SensorWriterQueue:
    """Thread-safe queue for sensor samples between capture and HDF5 writer."""

    def __init__(self, maxsize=4096):
        self._queue = queue.Queue(maxsize=maxsize)
        self._overflow_count = 0
        self._lock = threading.Lock()

    def put(self, sample: SensorSample) -> bool:
        try:
            self._queue.put_nowait(sample)
            return True
        except queue.Full:
            with self._lock:
                self._overflow_count += 1
            return False

    def get(self, timeout=0.1):
        try:
            item = self._queue.get(timeout=timeout)
            self._queue.task_done()
            return item
        except queue.Empty:
            return None

    def qsize(self):
        return self._queue.qsize()

    @property
    def maxsize(self):
        return self._queue.maxsize

    @property
    def overflow_count(self):
        return self._overflow_count


# ── HDF5 dataset layout specs ─────────────────────────────────────────

MTI_DATASETS = {
    "timestamp_ns": {"dtype": np.int64, "shape": (None,), "chunks": True},
    "host_timestamp_ns": {"dtype": np.int64, "shape": (None,), "chunks": True},
    "device_timestamp": {"dtype": np.int64, "shape": (None,), "chunks": True},
    "packet_counter": {"dtype": np.int64, "shape": (None,), "chunks": True},
    "acceleration": {"dtype": np.float32, "shape": (None, 3), "chunks": (1024, 3)},
    "angular_velocity": {"dtype": np.float32, "shape": (None, 3), "chunks": (1024, 3)},
    "magnetometer": {"dtype": np.float32, "shape": (None, 3), "chunks": (1024, 3)},
    "quaternion_raw": {"dtype": np.float32, "shape": (None, 4), "chunks": (1024, 4)},
    "latitude": {"dtype": np.float64, "shape": (None,), "chunks": True},
    "longitude": {"dtype": np.float64, "shape": (None,), "chunks": True},
    "altitude_msl": {"dtype": np.float64, "shape": (None,), "chunks": True},
    "altitude_ellipsoid": {"dtype": np.float64, "shape": (None,), "chunks": True},
    "velocity": {"dtype": np.float32, "shape": (None, 3), "chunks": (1024, 3)},
    "temperature": {"dtype": np.float32, "shape": (None,), "chunks": True},
    "baro_pressure": {"dtype": np.float32, "shape": (None,), "chunks": True},
    "status_word": {"dtype": np.uint32, "shape": (None,), "chunks": True},
    "status_byte": {"dtype": np.uint8, "shape": (None,), "chunks": True},
    "delta_velocity": {"dtype": np.float32, "shape": (None, 3), "chunks": (1024, 3)},
    "delta_quaternion": {"dtype": np.float32, "shape": (None, 4), "chunks": (1024, 4)},
}

ZED_IMU_DATASETS = {
    "timestamp_ns": {"dtype": np.int64, "shape": (None,), "chunks": True},
    "acceleration": {"dtype": np.float32, "shape": (None, 3), "chunks": (1024, 3)},
    "angular_velocity": {"dtype": np.float32, "shape": (None, 3), "chunks": (1024, 3)},
    "quaternion": {"dtype": np.float32, "shape": (None, 4), "chunks": (1024, 4)},
    "temperature": {"dtype": np.float32, "shape": (None,), "chunks": True},
}


def _make_empty(spec):
    """Create empty numpy array matching HDF5 dataset spec."""
    dtype = spec["dtype"]
    shape = spec["shape"]
    if len(shape) == 1:
        return np.empty(0, dtype=dtype)
    return np.empty((0,) + shape[1:], dtype=dtype)


def _to_vec3(arr, fallback=(0, 0, 0)):
    """Convert tuple/list to float32 [N,3] array."""
    if arr is None:
        return np.array([fallback], dtype=np.float32)
    if isinstance(arr, (tuple, list)):
        return np.array([arr], dtype=np.float32)
    return np.array(arr, dtype=np.float32).reshape(-1, 3)


def _to_vec4(arr, fallback=(0, 0, 0, 1)):
    """Convert tuple/list to float32 [N,4] array."""
    if arr is None:
        return np.array([fallback], dtype=np.float32)
    if isinstance(arr, (tuple, list)):
        return np.array([arr], dtype=np.float32)
    return np.array(arr, dtype=np.float32).reshape(-1, 4)


def _to_scalar(arr, dtype=np.float64, fallback=0.0):
    """Convert scalar or array to numpy scalar/array."""
    if arr is None:
        return np.array([fallback], dtype=dtype)
    if isinstance(arr, (int, float)):
        return np.array([arr], dtype=dtype)
    return np.array([arr], dtype=dtype).flatten()


class HDF5SensorWriter:
    """Writes MTi and ZED IMU samples to rotating HDF5 files.

    Thread model:
        - put_mti_sample() / put_zed_imu_sample() called from ANY thread
        - Internal writer thread drains queue and writes to h5py

    File rotation:
        - Default: rotate on UTC date change
        - Files named: YYYYMMDD_000000_mti.h5 (daily) or YYYYMMDD_HHMMSS_mti.h5
    """

    def __init__(self, sensors_dir: Path, enable_zed_imu: bool = True,
                 rotation_mode: str = "daily", max_size_gb: float = 4.0,
                 compression: str = "lzf", max_queue: int = 8192,
                 write_batch_size: int = HDF5_WRITE_BATCH_SIZE,
                 batch_max_latency_s: float = HDF5_BATCH_MAX_LATENCY_S):
        self._sensors_dir = Path(sensors_dir)
        self._sensors_dir.mkdir(parents=True, exist_ok=True)
        self._enable_zed_imu = enable_zed_imu
        self._rotation_mode = rotation_mode
        self._max_size_gb = max_size_gb
        self._compression = compression
        self._write_batch_size = write_batch_size
        self._batch_max_latency_s = batch_max_latency_s

        self._mti_queue = SensorWriterQueue(maxsize=max_queue)
        self._zed_imu_queue = SensorWriterQueue(maxsize=max_queue) if enable_zed_imu else None

        self._running = False
        self._thread = None
        self._lock = threading.Lock()

        # Current file state
        self._current_file = None
        self._current_filename = None
        self._current_date = None
        self._current_start_ns = None
        self._sample_count = 0
        self._total_bytes = 0

        # Manifest records for completed files
        self._manifest_records = []

        # Statistics
        self._mti_samples_written = 0
        self._zed_imu_samples_written = 0
        self._mti_drops = 0
        self._zed_imu_drops = 0

    def start(self):
        self._running = True
        self._thread = threading.Thread(
            target=self._write_loop, daemon=True, name="hdf5-writer"
        )
        self._thread.start()

    def stop(self):
        """Signal writer to stop and drain remaining samples."""
        # Wait for all queued samples to be processed
        self._mti_queue._queue.join()
        if self._zed_imu_queue:
            self._zed_imu_queue._queue.join()
        self._running = False
        if self._thread:
            self._thread.join(timeout=10)
        # Drain any remaining samples after thread exits
        self._drain_queue()
        self._close_current_file()

    def _drain_queue(self):
        """Write all remaining samples in the queue."""
        while True:
            sample = self._mti_queue.get(timeout=0.0)
            if sample is None:
                break
            self._write_sample(sample)
        if self._zed_imu_queue:
            while True:
                sample = self._zed_imu_queue.get(timeout=0.0)
                if sample is None:
                    break
                self._write_sample(sample)

    def put_mti_sample(self, sample: SensorSample) -> bool:
        sample.sensor_type = "mti"
        ok = self._mti_queue.put(sample)
        if not ok:
            with self._lock:
                self._mti_drops += 1
        return ok

    def put_zed_imu_sample(self, sample: SensorSample) -> bool:
        if not self._zed_imu_queue:
            return False
        sample.sensor_type = "zed_imu"
        ok = self._zed_imu_queue.put(sample)
        if not ok:
            with self._lock:
                self._zed_imu_drops += 1
        return ok

    def _write_loop(self):
        while self._running:
            # Batch writes to match the HDF5 chunk layout.  Resizing every
            # dataset for every 200 Hz sample is unnecessarily expensive.
            mti_sample = self._mti_queue.get(timeout=0.1)
            if mti_sample is not None:
                samples = self._drain_batch(self._mti_queue, mti_sample)
                self._write_mti_batch(samples)

            # Drain ZED IMU queue (non-blocking after first get)
            if self._zed_imu_queue:
                zed_sample = self._zed_imu_queue.get(timeout=0.001)
                if zed_sample is not None:
                    samples = self._drain_batch(self._zed_imu_queue, zed_sample)
                    self._write_zed_imu_batch(samples)

    def _drain_batch(self, source: SensorWriterQueue, first: SensorSample):
        samples = [first]
        deadline = time.monotonic() + self._batch_max_latency_s
        while len(samples) < self._write_batch_size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            sample = source.get(timeout=remaining)
            if sample is None:
                break
            samples.append(sample)
        return samples

    def _write_sample(self, sample: SensorSample):
        ts_ns = sample.timestamp_ns
        if ts_ns <= 0:
            return

        # Check if we need to rotate files
        self._maybe_rotate(ts_ns)

        # Open file if needed
        if self._current_file is None:
            self._open_file(ts_ns)

        # Write to appropriate group
        if sample.sensor_type == "mti":
            self._write_mti_batch([sample])
        elif sample.sensor_type == "zed_imu":
            self._write_zed_imu_batch([sample])

    def _prepare_batch_file(self, samples: list[SensorSample]):
        """Open/rotate before a same-sensor batch is appended."""
        valid = [sample for sample in samples if sample.timestamp_ns > 0]
        if not valid:
            return []
        first = valid[0]
        self._maybe_rotate(first.timestamp_ns)
        if self._current_file is None:
            self._open_file(first.timestamp_ns)
        return valid

    def _get_date(self, ts_ns: int):
        return datetime.fromtimestamp(ts_ns / 1e9, tz=timezone.utc).date()

    def _get_filename(self, ts_ns: int) -> str:
        dt = datetime.fromtimestamp(ts_ns / 1e9, tz=timezone.utc)
        if self._rotation_mode == "daily":
            return dt.strftime("%Y%m%d_000000_mti.h5")
        return dt.strftime("%Y%m%d_%H%M%S_mti.h5")

    def _maybe_rotate(self, ts_ns: int):
        if self._current_file is None:
            return
        current_date = self._get_date(ts_ns)
        if self._rotation_mode == "daily" and current_date != self._current_date:
            self._close_current_file()
            return
        # Size-based rotation
        if self._rotation_mode == "size":
            try:
                size_bytes = self._current_file.id.get_file_size()
                if size_bytes > self._max_size_gb * 1024**3:
                    self._close_current_file()
            except Exception:
                pass

    def _open_file(self, ts_ns: int):
        filename = self._get_filename(ts_ns)
        filepath = self._sensors_dir / filename

        # Safety: don't overwrite existing files — append suffix
        counter = 1
        while filepath.exists():
            stem = filepath.stem
            filepath = self._sensors_dir / f"{stem}_{counter:03d}.h5"
            counter += 1

        self._current_file = h5py.File(str(filepath), "w",
                                       libver="latest")
        self._current_date = self._get_date(ts_ns)
        self._current_start_ns = ts_ns
        self._sample_count = 0
        self._current_filename = filepath.name

        # Write file-level attributes
        self._current_file.attrs["schema_version"] = "2.0"
        self._current_file.attrs["sensor_model"] = "Xsens MTi-8"
        self._current_file.attrs["serial_port"] = "/dev/ttyUSB0"
        self._current_file.attrs["baud_rate"] = 2000000
        self._current_file.attrs["coordinate_system"] = "NED"
        self._current_file.attrs["quaternion_raw_order"] = "wxyz"
        self._current_file.attrs["orientation_convention"] = "quaternion_qLS"
        self._current_file.attrs["creation_time_utc"] = \
            datetime.now(tz=timezone.utc).isoformat()
        self._current_file.attrs["compression"] = self._compression

        # Create MTi group with empty datasets
        mti = self._current_file.create_group("mti")
        for name, spec in MTI_DATASETS.items():
            maxshape = tuple(None if s is None else s for s in spec["shape"])
            if len(spec["shape"]) == 1:
                maxshape = (None,)
            mti.create_dataset(
                name,
                shape=(0,) if len(spec["shape"]) == 1 else (0,) + spec["shape"][1:],
                maxshape=maxshape,
                dtype=spec["dtype"],
                chunks=spec.get("chunks"),
                compression=self._compression if self._compression else None,
                compression_opts=None,
            )

        # Create ZED IMU group if enabled
        if self._enable_zed_imu:
            zed = self._current_file.create_group("zed_imu")
            for name, spec in ZED_IMU_DATASETS.items():
                maxshape = tuple(None if s is None else s for s in spec["shape"])
                if len(spec["shape"]) == 1:
                    maxshape = (None,)
                zed.create_dataset(
                    name,
                    shape=(0,) if len(spec["shape"]) == 1 else (0,) + spec["shape"][1:],
                    maxshape=maxshape,
                    dtype=spec["dtype"],
                    chunks=spec.get("chunks"),
                    compression=self._compression if self._compression else None,
                    compression_opts=None,
                )

        # Single-Writer/Multiple-Reader mode: once enabled, any reader can
        # open this file concurrently (h5py.File(path, "r", swmr=True))
        # while this writer keeps appending, instead of getting "file is
        # already open for write" until the whole application exits. Must
        # be enabled only after every dataset that will ever exist in this
        # file has already been created above — SWMR forbids further
        # structural changes (new datasets/attrs), only appends to existing
        # ones. An external dataset audit found a "COMPLETED" session whose
        # HDF5 file could not be opened read-only and failed SWMR read too;
        # this is what makes concurrent/early reads actually work.
        try:
            self._current_file.swmr_mode = True
        except Exception as e:
            print(f"[HDF5] SWMR mode unavailable ({e}); "
                  f"file will only be readable after close().")

        print(f"[HDF5] Opened {filepath.name}")

    def _close_current_file(self):
        if self._current_file is None:
            return
        try:
            # Record end timestamp
            self._current_file.attrs["end_timestamp_ns"] = \
                self._sample_count  # placeholder
            self._current_file.flush()
            self._current_file.close()
        except Exception as e:
            print(f"[HDF5] Error closing: {e}")
        finally:
            # Save manifest record
            self._manifest_records.append({
                "filename": self._current_filename,
                "start_timestamp_ns": self._current_start_ns,
                "sample_count": self._sample_count,
            })
            print(f"[HDF5] Closed {self._current_filename} "
                  f"({self._sample_count} samples)")
            self._current_file = None
            self._current_filename = None
            self._sample_count = 0

    def _append_mti(self, name: str, data: np.ndarray):
        """Append data to an MTi dataset using resizable arrays."""
        ds = self._current_file[f"mti/{name}"]
        old_len = ds.shape[0]
        new_len = old_len + data.shape[0]
        ds.resize(new_len, axis=0)
        ds[old_len:new_len] = data

    def _append_zed_imu(self, name: str, data: np.ndarray):
        """Append data to a ZED IMU dataset using resizable arrays."""
        ds = self._current_file[f"zed_imu/{name}"]
        old_len = ds.shape[0]
        new_len = old_len + data.shape[0]
        ds.resize(new_len, axis=0)
        ds[old_len:new_len] = data

    def _write_mti_sample(self, sample: SensorSample):
        g = self._current_file["mti"]
        n = self._sample_count + 1

        self._append_mti("timestamp_ns",
                         np.array([sample.timestamp_ns], dtype=np.int64))
        self._append_mti("host_timestamp_ns",
                         np.array([sample.host_timestamp_ns or 0], dtype=np.int64))
        self._append_mti("device_timestamp",
                         np.array([sample.device_timestamp or 0], dtype=np.int64))
        self._append_mti("packet_counter",
                         np.array([sample.packet_counter or 0], dtype=np.int64))

        self._append_mti("acceleration", _to_vec3(sample.acceleration))
        self._append_mti("angular_velocity", _to_vec3(sample.angular_velocity))
        self._append_mti("magnetometer", _to_vec3(sample.magnetometer))
        self._append_mti("quaternion_raw", _to_vec4(sample.quaternion_raw))

        self._append_mti("latitude", _to_scalar(sample.latitude, np.float64))
        self._append_mti("longitude", _to_scalar(sample.longitude, np.float64))
        self._append_mti("altitude_msl", _to_scalar(sample.altitude_msl, np.float64))
        self._append_mti("altitude_ellipsoid",
                         _to_scalar(sample.altitude_ellipsoid, np.float64))

        self._append_mti("velocity", _to_vec3(sample.velocity))
        self._append_mti("temperature",
                         _to_scalar(sample.temperature, np.float32))
        self._append_mti("baro_pressure",
                         _to_scalar(sample.baro_pressure, np.float32))
        self._append_mti("status_word",
                         np.array([sample.status_word or 0], dtype=np.uint32))
        self._append_mti("status_byte",
                         np.array([sample.status_byte or 0], dtype=np.uint8))
        self._append_mti("delta_velocity", _to_vec3(sample.delta_velocity))
        self._append_mti("delta_quaternion", _to_vec4(sample.delta_quaternion))

        self._sample_count = n
        with self._lock:
            self._mti_samples_written += 1

    def _write_mti_batch(self, samples: list[SensorSample]):
        samples = self._prepare_batch_file(samples)
        if not samples:
            return
        # Queue batches cannot normally cross a UTC day.  Preserve rotation
        # correctness if a long scheduling pause makes one do so.
        first_date = self._get_date(samples[0].timestamp_ns)
        split = next((i for i, s in enumerate(samples)
                      if self._get_date(s.timestamp_ns) != first_date), None)
        if split is not None:
            self._write_mti_batch(samples[:split])
            self._write_mti_batch(samples[split:])
            return

        self._append_mti("timestamp_ns", np.asarray(
            [s.timestamp_ns for s in samples], dtype=np.int64))
        self._append_mti("host_timestamp_ns", np.asarray(
            [s.host_timestamp_ns or 0 for s in samples], dtype=np.int64))
        self._append_mti("device_timestamp", np.asarray(
            [s.device_timestamp or 0 for s in samples], dtype=np.int64))
        self._append_mti("packet_counter", np.asarray(
            [s.packet_counter or 0 for s in samples], dtype=np.int64))
        for name, attr, width in (
            ("acceleration", "acceleration", 3),
            ("angular_velocity", "angular_velocity", 3),
            ("magnetometer", "magnetometer", 3),
            ("quaternion_raw", "quaternion_raw", 4),
            ("velocity", "velocity", 3),
            ("delta_velocity", "delta_velocity", 3),
            ("delta_quaternion", "delta_quaternion", 4),
        ):
            default = (0, 0, 0, 1) if width == 4 else (0, 0, 0)
            self._append_mti(name, np.asarray(
                [getattr(s, attr) if getattr(s, attr) is not None else default
                 for s in samples], dtype=np.float32))
        for name, attr, dtype in (
            ("latitude", "latitude", np.float64),
            ("longitude", "longitude", np.float64),
            ("altitude_msl", "altitude_msl", np.float64),
            ("altitude_ellipsoid", "altitude_ellipsoid", np.float64),
            ("temperature", "temperature", np.float32),
            ("baro_pressure", "baro_pressure", np.float32),
            ("status_word", "status_word", np.uint32),
            ("status_byte", "status_byte", np.uint8),
        ):
            self._append_mti(name, np.asarray(
                [getattr(s, attr) or 0 for s in samples], dtype=dtype))
        self._sample_count += len(samples)
        with self._lock:
            self._mti_samples_written += len(samples)

    def _write_zed_imu_sample(self, sample: SensorSample):
        n = self._sample_count + 1
        self._append_zed_imu("timestamp_ns",
                             np.array([sample.timestamp_ns], dtype=np.int64))
        self._append_zed_imu("acceleration",
                             _to_vec3(sample.acceleration))
        self._append_zed_imu("angular_velocity",
                             _to_vec3(sample.angular_velocity))
        self._append_zed_imu("quaternion",
                             _to_vec4(sample.zed_orientation or sample.quaternion_raw))
        self._append_zed_imu("temperature",
                             _to_scalar(sample.temperature, np.float32))

        self._sample_count = n
        with self._lock:
            self._zed_imu_samples_written += 1

    def _write_zed_imu_batch(self, samples: list[SensorSample]):
        samples = self._prepare_batch_file(samples)
        if not samples:
            return
        self._append_zed_imu("timestamp_ns", np.asarray(
            [s.timestamp_ns for s in samples], dtype=np.int64))
        for name, attr, default in (
            ("acceleration", "acceleration", (0, 0, 0)),
            ("angular_velocity", "angular_velocity", (0, 0, 0)),
            ("quaternion", "zed_orientation", (0, 0, 0, 1)),
        ):
            self._append_zed_imu(name, np.asarray(
                [getattr(s, attr) if getattr(s, attr) is not None else default
                 for s in samples], dtype=np.float32))
        self._append_zed_imu("temperature", np.asarray(
            [s.temperature or 0 for s in samples], dtype=np.float32))
        self._sample_count += len(samples)
        with self._lock:
            self._zed_imu_samples_written += len(samples)

    # ── Public stats ──────────────────────────────────────────────────

    @property
    def mti_samples_written(self):
        return self._mti_samples_written

    @property
    def zed_imu_samples_written(self):
        return self._zed_imu_samples_written

    @property
    def mti_drops(self):
        return self._mti_drops

    @property
    def zed_imu_drops(self):
        return self._zed_imu_drops

    @property
    def queue_depth(self):
        return self._mti_queue.qsize() + (
            self._zed_imu_queue.qsize() if self._zed_imu_queue else 0
        )

    @property
    def current_filename(self):
        return self._current_filename

    @property
    def manifest_records(self):
        return list(self._manifest_records)

    def flush(self):
        """Force flush current HDF5 file."""
        if self._current_file:
            try:
                self._current_file.flush()
            except Exception:
                pass


def mti_frame_to_sample(mti_frame, host_timestamp_ns=None) -> SensorSample:
    """Convert MTiFrame dataclass to SensorSample for HDF5 writer."""
    return SensorSample(
        timestamp_ns=mti_frame.timestamp_ns,
        host_timestamp_ns=host_timestamp_ns or mti_frame.timestamp_ns,
        device_timestamp=mti_frame.sample_time_fine | (mti_frame.sample_time_coarse << 32) if hasattr(mti_frame, 'sample_time_coarse') else 0,
        packet_counter=mti_frame.packet_counter,
        acceleration=mti_frame.acceleration,
        angular_velocity=mti_frame.gyroscope,
        magnetometer=mti_frame.magnetometer,
        quaternion_raw=mti_frame.quaternion,
        latitude=mti_frame.latitude,
        longitude=mti_frame.longitude,
        altitude_msl=mti_frame.altitude_msl,
        altitude_ellipsoid=mti_frame.altitude_ellipsoid,
        velocity=mti_frame.velocity,
        temperature=mti_frame.temperature,
        baro_pressure=mti_frame.baro_pressure,
        status_word=mti_frame.status_word,
        status_byte=mti_frame.status_byte,
        delta_velocity=mti_frame.delta_velocity,
        delta_quaternion=mti_frame.delta_quaternion,
        sensor_type="mti",
    )


def zed_imu_to_sample(zed_imu_dict: dict, timestamp_ns: int) -> SensorSample:
    """Convert ZED IMU dict to SensorSample for HDF5 writer."""
    return SensorSample(
        timestamp_ns=timestamp_ns,
        acceleration=zed_imu_dict.get("linear_acceleration"),
        angular_velocity=zed_imu_dict.get("angular_velocity"),
        zed_orientation=zed_imu_dict.get("orientation"),
        temperature=zed_imu_dict.get("temperature"),
        sensor_type="zed_imu",
    )
