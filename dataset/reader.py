"""Dataset reader — load recorded multi-sensor datasets.

Reads and provides access to:
  * Camera frame index + depth (Parquet + NPY)
  * IMU data (HDF5)
  * Radar points + heatmap (Parquet index + JSON/NPY, or ROS bag)
  * LiDAR point clouds — derived on demand from one session's pcap + its
    single companion config JSON (``<session_id>/raw/lidar.pcap`` +
    ``<session_id>/raw/lidar_0.json``). There is no per-frame lidar JSON to
    read; scans are decoded from the pcap using the ouster SDK, keyed by
    the packet timestamps recorded in the pcap itself.
  * Frame synchronization metadata (metadata/synchronization/, derived —
    see fusion/offline_sync.py)

Every recording session lives in its own subfolder —
``<dataset_root>/<session_id>/`` — a complete, self-contained,
immutable directory tree (depth/, sensors/, radar/, metadata/,
calibration/, manifests/, raw/), created by ``DatasetWriter.begin_session()``.
Nothing from one session shares a file, or a directory, with another.
``resolve_lidar_paths()`` finds the right pcap/config-JSON pair for a given
(or the most recently modified) session's subfolder.

Two APIs are provided over the same underlying logic:
  * Free functions (detect_format, load_v2_frames, load_v2_mti) — used by
    playback.py / validate_dataset.py.
  * DatasetReader class — used by playback_dataset.py for its lidar
    inspection, and extended here with frames()/mti_samples() for callers
    that prefer an object with cached state.
"""

from __future__ import annotations

import glob
from pathlib import Path
from typing import Iterator, NamedTuple, Optional

import h5py
import numpy as np
import pandas as pd


def list_sessions(root: str | Path) -> list[str]:
    """List session_ids under a dataset root, most recently modified first.

    A directory counts as a session if it has its own raw/ subfolder —
    that's the one thing every session, recording any combination of
    sensors, is guaranteed to have (DatasetWriter creates it unconditionally
    in _configure_dirs()).
    """
    root = Path(root)
    if not root.exists():
        return []
    candidates = [
        d for d in root.iterdir()
        if d.is_dir() and (d / "raw").is_dir()
    ]
    candidates.sort(key=lambda d: d.stat().st_mtime, reverse=True)
    return [d.name for d in candidates]


def resolve_lidar_paths(root: str | Path, session_id: str | None = None
                        ) -> tuple[Optional[Path], Optional[Path]]:
    """Find the (pcap, config_json) pair for one session's lidar recording.

    With session_id given, looks in that session's own
    <session_id>/raw/lidar.pcap (+ _0.json). Without one, tries every
    session under root, most recently modified first — a reasonable
    default for ad-hoc inspection tools, but callers that need a specific
    session (e.g. offline sync) should always pass session_id explicitly.

    Also accepts two earlier, now-superseded layouts so older datasets
    remain readable: a flat raw/lidar_<session_id>.pcap (the brief interim
    design before folder-per-session), and the original flat raw/lidar.pcap
    (before any session isolation existed).
    """
    root = Path(root)

    def _exact(pcap_path: Path, meta_path: Path) -> tuple[Optional[Path], Optional[Path]]:
        if pcap_path.exists() and meta_path.exists():
            return pcap_path, meta_path
        return None, None

    if session_id:
        # 1. Current layout: this session's own subfolder.
        pcap_path, meta_path = _exact(
            root / session_id / "raw" / "lidar.pcap",
            root / session_id / "raw" / "lidar_0.json",
        )
        if pcap_path is not None:
            return pcap_path, meta_path
        # 2. Brief interim layout: flat raw/, filename suffixed by session_id.
        return _exact(
            root / "raw" / f"lidar_{session_id}.pcap",
            root / "raw" / f"lidar_{session_id}_0.json",
        )

    for sid in list_sessions(root):
        pcap_path, meta_path = _exact(
            root / sid / "raw" / "lidar.pcap",
            root / sid / "raw" / "lidar_0.json",
        )
        if pcap_path is not None:
            return pcap_path, meta_path

    # 3. Original flat, pre-session-isolation layout (unsuffixed).
    return _exact(root / "raw" / "lidar.pcap", root / "raw" / "lidar_0.json")


class LiDARFrame(NamedTuple):
    """One LiDAR scan, decoded on demand from the session pcap."""
    frame_id: int
    timestamp_ns: int  # from the pcap's own packet capture timestamps
    points: np.ndarray  # (N, 3) float32 XYZ in meters
    reflectivity: Optional[np.ndarray] = None  # (N,) float32


def iter_lidar_frames(pcap_path: str | Path,
                       meta_path: str | Path | None = None
                       ) -> Iterator[LiDARFrame]:
    """Lazily decode scans from a single Ouster pcap + its config JSON.

    Never reads or requires per-frame JSON files — this is the only way
    lidar point clouds are derived for a recorded session. ``meta_path``
    defaults to the ``<stem>_0.json`` file ``LidarReceiver.start_pcap()``
    writes alongside the pcap; ``open_source`` auto-discovers it when not
    given explicitly.

    Each frame's timestamp is taken directly from the pcap's own recorded
    packet-capture timestamps (``get_first_valid_packet_timestamp()``), not
    fabricated or read from any side-channel file.
    """
    pcap_path = Path(pcap_path)
    if not pcap_path.exists():
        raise FileNotFoundError(f"Lidar pcap not found: {pcap_path}")
    if meta_path is None:
        meta_path = pcap_path.with_name(f"{pcap_path.stem}_0.json")
    meta_path = Path(meta_path)
    if not meta_path.exists():
        raise FileNotFoundError(
            f"Lidar pcap {pcap_path} has no companion config JSON at "
            f"{meta_path} — a pcap alone is not replayable."
        )

    from ouster.sdk import open_source
    from ouster.sdk.core import ChanField, XYZLut

    source = open_source(str(pcap_path), collate=True)
    info = source.sensor_info[0]
    xyz_lut = XYZLut(info, use_extrinsics=False)

    for frameset in source:
        for frame in frameset.valid_frames():
            if frame is None:
                continue
            rng = (
                frame.field(ChanField.RANGE)
                if frame.has_field(ChanField.RANGE) else None
            )
            if rng is None:
                continue
            xyz = xyz_lut(rng).astype(np.float32)
            valid = (
                np.isfinite(xyz).all(axis=-1) & (np.abs(xyz).sum(axis=-1) > 0)
            )
            points = xyz[valid].reshape(-1, 3)
            reflectivity = None
            if frame.has_field(ChanField.REFLECTIVITY):
                reflectivity = (
                    frame.field(ChanField.REFLECTIVITY)
                    .astype(np.float32)[valid]
                )
            yield LiDARFrame(
                frame_id=frame.frame_id,
                timestamp_ns=int(frame.get_first_valid_packet_timestamp()),
                points=points,
                reflectivity=reflectivity,
            )


class IMUSample(NamedTuple):
    """One IMU sample from HDF5 recording."""
    timestamp_ns: int
    accel: tuple[float, float, float]
    gyro: tuple[float, float, float]
    euler: tuple[float, float, float]


def detect_format(root: Path) -> str:
    """Return 'v1', 'v2', or 'unknown' based on directory structure."""
    root = Path(root)
    if (root / "timestamps.csv").exists() or (root / "images").exists():
        return "v1"
    if (root / "metadata" / "frames").exists() or (root / "sensors").exists():
        return "v2"
    return "unknown"


def load_v2_frames(root: Path) -> list[dict]:
    """Load the camera frame index from Parquet, return list of dicts
    ordered by timestamp_ns."""
    root = Path(root)
    files = sorted(
        glob.glob(str(root / "metadata" / "frames" / "**" / "*.parquet"),
                   recursive=True)
    )
    if not files:
        return []
    df = pd.concat((pd.read_parquet(f) for f in files), ignore_index=True)
    df = df.sort_values("timestamp_ns")
    return df.to_dict("records")


def load_v2_mti(root: Path) -> dict:
    """Load all MTi data from HDF5, return dict mapping
    timestamp_ns -> {"ax","ay","az","gx","gy","gz"}."""
    root = Path(root)
    sensors_dir = root / "sensors"
    out: dict[int, dict] = {}
    if not sensors_dir.exists():
        return out
    for h5_path in sorted(sensors_dir.glob("*.h5")):
        try:
            with h5py.File(str(h5_path), "r") as f:
                if "mti" not in f:
                    continue
                g = f["mti"]
                ts = np.asarray(g["timestamp_ns"][:])
                accel = np.asarray(g["acceleration"][:])
                gyro = np.asarray(g["angular_velocity"][:])
                for i in range(len(ts)):
                    out[int(ts[i])] = {
                        "ax": float(accel[i, 0]), "ay": float(accel[i, 1]),
                        "az": float(accel[i, 2]),
                        "gx": float(gyro[i, 0]), "gy": float(gyro[i, 1]),
                        "gz": float(gyro[i, 2]),
                    }
        except Exception as e:
            print(f"Warning: Failed to load {h5_path}: {e}")
    return out


class DatasetReader:
    """Load recorded multi-sensor dataset.

    LiDAR scans are decoded lazily from one session's pcap + its config
    JSON (see resolve_lidar_paths()) — there is no per-frame JSON to load.
    Decoding the whole pcap can be slow for a long session, so it only
    happens on first access to a lidar accessor (or an explicit call to
    ``_load_lidar_frames()``), not in the constructor.
    """

    def __init__(self, root: str | Path, session_id: str | None = None):
        self.root = Path(root)
        self.session_id = session_id
        self._lidar_frames: dict[int, LiDARFrame] = {}
        self._lidar_loaded = False

    def _lidar_paths(self) -> tuple[Optional[Path], Optional[Path]]:
        return resolve_lidar_paths(self.root, self.session_id)

    def _load_lidar_frames(self):
        """Decode every scan from the session pcap into memory.

        Safe to call more than once — a no-op after the first successful
        load. Does nothing (leaves zero frames) if no pcap was recorded for
        this session.
        """
        if self._lidar_loaded:
            return
        self._lidar_loaded = True
        pcap_path, meta_path = self._lidar_paths()
        if pcap_path is None:
            return
        try:
            for frame in iter_lidar_frames(pcap_path, meta_path):
                self._lidar_frames[frame.frame_id] = frame
        except Exception as e:
            print(f"Warning: Failed to decode {pcap_path}: {e}")

    def get_lidar_frame(self, frame_id: int) -> Optional[LiDARFrame]:
        """Get one LiDAR frame by ID (decoded from the pcap on first use)."""
        self._load_lidar_frames()
        return self._lidar_frames.get(frame_id)

    def lidar_frames(self) -> list[LiDARFrame]:
        """Get all LiDAR frames, decoded from the pcap on first use."""
        self._load_lidar_frames()
        return sorted(self._lidar_frames.values(), key=lambda f: f.frame_id)

    @property
    def num_lidar_frames(self) -> int:
        """Number of LiDAR frames decoded (triggers decoding on first use)."""
        self._load_lidar_frames()
        return len(self._lidar_frames)

    def detect_format(self) -> str:
        return detect_format(self.root)

    def frames(self) -> pd.DataFrame:
        """Camera frame index as a DataFrame, sorted by timestamp_ns."""
        rows = load_v2_frames(self.root)
        return pd.DataFrame(rows)

    def mti_samples(self) -> dict:
        """MTi samples keyed by timestamp_ns (see load_v2_mti)."""
        return load_v2_mti(self.root)
