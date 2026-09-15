"""Logical session metadata — Parquet-based, one row per session.

A session is a REC→STOP cycle. It's metadata only — no separate directory.

Schema:
    session_id: string (YYYYMMDD_HHMMSS)
    start_timestamp_ns: int64
    end_timestamp_ns: int64
    start_datetime_utc: string
    end_datetime_utc: string
    status: string (OPEN | COMPLETED | INTERRUPTED)
    vehicle: string
    route: string
    operator: string
    weather: string
    notes: string
    calibration_id: string
    frames_start: int64
    frames_end: int64
    mti_start_index: int64
    mti_end_index: int64
"""

from __future__ import annotations
import threading
from datetime import datetime, timezone
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

SESSION_SCHEMA = pa.schema([
    ("session_id", pa.string()),
    ("start_timestamp_ns", pa.int64()),
    ("end_timestamp_ns", pa.int64()),
    ("start_datetime_utc", pa.string()),
    ("end_datetime_utc", pa.string()),
    ("status", pa.string()),
    ("vehicle", pa.string()),
    ("route", pa.string()),
    ("operator", pa.string()),
    ("weather", pa.string()),
    ("notes", pa.string()),
    ("calibration_id", pa.string()),
    ("frames_start", pa.int64()),
    ("frames_end", pa.int64()),
    ("mti_start_index", pa.int64()),
    ("mti_end_index", pa.int64()),
])


class SessionWriter:
    """Writes logical session records to partitioned Parquet."""

    def __init__(self, metadata_dir: Path, compression: str = "snappy"):
        self._sessions_dir = metadata_dir / "sessions"
        self._sessions_dir.mkdir(parents=True, exist_ok=True)
        self._compression = compression
        self._lock = threading.Lock()
        self._active_session = None

    def begin_session(self, session_id: str, timestamp_ns: int,
                      calibration_id: str = "",
                      vehicle: str = "", route: str = "",
                      operator: str = "", weather: str = "",
                      notes: str = "") -> dict:
        """Start a new logical session."""
        dt = datetime.fromtimestamp(timestamp_ns / 1e9, tz=timezone.utc)
        session = {
            "session_id": session_id,
            "start_timestamp_ns": timestamp_ns,
            "end_timestamp_ns": 0,
            "start_datetime_utc": dt.isoformat(),
            "end_datetime_utc": "",
            "status": "OPEN",
            "vehicle": vehicle,
            "route": route,
            "operator": operator,
            "weather": weather,
            "notes": notes,
            "calibration_id": calibration_id,
            "frames_start": 0,
            "frames_end": 0,
            "mti_start_index": 0,
            "mti_end_index": 0,
        }
        with self._lock:
            self._active_session = session
        # Persist the OPEN row immediately — if the process crashes before
        # end_session() ever runs, this is the only record that the session
        # happened at all. Consumers reading metadata/sessions/ should treat
        # the row with the latest status per session_id as authoritative
        # (COMPLETED written by end_session() supersedes this OPEN row).
        self._write_session(dict(session))
        return session

    def end_session(self, session_id: str, end_timestamp_ns: int,
                    status: str = "COMPLETED",
                    frames_end: int = 0, mti_end_index: int = 0):
        """Close the active session and write to Parquet."""
        with self._lock:
            session = self._active_session
            self._active_session = None

        if session is None:
            return

        dt = datetime.fromtimestamp(end_timestamp_ns / 1e9, tz=timezone.utc)
        session["end_timestamp_ns"] = end_timestamp_ns
        session["end_datetime_utc"] = dt.isoformat()
        session["status"] = status
        session["frames_end"] = frames_end
        session["mti_end_index"] = mti_end_index

        self._write_session(session)

    def _write_session(self, session: dict):
        date_str = datetime.fromtimestamp(
            session["start_timestamp_ns"] / 1e9, tz=timezone.utc
        ).strftime("%Y%m%d")

        part_dir = self._sessions_dir / f"date={date_str}"
        part_dir.mkdir(parents=True, exist_ok=True)

        # Find next part number
        existing = sorted(part_dir.glob("part-*.parquet"))
        next_num = len(existing) + 1
        part_path = part_dir / f"part-{next_num:06d}.parquet"

        table = pa.Table.from_pylist([session], schema=SESSION_SCHEMA)
        pq.write_table(table, str(part_path), compression=self._compression)
        print(f"[SESSION] Wrote {session['session_id']} → {part_path.name}")
