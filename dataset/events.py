"""Event index writer — partitioned Parquet, append-only.

Schema:
    event_id: int64 (auto-increment)
    timestamp_ns: int64
    session_id: string
    event_type: string
    description: string
"""

from __future__ import annotations
import threading
from datetime import datetime, timezone
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

EVENT_SCHEMA = pa.schema([
    ("event_id", pa.int64()),
    ("timestamp_ns", pa.int64()),
    ("session_id", pa.string()),
    ("event_type", pa.string()),
    ("description", pa.string()),
])


class EventWriter:
    """Writes events to partitioned Parquet."""

    def __init__(self, metadata_dir: Path, compression: str = "snappy"):
        self._events_dir = metadata_dir / "events"
        self._events_dir.mkdir(parents=True, exist_ok=True)
        self._compression = compression
        self._lock = threading.Lock()
        self._event_counter = 0
        self._buffer = []

    def add_event(self, timestamp_ns: int, event_type: str,
                  description: str = "", session_id: str = ""):
        """Buffer an event. Will be flushed with frame index or on demand."""
        with self._lock:
            self._event_counter += 1
            self._buffer.append({
                "event_id": self._event_counter,
                "timestamp_ns": timestamp_ns,
                "session_id": session_id,
                "event_type": event_type,
                "description": description,
            })

    def flush(self):
        """Write buffered events to Parquet."""
        with self._lock:
            if not self._buffer:
                return
            rows = list(self._buffer)
            self._buffer.clear()

        # Group by date
        by_date = {}
        for row in rows:
            dt = datetime.fromtimestamp(row["timestamp_ns"] / 1e9,
                                        tz=timezone.utc)
            date_str = dt.strftime("%Y%m%d")
            by_date.setdefault(date_str, []).append(row)

        for date_str, date_rows in by_date.items():
            part_dir = self._events_dir / f"date={date_str}"
            part_dir.mkdir(parents=True, exist_ok=True)
            existing = sorted(part_dir.glob("part-*.parquet"))
            next_num = len(existing) + 1
            part_path = part_dir / f"part-{next_num:06d}.parquet"

            table = pa.Table.from_pylist(date_rows, schema=EVENT_SCHEMA)
            pq.write_table(table, str(part_path), compression=self._compression)
