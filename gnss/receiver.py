"""Combined GNSS receiver: serial GPS + NTRIP RTK corrections.

Connects to a serial GPS receiver, sends NTRIP corrections,
and parses NMEA output for position/fix data.

Usage:
    gnss = GNSSReceiver(
        serial_port="/dev/ttyUSB1",
        baudrate=115200,
        ntrip_host="195.182.72.152",
        ntrip_port=2111,
        ntrip_user="2976shns",
        ntrip_pass="stu9ofro",
        mountpoint="VRS_CMRx",
    )
    gnss.start()
    fix = gnss.latest()
    gnss.stop()
"""

from __future__ import annotations
import threading
import time
from collections import deque
from datetime import datetime, timezone

import serial

from gnss.ntrip_client import NTRIPClient
from gnss.nmea_parser import parse_gga, parse_rmc

# GPS epoch: January 6, 1980 00:00:00 UTC
_GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
_GPS_WEEK_SECS = 7 * 24 * 3600


def _utc_to_gps_week_tow(utc_str: str):
    """Convert UTC time string 'HHMMSS.SS' to (gps_week, tow)."""
    try:
        h = int(utc_str[:2])
        m = int(utc_str[2:4])
        s = float(utc_str[4:])
        now = datetime.now(timezone.utc).replace(
            hour=h, minute=m, second=int(s),
            microsecond=int((s % 1) * 1e6),
        )
        delta = now - _GPS_EPOCH
        total_secs = int(delta.total_seconds())
        gps_week = total_secs // _GPS_WEEK_SECS
        tow = total_secs % _GPS_WEEK_SECS + (s % 1)
        return gps_week, tow
    except Exception:
        return 0, 0.0


class GNSSReceiver:
    """Serial GPS + NTRIP corrections, runs in background thread."""

    def __init__(self, serial_port: str = "/dev/ttyUSB1",
                 baudrate: int = 115200,
                 ntrip_host: str = "", ntrip_port: int = 2111,
                 ntrip_user: str = "", ntrip_pass: str = "",
                 mountpoint: str = "VRS_CMRx",
                 ntrip_enabled: bool = True,
                 serial_timeout: float = 1.0,
                 buffer_size: int = 300):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.mountpoint = mountpoint
        self.ntrip_enabled = ntrip_enabled
        self.serial_timeout = serial_timeout

        self._ser = None
        self._ntrip = None
        self._running = False
        self._thread = None

        self._lock = threading.Lock()
        self._latest_gga = None
        self._latest_rmc = None
        self._fix_count = 0
        self._rtk_fixed = False
        self._latest_timestamp_ns = 0
        self._ntrip_connected = False
        self._bytes_rx = 0
        # Buffered fix history — independent of any camera/IMU match, used by
        # GNSSStructuredPoller to persist every fix at native GNSS rate.
        self._buffer: deque[dict] = deque(maxlen=buffer_size)
        # Raw NMEA byte log — independent of decoding, mirrors
        # mti8.serial_reader's raw Xbus tee.
        self._raw_log_file = None
        # RLock, not Lock: start_raw_log() calls stop_raw_log() while
        # already holding this lock (to safely rotate an existing log
        # file) — a plain Lock would deadlock the caller (this previously
        # froze the Qt main thread solid on every REC press).
        self._raw_lock = threading.RLock()

        if ntrip_enabled and ntrip_host:
            self._ntrip = NTRIPClient(
                ntrip_host, ntrip_port, ntrip_user, ntrip_pass
            )

    def start(self) -> bool:
        try:
            self._ser = serial.Serial(
                self.serial_port, self.baudrate,
                timeout=self.serial_timeout
            )
            self._ser.reset_input_buffer()
        except Exception:
            return False

        if self._ntrip and self.ntrip_enabled:
            if not self._ntrip.connect(self.mountpoint):
                pass
            else:
                self._ntrip_connected = True

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def _loop(self):
        while self._running:
            try:
                raw_line = self._ser.readline()
                with self._raw_lock:
                    if self._raw_log_file is not None:
                        self._raw_log_file.write(raw_line)
                        self._raw_log_file.flush()
                line = raw_line.decode("ascii", errors="replace").strip()
                if not line.startswith("$"):
                    continue

                self._bytes_rx += len(line)

                if line.endswith("GGA"):
                    gga = parse_gga(line)
                    if gga and gga.valid:
                        with self._lock:
                            self._latest_gga = gga
                            self._rtk_fixed = gga.is_rtk_fixed
                            self._fix_count += 1
                            self._latest_timestamp_ns = time.monotonic_ns()
                            fix = self._build_fix_locked()
                            if fix is not None:
                                self._buffer.append(fix)
                        if self._ntrip and self._ntrip_connected:
                            self._ntrip.send_gga(line)

                elif line.endswith("RMC"):
                    rmc = parse_rmc(line)
                    if rmc and rmc.valid:
                        with self._lock:
                            self._latest_rmc = rmc
                            self._latest_timestamp_ns = time.monotonic_ns()

            except serial.SerialException:
                break
            except Exception:
                continue

    def start_raw_log(self, path: str):
        """Tee every raw NMEA line (pre-parse) to a file — independent of
        decoding, so a parser bug or unsupported sentence never loses data."""
        with self._raw_lock:
            self.stop_raw_log()
            self._raw_log_file = open(path, "wb")

    def stop_raw_log(self):
        with self._raw_lock:
            if self._raw_log_file is not None:
                try:
                    self._raw_log_file.close()
                except Exception:
                    pass
                self._raw_log_file = None

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._ntrip:
            self._ntrip.disconnect()
        if self._ser and self._ser.is_open:
            self._ser.close()
        self.stop_raw_log()

    def _build_fix_locked(self) -> dict | None:
        """Build the current fix dict. Caller must hold self._lock."""
        if not self._latest_gga:
            return None

        utc_time = ""
        gps_week = 0
        gps_tow = 0.0
        if self._latest_rmc and hasattr(self._latest_rmc, "utc_time"):
            utc_time = self._latest_rmc.utc_time
            gps_week, gps_tow = _utc_to_gps_week_tow(utc_time)

        return {
            "lat": self._latest_gga.lat_decimal,
            "lon": self._latest_gga.lon_decimal,
            "alt": self._latest_gga.altitude,
            "fix": self._latest_gga.fix_type,
            "fix_quality": self._latest_gga.fix_quality,
            "satellites": self._latest_gga.satellites,
            "hdop": self._latest_gga.hdop,
            "speed": (
                self._latest_rmc.speed_mps
                if self._latest_rmc else 0.0
            ),
            "course": (
                self._latest_rmc.course_deg
                if self._latest_rmc else 0.0
            ),
            "rtk_fixed": self._rtk_fixed,
            "utc_time": utc_time,
            "gps_week": gps_week,
            "gps_tow": gps_tow,
            "timestamp_ns": self._latest_timestamp_ns,
        }

    def latest(self) -> dict | None:
        with self._lock:
            return self._build_fix_locked()

    def get_new_since(self, after_ts_ns: int) -> list[dict]:
        """Return every buffered fix with timestamp_ns > after_ts_ns, oldest
        first — used by GNSSStructuredPoller to persist every fix at native
        rate, independent of any camera frame.
        """
        with self._lock:
            return [f for f in self._buffer if f["timestamp_ns"] > after_ts_ns]

    @property
    def running(self) -> bool:
        return self._running

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def ntrip_connected(self) -> bool:
        return self._ntrip_connected

    @property
    def fix_count(self) -> int:
        return self._fix_count

    @property
    def bytes_received(self) -> int:
        return self._bytes_rx + (
            self._ntrip.bytes_received if self._ntrip else 0
        )
