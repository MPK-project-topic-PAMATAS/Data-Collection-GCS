"""Serial reader for TI AWR1843BOOST data port (TLV stream).

Reads raw bytes from /dev/ttyACM1 and yields complete mmWave frames
(magic + header + TLVs) as soon as a full frame (per header totalPacketLen)
has arrived.

Includes reconnection logic: if the serial port disconnects (common with
TI AWR1843 USB CDC-ACM), the reader will pause and retry until the port
is available again.
"""

from __future__ import annotations

import struct
import threading
import time
import queue
import logging

from config.settings import RADAR_DATA_PORT, RADAR_DATA_BAUD

_MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"
_MAX_FRAME = 0x20000  # 128 KiB safety cap

# totalPacketLen lives at offset 12 (after 8B magic + 4B version)
_TOTAL_LEN_OFFSET = 12

_RECONNECT_DELAY_S = 0.5
_RECONNECT_MAX_S = 5.0
_MAX_CONSECUTIVE_ERRORS = 5

log = logging.getLogger(__name__)


class RadarSerialReader:
    """Background thread that buffers serial bytes and yields full frames."""

    def __init__(self, port: str = RADAR_DATA_PORT, baud: int = RADAR_DATA_BAUD):
        self._port = port
        self._baud = baud
        self._q: queue.Queue[bytes] = queue.Queue(maxsize=60)
        self._serial = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._buf = bytearray()

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _open_port(self) -> bool:
        """Open (or reopen) the DATA serial port.

        DTR/RTS are asserted (high) so the EVM runs rather than sitting in
        reset, which would look like "not connected".
        """
        import serial as _serial
        try:
            self._serial = _serial.Serial(
                self._port, self._baud, timeout=0.1,
                bytesize=_serial.EIGHTBITS,
                parity=_serial.PARITY_NONE,
                stopbits=_serial.STOPBITS_ONE,
            )
            self._serial.dtr = True
            self._serial.rts = True
            self._serial.reset_input_buffer()
            self._buf.clear()
            log.info("Radar data port opened: %s @ %d", self._port, self._baud)
            return True
        except Exception as e:
            log.warning("Radar data port open failed: %s", e)
            return False

    def _read_loop(self):
        consecutive_errors = 0
        reconnect_delay = _RECONNECT_DELAY_S

        while self._running:
            # Open port if not connected
            if self._serial is None or not self._serial.is_open:
                if not self._open_port():
                    time.sleep(reconnect_delay)
                    reconnect_delay = min(reconnect_delay * 1.5, _RECONNECT_MAX_S)
                    continue
                reconnect_delay = _RECONNECT_DELAY_S

            try:
                n = self._serial.in_waiting
                if n > 0:
                    self._buf.extend(self._serial.read(n))
                    self._extract_frames()
                    consecutive_errors = 0
                else:
                    time.sleep(0.0001)
            except (OSError, ValueError) as e:
                consecutive_errors += 1
                log.warning("Radar serial read error (%d/%d): %s",
                            consecutive_errors, _MAX_CONSECUTIVE_ERRORS, e)
                # Close stale port — will reconnect on next iteration
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
                time.sleep(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 1.5, _RECONNECT_MAX_S)
                if consecutive_errors >= _MAX_CONSECUTIVE_ERRORS:
                    log.error("Radar serial port persistently failing, "
                              "will keep retrying")
                    consecutive_errors = 0  # reset counter, keep trying
            except Exception as e:
                log.debug("Radar read loop unexpected error: %s", e)
                time.sleep(0.01)

    def _extract_frames(self):
        while True:
            # Align to the 8-byte magic
            idx = self._buf.find(_MAGIC)
            if idx < 0:
                if len(self._buf) >= 8:
                    del self._buf[:-7]
                return
            if idx > 0:
                del self._buf[:idx]

            if len(self._buf) < _TOTAL_LEN_OFFSET + 4:
                return
            total_len = struct.unpack_from("<I", self._buf, _TOTAL_LEN_OFFSET)[0]
            if total_len < 36 or total_len > _MAX_FRAME:
                # Corrupt: drop one byte of magic and resync
                del self._buf[:1]
                continue

            if len(self._buf) < total_len:
                return

            frame = bytes(self._buf[:total_len])
            del self._buf[:total_len]
            try:
                self._q.put_nowait(frame)
            except queue.Full:
                pass  # ponytail: drop stale frames, add warning log if throughput matters

    def read(self, timeout: float = 0.005) -> bytes | None:
        try:
            return self._q.get(timeout=timeout)
        except queue.Empty:
            return None

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._serial and self._serial.is_open:
            self._serial.close()
