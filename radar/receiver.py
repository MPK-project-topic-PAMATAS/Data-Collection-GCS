"""TI AWR1843BOOST radar receiver.

Opens two serial ports:
  CFG (/dev/ttyACM0, 115200)  — CLI for sending the sensor config
  DATA (/dev/ttyACM1, 921600) — TLV detected-objects data

A background thread decodes incoming frames and exposes the latest RadarFrame
via `get_frame()` / `get_nearest(ts)`.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque

from radar.serial_reader import RadarSerialReader
from radar.parser import parse_frame_header, parse_tlvs, _HEADER_SIZE
from radar.frame import RadarFrame
from radar.ports import detect_ports
from config.settings import (
    RADAR_CFG_PORT, RADAR_CFG_BAUD,
    RADAR_DATA_PORT, RADAR_DATA_BAUD,
    RADAR_BUFFER_SIZE,
    RADAR_NUM_RANGE_BINS, RADAR_NUM_DOPPLER_BINS, RADAR_NUM_VIRT_ANT,
    RADAR_STREAM_TIMEOUT_NS,
    RADAR_RECONFIG_TIMEOUT_NS,
)

log = logging.getLogger(__name__)

_WATCHDOG_POLL_S = 1.0
_RECONFIG_BACKOFF_S = 2.0
_RECONFIG_BACKOFF_MAX_S = 30.0

# 3TX elevation-capable config (TX0 + TX1 + TX2), 256 ADC samples, heatmap
# output OFF (this firmware can't combine the 3TX chirp table with heatmap
# output; the dashboard shows the point cloud only).
#
# (3TX details) TX1 is the antenna
# offset vertically from TX0/TX2 (confirmed both live and in TI's EVM
# user's guide SPRUIM4B section 2.7 — see docs/RADAR_POINT_CLOUD.md);
# enabling it is what makes z (elevation) vary instead of being hard-
# zeroed. This firmware build cannot combine the 3TX chirp table with the
# range-Doppler heatmap output (guiMonitor's heatmap flag) — enabling both
# together makes sensorStart throw a firmware exception and wedges the
# CLI, needing a physical unplug/replug to recover (confirmed twice) — so
# guiMonitor here has heatmap OFF. dashboard/radar_panel.py's
# RadarCameraOverlay (which replaced the old range-Doppler panel) doesn't
# need it anyway.
#
# cfarCfg's thresholdScale is 20 (not the TI-default 15): validated live
# with a single fixed target at ~2m — threshold 20 gave the tightest,
# most reliable detections (100% frame coverage, elevation std ~5mm; see
# docs/RADAR_POINT_CLOUD.md for the full sweep, including why threshold
# 25 is WORSE, not better — it drops detection reliability). An earlier,
# less careful jitter test (closest-point-in-scene, confounded by which
# object was closest changing between frames) wrongly suggested 3TX was
# just noisy; it wasn't — that test's methodology was the problem, not
# the radar. Re-validate against a fixed target if this profile changes.
_SENSOR_CONFIG_CMDS = [
    "sensorStop",
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 15 7 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "profileCfg 0 77 267 7 57.14 0 0 70 1 256 5209 0 0 30",
    "chirpCfg 0 0 0 0 0 0 0 1",
    "chirpCfg 1 1 0 0 0 0 0 4",
    "chirpCfg 2 2 0 0 0 0 0 2",
    "frameCfg 0 2 16 0 100 1 0",
    "lowPower 0 0",
    "guiMonitor -1 1 1 0 0 0 1",
    "cfarCfg -1 0 2 8 4 3 0 20 1",
    "cfarCfg -1 1 0 4 2 3 1 20 1",
    "multiObjBeamForming -1 1 0.5",
    "clutterRemoval -1 0",
    "calibDcRangeSig -1 0 -5 8 256",
    "extendedMaxVelocity -1 0",
    "lvdsStreamCfg -1 0 0 0",
    "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0",
    "measureRangeBiasAndRxChanPhase 0 1.5 0.2",
    "CQRxSatMonitor 0 3 5 121 0",
    "CQSigImgMonitor 0 127 4",
    "analogMonitor 0 0",
    "aoaFovCfg -1 -90 90 -90 90",
    "cfarFovCfg -1 0 0 8.92",
    "cfarFovCfg -1 1 -1 1.00",
    "calibData 0 0 0",
]


class RadarReceiver:

    def __init__(self):
        self._cfg_port = RADAR_CFG_PORT
        self._data_port = RADAR_DATA_PORT
        self._cfg_serial = None
        self._latest: RadarFrame | None = None
        self._frame_count = 0
        self._rate = 0.0
        self._last_frame_ns = 0
        self._buffer: deque[RadarFrame] = deque(maxlen=RADAR_BUFFER_SIZE)
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False
        self._frame_id = 0
        self._prev_header_frame_number: int | None = None
        self._dropped_frames = 0
        self._watchdog_thread: threading.Thread | None = None

    def start(self):
        import serial as _serial
        data_port, cfg_port = detect_ports(RADAR_DATA_BAUD)
        if data_port and cfg_port:
            self._data_port, self._cfg_port = data_port, cfg_port
        # Assert DTR/RTS so the EVM leaves reset and the CLI boots. On this
        # carrier DTR/RTS double as boot/reset; leaving them LOW holds the
        # device in reset (appears "not connected"). Open the port ONCE and
        # do not pulse DTR during operation.
        self._cfg_serial = _serial.Serial(self._cfg_port, RADAR_CFG_BAUD, timeout=0.1)
        self._cfg_serial.dtr = True
        self._cfg_serial.rts = True
        self._cfg_serial.reset_input_buffer()
        self._apply_config()
        self._data_reader = RadarSerialReader(self._data_port, RADAR_DATA_BAUD)
        self._data_reader.start()
        self._running = True
        self._start_time = time.monotonic_ns()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True, name="radar-watchdog"
        )
        self._watchdog_thread.start()

    def _apply_config(self):
        # If the EVM is wedged (e.g. after a DTR-triggered reset) the CLI never
        # prints a prompt. Fail fast with a clear error instead of blocking
        # ~3s x len(cmds) with no recovery.
        for cmd in _SENSOR_CONFIG_CMDS:
            self._cfg_serial.write((cmd + "\n").encode())
            if not self._wait_prompt(3.0):
                raise RuntimeError(
                    f"Radar CFG did not respond to '{cmd}' — EVM unresponsive. "
                    "Re-plug the sensor and retry."
                )
        self._cfg_serial.write(b"sensorStart\n")
        self._cfg_serial.flush()
        time.sleep(0.1)

    def _wait_prompt(self, timeout_s: float) -> bool:
        """Block until the CLI prints its prompt, proving it consumed the last
        line. A fast block-write makes the mmWave CLI drop lines, which later
        surfaces as a 'partial configuration' error on sensorStart."""
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            self._cfg_serial.timeout = 0.05
            try:
                chunk = self._cfg_serial.read(self._cfg_serial.in_waiting or 1)
            except Exception:
                time.sleep(0.02)  # flaky ttyACM read; back off and continue
                continue
            if chunk:
                buf.extend(chunk)
                if b"mmwDemo:/>" in buf or b"mmwDemo: >" in buf:
                    return True
            else:
                time.sleep(0.02)
        return False

    def _loop(self):
        while self._running:
            data = self._data_reader.read()
            if data is None:
                continue

            hdr = parse_frame_header(data)
            if hdr is None:
                continue

            payload = data[_HEADER_SIZE:]
            parsed = parse_tlvs(
                payload, hdr["num_detected_obj"],
                num_range_bins=RADAR_NUM_RANGE_BINS,
                num_doppler_bins=RADAR_NUM_DOPPLER_BINS,
                num_virt_ant=RADAR_NUM_VIRT_ANT,
            )
            if not parsed:
                continue

            self._frame_id += 1
            now = time.monotonic_ns()

            self._track_frame_gap(hdr["frame_number"])

            frame = RadarFrame(
                points=parsed.get("points", []),
                range_doppler=parsed.get("range_doppler"),
                range_profile=parsed.get("range_profile"),
                range_azimuth=parsed.get("range_azimuth"),
                timestamp_ns=now,
                frame_id=hdr["frame_number"],
                num_objects=hdr["num_detected_obj"],
                num_range_bins=parsed.get("num_range_bins", 0),
                num_doppler_bins=parsed.get("num_doppler_bins", 0),
            )

            with self._lock:
                self._latest = frame
                self._buffer.append(frame)
                self._frame_count += 1
                self._last_frame_ns = now
                elapsed = now - self._start_time
                self._rate = self._frame_count / (elapsed / 1e9) if elapsed > 0 else 0.0

    def _watchdog_loop(self):
        """Recover from a chip reset that RadarSerialReader's own port
        reconnect can't see: the DATA port comes back open, but the AWR1843
        forgot its config and needs _SENSOR_CONFIG_CMDS + sensorStart resent
        before it streams again. Without this, a transient USB brownout on
        the shared CDC-ACM link silently ends radar recording for the rest
        of the session.
        """
        backoff = _RECONFIG_BACKOFF_S
        while self._running:
            time.sleep(_WATCHDOG_POLL_S)
            if not self._running:
                break
            with self._lock:
                last = self._last_frame_ns
            if last == 0:
                continue  # still waiting for the first frame since start()
            idle_ns = time.monotonic_ns() - last
            if idle_ns <= RADAR_RECONFIG_TIMEOUT_NS or not self.connected:
                continue
            log.warning(
                "Radar idle %.1fs with DATA port connected — reapplying "
                "sensor config", idle_ns / 1e9,
            )
            if self._reconfigure():
                backoff = _RECONFIG_BACKOFF_S
                # Give the chip a moment to start producing frames before
                # re-checking, so we don't immediately retrigger.
                time.sleep(_WATCHDOG_POLL_S)
            else:
                time.sleep(backoff)
                backoff = min(backoff * 1.5, _RECONFIG_BACKOFF_MAX_S)

    def _reconfigure(self) -> bool:
        """Re-detect ports, reopen the CFG port, and resend the sensor
        config + sensorStart. Safe to call repeatedly; failures are logged
        and swallowed so the watchdog keeps retrying rather than killing
        the receiver.

        Confirmed live: after a physical unplug/replug the EVM can
        re-enumerate under a *different* /dev/ttyACM* node (observed
        ttyACM0 -> ttyACM2 for CFG). The previous version trusted
        ``self._cfg_port``/``self._cfg_serial.is_open`` unconditionally,
        so it kept retrying a dead file descriptor at a now-nonexistent
        path forever ([Errno 5] Input/output error on every attempt,
        never recovering). Re-running detect_ports() here picks up the
        new node; the DATA port's own reader also gets its path updated
        in case that one renumbered too (RadarSerialReader's own
        reconnect loop only knows the path it was constructed with).
        """
        import serial as _serial

        data_port, cfg_port = detect_ports(RADAR_DATA_BAUD)
        if cfg_port and cfg_port != self._cfg_port:
            log.warning("Radar CFG port renumbered: %s -> %s",
                        self._cfg_port, cfg_port)
            self._cfg_port = cfg_port
        if data_port and data_port != self._data_port:
            log.warning("Radar DATA port renumbered: %s -> %s",
                        self._data_port, data_port)
            self._data_port = data_port
            data_reader = getattr(self, "_data_reader", None)
            if data_reader is not None:
                data_reader._port = data_port
                # The reader's currently-open handle (if any) points at the
                # stale path's node, which may now be a *different device*
                # (e.g. the old DATA node reassigned to the CFG interface)
                # rather than simply gone — it can sit there "open" and
                # silent, never producing a magic-word frame but never
                # raising a read error either, so the reader's own
                # reconnect logic would never trigger on its own. Force it
                # to reopen against the corrected path immediately.
                stale = data_reader._serial
                data_reader._serial = None
                if stale is not None:
                    try:
                        stale.close()
                    except Exception:
                        pass

        # A pyserial handle can report is_open=True while the underlying
        # device node is gone (that's exactly what produced the repeated
        # I/O errors above) — always force a fresh handle rather than
        # trusting the stale one.
        if self._cfg_serial is not None:
            try:
                self._cfg_serial.close()
            except Exception:
                pass
            self._cfg_serial = None

        try:
            self._cfg_serial = _serial.Serial(
                self._cfg_port, RADAR_CFG_BAUD, timeout=0.1
            )
            self._cfg_serial.dtr = True
            self._cfg_serial.rts = True
            self._cfg_serial.reset_input_buffer()
            self._apply_config()
            return True
        except Exception as e:
            log.warning("Radar reconfigure attempt failed (cfg_port=%s): %s",
                        self._cfg_port, e)
            return False

    def _track_frame_gap(self, frame_number: int):
        """Update the dropped-frame count from the sensor's own
        frame_number header sequence. Unit-testable independent of the
        serial threads."""
        if self._prev_header_frame_number is not None:
            gap = frame_number - self._prev_header_frame_number - 1
            if gap > 0:
                self._dropped_frames += gap
        self._prev_header_frame_number = frame_number

    def get_frame(self) -> RadarFrame | None:
        with self._lock:
            return self._latest

    def get_nearest(self, target_ts: int) -> RadarFrame | None:
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

    def get_new_since(self, after_ts_ns: int) -> list[RadarFrame]:
        """Return every buffered frame with timestamp_ns > after_ts_ns, oldest
        first — used by native-rate recorders to drain every frame since the
        last poll without skipping any received between polls.
        """
        with self._lock:
            return [f for f in self._buffer if f.timestamp_ns > after_ts_ns]

    @property
    def rate(self) -> float:
        with self._lock:
            return self._rate

    @property
    def connected(self) -> bool:
        """True if the DATA serial connection is currently open."""
        reader = getattr(self, "_data_reader", None)
        if reader is None:
            return False
        s = getattr(reader, "_serial", None)
        return bool(s and s.is_open)

    @property
    def streaming(self) -> bool:
        """True while complete AWR1843 UART frames continue to arrive.

        This is intentionally different from a camera↔radar synchronization
        result: the dashboard must not report a serial disconnect merely
        because one camera frame has no radar match.
        """
        with self._lock:
            last_frame_ns = self._last_frame_ns
        return self.connected and last_frame_ns > 0 and (
            time.monotonic_ns() - last_frame_ns <= RADAR_STREAM_TIMEOUT_NS
        )

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._frame_count

    @property
    def dropped_frames(self) -> int:
        """Frames missing per gaps in the sensor's own frame_number header."""
        with self._lock:
            return self._dropped_frames

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=2)
        self._data_reader.stop()
        if self._cfg_serial and self._cfg_serial.is_open:
            try:
                self._cfg_serial.write(b"sensorStop\n")
                time.sleep(0.05)
            except Exception:
                pass
            self._cfg_serial.close()
