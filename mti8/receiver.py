import threading
import time
from collections import deque
from mti8.serial_reader import SerialReader
from mti8.xbus_parser import XbusParser, MTDATA2
from mti8.mtdata2_decoder import decode
from mti8.validator import MTiValidator, MTiSampleState


class MTiReceiver:
    def __init__(self, buffer_size=300):
        self._reader = SerialReader()
        self._parser = XbusParser()
        self._validator = MTiValidator()
        self._latest = None
        self._latest_state = MTiSampleState.NO_PACKET
        self._buffer = deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._frame_count = 0
        self._start_time = 0.0
        self._rate = 0.0
        self._firmware = ""

    def start(self):
        self._reader.start()
        self._running = True
        self._start_time = time.monotonic()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def start_raw_log(self, path: str):
        self._reader.start_raw_log(path)

    def stop_raw_log(self):
        self._reader.stop_raw_log()

    def _loop(self):
        while self._running:
            data = self._reader.read_bytes()
            if data is None:
                continue

            self._parser.feed(data)
            packets = self._parser.parse_packets()

            for pkt in packets:
                if pkt.mid != MTDATA2:
                    continue

                frame = decode(pkt.payload)
                frame.timestamp_ns = time.monotonic_ns()

                with self._lock:
                    validated, state = self._validator.validate(frame)
                    self._latest = validated
                    self._latest_state = state
                    self._buffer.append(validated)
                    self._frame_count += 1
                    elapsed = time.monotonic() - self._start_time
                    self._rate = (
                        self._frame_count / elapsed if elapsed > 0 else 0.0
                    )

    def get_frame(self):
        with self._lock:
            return self._latest

    def get_validated_frame(self):
        """Return (MTiFrame, MTiSampleState) — always valid data."""
        with self._lock:
            return self._latest, self._latest_state

    @property
    def sample_state(self):
        with self._lock:
            return self._latest_state

    @property
    def counters(self):
        with self._lock:
            c = self._validator.counters
            return (c.total_packets, c.valid_packets,
                    c.invalid_packets, c.dropped_packets,
                    c.suspicious_packets)

    def get_nearest(self, target_ts):
        with self._lock:
            if not self._buffer:
                return None

            best = None
            best_delta = float("inf")

            for frame in reversed(self._buffer):
                delta = abs(frame.timestamp_ns - target_ts)
                if delta >= best_delta:
                    break
                best = frame
                best_delta = delta

            return best

    def get_new_since(self, after_ts_ns: int) -> list:
        """Return every buffered frame with timestamp_ns > after_ts_ns, oldest
        first. Used by native-rate recorders to drain every sample since the
        last poll without skipping any that arrived between polls — unlike
        get_frame()/get_nearest(), which only ever see the single latest or
        nearest-matched sample.
        """
        with self._lock:
            return [f for f in self._buffer if f.timestamp_ns > after_ts_ns]

    @property
    def running(self):
        return self._running

    @property
    def rate(self):
        return self._rate

    @property
    def firmware(self):
        return self._firmware

    @firmware.setter
    def firmware(self, value: str):
        self._firmware = value

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self._reader.stop()
