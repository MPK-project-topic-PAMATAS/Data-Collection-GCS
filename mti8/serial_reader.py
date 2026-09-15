import serial
import serial.tools.list_ports
import threading
import time
import queue

from config.settings import MTI_PORT, MTI_BAUDRATE, MTI_TIMEOUT

# Xbus: Go to Measurement Mode
_CMD_GO_MEASUREMENT = bytes([0xFA, 0xFF, 0x31, 0x00, 0x31])


class SerialReader:
    def __init__(self):
        self._port = MTI_PORT
        self._baudrate = MTI_BAUDRATE
        self._queue = queue.Queue(maxsize=8000)
        self._serial = None
        self._thread = None
        self._running = False
        self._raw_file = None
        self._raw_lock = threading.Lock()

    def _auto_detect_port(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "ttyUSB" in p.device or "ttyACM" in p.device:
                return p.device
        for p in ports:
            desc = p.description.lower()
            if "motion" in desc or "mti" in desc or "xsens" in desc:
                return p.device
        if ports:
            return ports[0].device
        return None

    def start(self):
        port = self._auto_detect_port()
        if port is None:
            raise RuntimeError("No serial port found — plug in the MTi 8")

        self._port = port
        print(f"Opening {port} ...")

        self._serial = serial.Serial(
            port,
            self._baudrate,
            timeout=MTI_TIMEOUT,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self._serial.reset_input_buffer()

        # Ensure measurement mode
        self._serial.write(_CMD_GO_MEASUREMENT)
        time.sleep(0.1)

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print("Connected")

    def start_raw_log(self, path: str):
        """Start writing raw serial bytes to a binary file."""
        with self._raw_lock:
            self._raw_file = open(path, "wb")

    def stop_raw_log(self):
        """Stop writing raw serial bytes."""
        with self._raw_lock:
            if self._raw_file:
                self._raw_file.close()
                self._raw_file = None

    def _read_loop(self):
        while self._running:
            try:
                waiting = self._serial.in_waiting
                if waiting > 0:
                    data = self._serial.read(waiting)
                    with self._raw_lock:
                        if self._raw_file:
                            self._raw_file.write(data)
                    self._queue.put_nowait(data)
                else:
                    time.sleep(0.0001)
            except serial.SerialException:
                self._reconnect()
            except queue.Full:
                pass  # ponytail: drop oldest when GUI stalls, add warning log if throughput matters

    def _reconnect(self):
        print("Disconnected, reconnecting...")
        try:
            self._serial.close()
        except Exception:
            pass

        for _ in range(50):
            if not self._running:
                return
            try:
                self._serial = serial.Serial(
                    self._port, self._baudrate, timeout=MTI_TIMEOUT
                )
                self._serial.reset_input_buffer()
                self._serial.write(_CMD_GO_MEASUREMENT)
                time.sleep(0.1)
                print(f"Reconnected: {self._port}")
                return
            except serial.SerialException:
                time.sleep(0.1)

        print("Reconnect failed")

    def read_bytes(self, timeout=0.005):
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def stop(self):
        self._running = False
        self.stop_raw_log()
        if self._thread:
            self._thread.join(timeout=2)
        if self._serial and self._serial.is_open:
            self._serial.close()
        print("Serial closed")
