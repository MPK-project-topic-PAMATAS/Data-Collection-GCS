"""NTRIP client for receiving RTK corrections from an NTRIP caster.

Sends GGA sentences to the caster and receives RTCM correction streams.

Usage:
    client = NTRIPClient("195.182.72.152", 2111, "user", "pass")
    client.connect("VRS_CMRx")
    for data in client.stream():
        serial_port.write(data)  # feed to GPS receiver
"""

import base64
import socket


class NTRIPClient:
    """Minimal NTRIP 1.0 client."""

    BUFFER_SIZE = 4096

    def __init__(self, host: str, port: int, username: str, password: str,
                 timeout: float = 10.0):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.timeout = timeout

        self._sock = None
        self._connected = False
        self._running = False
        self._thread = None
        self._latest_gga = ""
        self._bytes_received = 0

    def connect(self, mountpoint: str) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(self.timeout)
            self._sock.connect((self.host, self.port))

            credentials = base64.b64encode(
                f"{self.username}:{self.password}".encode()
            ).decode()

            request = (
                f"GET /{mountpoint} HTTP/1.1\r\n"
                f"Host: {self.host}:{self.port}\r\n"
                f"Ntrip-Version: Ntrip/1.0\r\n"
                f"User-Agent: NTRIP/1.0\r\n"
                f"Authorization: Basic {credentials}\r\n"
                f"\r\n"
            )
            self._sock.sendall(request.encode())

            response = self._sock.recv(1024).decode("utf-8", errors="replace")
            if "200 OK" in response:
                self._connected = True
                self._running = True
                return True
            else:
                self._sock.close()
                self._sock = None
                return False

        except Exception:
            if self._sock:
                self._sock.close()
                self._sock = None
            return False

    def send_gga(self, gga_sentence: str):
        if not self._connected or not self._sock:
            return
        try:
            if not gga_sentence.endswith("\r\n"):
                gga_sentence += "\r\n"
            self._sock.sendall(gga_sentence.encode())
            self._latest_gga = gga_sentence
        except Exception:
            self._connected = False

    def stream(self):
        """Yields RTCM correction bytes from the caster."""
        while self._running and self._connected:
            try:
                data = self._sock.recv(self.BUFFER_SIZE)
                if not data:
                    break
                self._bytes_received += len(data)
                yield data
            except socket.timeout:
                continue
            except Exception:
                break
        self._connected = False

    def disconnect(self):
        self._running = False
        self._connected = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def bytes_received(self) -> int:
        return self._bytes_received
