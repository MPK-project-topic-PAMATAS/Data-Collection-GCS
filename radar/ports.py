"""TI AWR1843 dual-COM port detection.

The EVM enumerates as two ttyACM devices (CFG @ 115200, DATA @ 921600) whose
numbers can change across reboots/inserts. Detect the TI ports and tell which
one streams mmWave frames (the DATA port) vs echoes config (the CFG port).
"""

from __future__ import annotations

import glob
import struct
import time

TI_VENDOR = "0451"
TI_PRODUCT = "bef3"
_MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"
_TOTAL_LEN_OFFSET = 12


def _vendor_product(tty: str) -> str | None:
    """Return 'VID:PID' of the USB device backing a ttyACM node, if any."""
    try:
        # /sys/class/tty/ttyACMx -> .../<usb>/<port>/<iface>/tty/ttyACMx
        import os
        dev = os.path.realpath(f"/sys/class/tty/{os.path.basename(tty)}/device")
        # realpath collapses the symlink chain to the USB interface dir;
        # one parent up is the USB device (holds VID:PID).
        usb_dev = os.path.realpath(os.path.join(dev, ".."))
        vid = open(os.path.join(usb_dev, "idVendor")).read().strip()
        pid = open(os.path.join(usb_dev, "idProduct")).read().strip()
        return f"{vid}:{pid}"
    except OSError:
        return None


def ti_ports() -> list[str]:
    """All /dev/ttyACM* nodes that belong to a TI CC1352R1 (0451:bef3)."""
    ports = []
    for tty in sorted(glob.glob("/dev/ttyACM*")):
        if _vendor_product(tty) == f"{TI_VENDOR}:{TI_PRODUCT}":
            ports.append(tty)
    return ports


def _iface_number(tty: str) -> int:
    """USB interface number backing a ttyACM node (0 if unknown)."""
    import os
    try:
        dev = os.path.realpath(f"/sys/class/tty/{os.path.basename(tty)}/device")
        return int(open(os.path.join(dev, "bInterfaceNumber")).read().strip(), 16)
    except (OSError, ValueError):
        return 0


def _is_data_port(path: str, baud: int, timeout_s: float = 1.0) -> bool:
    """Open the port at the DATA baud and check for a frame magic in the stream.

    DTR/RTS are asserted (high) so probing does not hold the EVM in reset and
    the sensor keeps running.
    """
    import serial

    try:
        s = serial.Serial(path, baud, timeout=0.05)
        s.dtr = True
        s.rts = True
    except Exception:
        return False
    try:
        buf = bytearray()
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            s.timeout = 0.05
            chunk = s.read(s.in_waiting or 64)
            if chunk:
                buf.extend(chunk)
            idx = buf.find(_MAGIC)
            if idx >= 0 and len(buf) >= idx + _TOTAL_LEN_OFFSET + 4:
                return True
            keep = max(0, len(buf) - 8)
            del buf[:keep]
    except Exception:
        return False
    finally:
        try:
            s.close()
        except Exception:
            pass
    return False


def detect_ports(data_baud: int = 921600) -> tuple[str | None, str | None]:
    """Return (data_port, cfg_port). Each is None if not found.

    The TI EVM exposes two ACM interfaces: CFG (CLI, 115200) is the first
    CDC interface and DATA (921600) the second/higher one. Order by interface
    number so the mapping is stable regardless of ttyACM* numbering.
    """
    ports = sorted(ti_ports(), key=_iface_number)
    if not ports:
        return None, None
    if len(ports) >= 2:
        return ports[-1], ports[0]  # (data, cfg)
    # only one TI port found; prefer a live frame probe to disambiguate
    if _is_data_port(ports[0], data_baud):
        return ports[0], None
    return None, ports[0]
