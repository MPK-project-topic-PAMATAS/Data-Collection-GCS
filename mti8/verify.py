#!/usr/bin/env python3
"""Verify MTi-8 parser output against MT Manager.

Runs the MTi-8 for a fixed duration, decodes every XDI field, and prints
the latest packet plus summary statistics. Compare the printed values
side-by-side with MT Manager.

Usage:
    python -m mti8.verify              # 10 seconds, default
    python -m mti8.verify --duration 30  # 30 seconds
"""
import argparse
import math
import time

from mti8.serial_reader import SerialReader
from mti8.xbus_parser import XbusParser, MTDATA2
from mti8.mtdata2_decoder import decode


def _fmt_vec(v, unit=""):
    if v is None:
        return "None"
    if len(v) == 3:
        return (
            f"({v[0]:+.6f}, {v[1]:+.6f}, {v[2]:+.6f}) "
            f"{unit}"
        )
    if len(v) == 4:
        return (
            f"({v[0]:+.6f}, {v[1]:+.6f}, {v[2]:+.6f}, "
            f"{v[3]:+.6f}) {unit}"
        )
    return str(v)


def _fmt_compact(vec, labels="xyz"):
    """Format 3-component vector compactly: x=... y=... z=..."""
    parts = [f"{lab}={v:.6f}" for lab, v in zip(labels, vec)]
    return "  ".join(parts)


def _print_frame(frame, count, prev_counter,
                 prev_sample_time, prev_time):
    norm = None
    if frame.quaternion:
        qw, qx, qy, qz = frame.quaternion  # XSens wire: (w, x, y, z)
        norm = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)

    dt_ms = None
    if prev_sample_time is not None and frame.sample_time_fine > 0:
        dt_ticks = frame.sample_time_fine - prev_sample_time
        # sample time fine is in 100ns ticks → ms
        dt_ms = dt_ticks / 10000.0

    wall_dt = None
    if prev_time is not None:
        wall_dt = (frame.timestamp_ns - prev_time) / 1e6

    counter_gap = None
    if prev_counter is not None and frame.packet_counter > 0:
        counter_gap = frame.packet_counter - prev_counter - 1

    sep = "=" * 60
    print(f"\n{sep}")
    print(f"  Packet #{count}  (counter={frame.packet_counter})")
    print(sep)

    print("\n  Timestamp:")
    print(f"    Packet Counter    : {frame.packet_counter}")
    secs = frame.sample_time_fine / 10000000
    print(f"    Sample Time Fine  : "
          f"{frame.sample_time_fine}  ({secs:.3f} s)")
    print(f"    Sample Time Coarse: "
          f"{frame.sample_time_coarse}")
    if dt_ms is not None:
        print(f"    dt (fine ticks)   : {dt_ms:.3f} ms")
    if wall_dt is not None:
        print(f"    dt (wall)         : {wall_dt:.2f} ms")
    if counter_gap is not None and counter_gap > 0:
        msg = "    *** PACKET LOSS: "
        msg += f"{counter_gap} packet(s) missed ***"
        print(msg)
    elif counter_gap == 0:
        print("    dcounter         : 1 (no loss)")

    print("\n  Orientation:")
    print(f"    Quaternion (w,x,y,z): "
          f"{_fmt_vec(frame.quaternion)}")
    if norm is not None:
        ok = abs(norm - 1.0) < 0.001
        tag = "OK" if ok else "*** NOT NORMALIZED ***"
        print(f"    Quaternion norm     : "
              f"{norm:.8f}  {tag}")
    print(f"    Euler (r,p,y)       : "
          f"{_fmt_vec(frame.euler, 'deg')}")
    if frame.rotation_matrix:
        print(f"    Rotation Matrix     : "
              f"{_fmt_vec(frame.rotation_matrix)}")

    print("\n  Calibrated Inertial:")
    print(f"    Acceleration (m/s2) : "
          f"{_fmt_vec(frame.acceleration, 'm/s2')}")
    if frame.acceleration:
        ax, ay, az = frame.acceleration
        mag = math.sqrt(ax**2 + ay**2 + az**2)
        print(f"    |accel|             : "
              f"{mag:.4f} m/s2  (expect ~9.81 if stationary)")
    print(f"    Gyroscope (rad/s)   : "
          f"{_fmt_vec(frame.gyroscope, 'rad/s')}")
    if frame.gyroscope:
        gx, gy, gz = frame.gyroscope
        mag = math.sqrt(gx**2 + gy**2 + gz**2)
        print(f"    |gyro|              : "
              f"{mag:.6f} rad/s  (expect ~0 if stationary)")
    print(f"    Magnetometer (a.u.) : "
          f"{_fmt_vec(frame.magnetometer, 'a.u.')}")
    if frame.magnetometer:
        mx, my, mz = frame.magnetometer
        mag = math.sqrt(mx**2 + my**2 + mz**2)
        print(f"    |mag|               : "
              f"{mag:.4f}  (should be ~1.0 normalized)")

    print("\n  Delta Values:")
    print(f"    Delta Velocity      : "
          f"{_fmt_vec(frame.delta_velocity, 'm/s')}")
    print(f"    Delta Quaternion    : "
          f"{_fmt_vec(frame.delta_quaternion)}")

    print("\n  Position:")
    print(f"    Latitude            : "
          f"{frame.latitude:.8f} deg")
    print(f"    Longitude           : "
          f"{frame.longitude:.8f} deg")
    print(f"    Altitude MSL        : "
          f"{frame.altitude_msl:.4f} m")
    print(f"    Altitude Ellipsoid  : "
          f"{frame.altitude_ellipsoid:.4f} m")

    print("\n  Velocity:")
    print(f"    Velocity (x,y,z)    : "
          f"{_fmt_vec(frame.velocity, 'm/s')}")

    print("\n  Environment:")
    print(f"    Temperature         : "
          f"{frame.temperature:.2f} C")
    print(f"    Barometric Pressure : "
          f"{frame.baro_pressure:.1f} Pa")

    print("\n  Status:")
    print(f"    Status Word         : "
          f"0x{frame.status_word:08X}")
    if frame.status_word:
        bits = []
        if frame.status_word & 0x01:
            bits.append("SelfTest")
        if frame.status_word & 0x02:
            bits.append("FilterValid")
        if frame.status_word & 0x04:
            bits.append("GNSSFix")
        if frame.status_word & 0x08:
            bits.append("NoRotation")
        if frame.status_word & 0x10:
            bits.append("Representative")
        flags = ", ".join(bits) if bits else "none"
        print(f"    Status Flags        : {flags}")
    print(f"    Status Byte         : "
          f"0x{frame.status_byte:02X}")


def verify(duration=10):
    reader = SerialReader()
    parser = XbusParser()
    reader.start()

    count = 0
    prev_counter = None
    prev_sample_time = None
    prev_time = None
    start = time.monotonic()
    last_frame = None

    # Counters for packet loss
    total_expected = 0
    total_received = 0
    prev_counter_for_loss = None

    # Quaternion norm stats
    norm_min = float("inf")
    norm_max = 0.0

    # Timestamp delta stats
    dt_list = []

    print(f"\nReading MTi-8 for {duration} seconds...")
    print("Compare the printed values with MT Manager.\n")

    try:
        while time.monotonic() - start < duration:
            data = reader.read_bytes()
            if data is None:
                continue

            parser.feed(data)
            packets = parser.parse_packets()

            for pkt in packets:
                if pkt.mid != MTDATA2:
                    continue

                frame = decode(pkt.payload)
                frame.timestamp_ns = time.monotonic_ns()
                count += 1
                last_frame = frame

                # Print first 3 frames and every 50th
                if count <= 3 or count % 50 == 0:
                    _print_frame(
                        frame, count, prev_counter,
                        prev_sample_time, prev_time,
                    )

                # Quaternion norm
                if frame.quaternion:
                    qw, qx, qy, qz = frame.quaternion  # XSens wire: (w,x,y,z)
                    norm = math.sqrt(
                        qw**2 + qx**2 + qy**2 + qz**2
                    )
                    norm_min = min(norm_min, norm)
                    norm_max = max(norm_max, norm)

                # Timestamp delta
                if (prev_sample_time is not None
                        and frame.sample_time_fine > 0):
                    dt = frame.sample_time_fine - prev_sample_time
                    if 0 < dt < 1000000:  # < 100ms sanity
                        dt_list.append(dt / 10000.0)

                # Packet loss
                if (prev_counter_for_loss is not None
                        and frame.packet_counter > 0):
                    gap = (frame.packet_counter
                           - prev_counter_for_loss - 1)
                    total_expected += gap + 1
                    total_received += 1
                elif prev_counter_for_loss is None:
                    total_received = 1

                prev_counter = frame.packet_counter
                prev_sample_time = frame.sample_time_fine
                prev_time = frame.timestamp_ns
                prev_counter_for_loss = frame.packet_counter

    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()

    if count == 0:
        print("No packets received! Check serial connection.")
        return

    # ── Summary ────────────────────────────────────────────────
    sep = "=" * 60
    print(f"\n{sep}")
    print(f"  SUMMARY ({count} packets in {duration}s)")
    print(sep)

    print("\n  Packet Loss:")
    if total_expected > 1:
        loss = total_expected - total_received
        pct = (loss / total_expected * 100
               if total_expected > 0 else 0)
        print(f"    Expected: ~{total_expected}  "
              f"Received: {total_received}  "
              f"Loss: {loss} ({pct:.2f}%)")
    else:
        print("    (not enough data)")

    print("\n  Quaternion Norm:")
    if norm_min < float("inf"):
        print(f"    Min: {norm_min:.8f}  "
              f"Max: {norm_max:.8f}  "
              f"(expect 1.0000 +/-0.001)")
    else:
        print("    (no quaternion data)")

    print("\n  Sample Time dt:")
    if dt_list:
        avg = sum(dt_list) / len(dt_list)
        mn = min(dt_list)
        mx = max(dt_list)
        print(f"    Avg: {avg:.2f} ms  "
              f"Min: {mn:.2f} ms  Max: {mx:.2f} ms")
        if abs(avg - 10.0) < 2.0:
            print("    -> Configured rate: ~100 Hz")
        elif abs(avg - 5.0) < 1.0:
            print("    -> Configured rate: ~200 Hz")
        elif abs(avg - 2.5) < 0.5:
            print("    -> Configured rate: ~400 Hz")
        else:
            hz = 1000.0 / avg
            print(f"    -> Rate: ~{hz:.0f} Hz (check config)")
    else:
        print("    (not enough data)")

    if last_frame:
        print("\n  Last Frame Values "
              "(for MT Manager comparison):")
        print(f"    Packet Counter : "
              f"{last_frame.packet_counter}")
        print(f"    Sample Time    : "
              f"{last_frame.sample_time_fine}")
        if last_frame.quaternion:
            q = last_frame.quaternion
            print(f"    Quaternion     : "
                  f"{_fmt_compact(q, 'wxyz')}")
        if last_frame.acceleration:
            a = last_frame.acceleration
            print(f"    Acceleration   : "
                  f"{_fmt_compact(a)}")
        if last_frame.gyroscope:
            g = last_frame.gyroscope
            print(f"    Gyroscope      : "
                  f"{_fmt_compact(g)}")
        if last_frame.magnetometer:
            m = last_frame.magnetometer
            print(f"    Magnetometer   : "
                  f"{_fmt_compact(m)}")
        print(f"    Status Word    : "
              f"0x{last_frame.status_word:08X}")

    print()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", "-d", type=int, default=10)
    args = ap.parse_args()
    verify(args.duration)
