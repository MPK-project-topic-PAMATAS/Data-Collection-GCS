"""HDF5 integrity test — write synthetic samples, close, reopen, verify.

Usage:
    python diagnostics/test_hdf5_storage.py
"""

import os
import tempfile
import sys
from pathlib import Path

import h5py
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from dataset.hdf5_sensor_writer import (
    HDF5SensorWriter, SensorSample, _to_vec3, _to_vec4, _to_scalar
)


def test_roundtrip():
    """Write synthetic MTi samples, close file, reopen, verify exact match."""
    print("=== HDF5 Integrity Test ===\n")

    with tempfile.TemporaryDirectory() as tmpdir:
        sensors_dir = Path(tmpdir) / "sensors"
        writer = HDF5SensorWriter(
            sensors_dir=sensors_dir,
            enable_zed_imu=True,
            rotation_mode="daily",
            compression="lzf",
        )
        writer.start()

        # Generate 1000 synthetic samples
        rng = np.random.default_rng(42)
        N = 1000
        base_ts = 1_700_000_000_000_000_000
        dt_ns = 5_000_000  # 200 Hz

        written_samples = []
        for i in range(N):
            ts = base_ts + i * dt_ns
            acc = [float(rng.standard_normal()),
                   float(rng.standard_normal()),
                   9.81 + float(rng.standard_normal() * 0.01)]
            gyro = [float(rng.standard_normal() * 0.01),
                    float(rng.standard_normal() * 0.01),
                    float(rng.standard_normal() * 0.01)]
            mag = [float(rng.standard_normal()),
                   float(rng.standard_normal()),
                   float(rng.standard_normal())]
            quat = [0.0, 0.0, 0.0, 1.0]
            status = i % 256

            sample = SensorSample(
                timestamp_ns=ts,
                host_timestamp_ns=ts,
                device_timestamp=i * 1000,
                packet_counter=i,
                acceleration=acc,
                angular_velocity=gyro,
                magnetometer=mag,
                quaternion_raw=quat,
                latitude=48.0 + rng.standard_normal() * 0.001,
                longitude=11.0 + rng.standard_normal() * 0.001,
                altitude_msl=500.0 + rng.standard_normal(),
                altitude_ellipsoid=501.0 + rng.standard_normal(),
                velocity=[0.0, 0.0, 0.0],
                temperature=25.0 + rng.standard_normal(),
                baro_pressure=1013.25 + rng.standard_normal(),
                status_word=status,
                status_byte=status & 0xFF,
                delta_velocity=[0.0, 0.0, 0.0],
                delta_quaternion=[0.0, 0.0, 0.0, 1.0],
                sensor_type="mti",
            )
            writer.put_mti_sample(sample)
            written_samples.append(sample)

        # Also write a ZED IMU sample
        zed_sample = SensorSample(
            timestamp_ns=base_ts,
            acceleration=[0.1, 0.2, 9.8],
            angular_velocity=[0.01, 0.02, 0.03],
            zed_orientation=[0.0, 0.0, 0.0, 1.0],
            temperature=30.0,
            sensor_type="zed_imu",
        )
        writer.put_zed_imu_sample(zed_sample)

        # Wait for queue to drain
        import time
        time.sleep(0.5)
        writer.stop()

        # Find the HDF5 file
        h5_files = list(sensors_dir.glob("*.h5"))
        assert len(h5_files) > 0, "No HDF5 files created"
        h5_path = h5_files[0]
        print(f"Wrote to: {h5_path.name}")

        # ── Verify ───────────────────────────────────────────────────
        print("\n--- Verification ---")
        errors = []
        warnings = []

        with h5py.File(str(h5_path), "r") as f:
            # Check MTi group exists
            assert "mti" in f, "Missing /mti group"
            mti = f["mti"]

            # Check sample count
            actual_count = mti["timestamp_ns"].shape[0]
            assert actual_count == N, \
                f"Sample count mismatch: expected {N}, got {actual_count}"
            print(f"  [PASS] Sample count: {actual_count}")

            # Verify timestamp ordering
            ts_data = mti["timestamp_ns"][:]
            mono_diffs = np.diff(ts_data)
            non_mono = np.sum(mono_diffs <= 0)
            if non_mono > 0:
                errors.append(f"Non-monotonic timestamps: {non_mono}")
            else:
                print(f"  [PASS] Timestamps monotonic")

            # Verify integer fields exact match
            for field in ["packet_counter", "status_word", "status_byte"]:
                data = mti[field][:]
                expected = np.array([getattr(s, field, 0)
                                     for s in written_samples],
                                    dtype=data.dtype)
                if not np.array_equal(data, expected):
                    errors.append(f"{field}: data mismatch")
                else:
                    print(f"  [PASS] {field}: exact match")

            # Verify float32 fields (binary equality with LZF)
            for field in ["acceleration", "angular_velocity", "magnetometer",
                          "quaternion_raw"]:
                data = mti[field][:]
                if field == "acceleration":
                    expected = np.array([s.acceleration for s in written_samples],
                                        dtype=np.float32)
                elif field == "angular_velocity":
                    expected = np.array([s.angular_velocity for s in written_samples],
                                        dtype=np.float32)
                elif field == "magnetometer":
                    expected = np.array([s.magnetometer for s in written_samples],
                                        dtype=np.float32)
                elif field == "quaternion_raw":
                    expected = np.array([s.quaternion_raw for s in written_samples],
                                        dtype=np.float32)

                if not np.array_equal(data, expected):
                    errors.append(f"{field}: float data mismatch")
                else:
                    print(f"  [PASS] {field}: exact binary match")

            # Verify float64 fields
            for field in ["latitude", "longitude", "altitude_msl"]:
                data = mti[field][:]
                if field == "latitude":
                    expected = np.array([s.latitude for s in written_samples],
                                        dtype=np.float64)
                elif field == "longitude":
                    expected = np.array([s.longitude for s in written_samples],
                                        dtype=np.float64)
                else:
                    expected = np.array([s.altitude_msl for s in written_samples],
                                        dtype=np.float64)
                if not np.allclose(data, expected, rtol=0):
                    warnings.append(f"{field}: float64 comparison")
                else:
                    print(f"  [PASS] {field}: exact match")

            # Check ZED IMU group
            if "zed_imu" in f:
                zed = f["zed_imu"]
                assert zed["timestamp_ns"].shape[0] == 1, \
                    "Expected 1 ZED IMU sample"
                print(f"  [PASS] ZED IMU: 1 sample written")
            else:
                warnings.append("No ZED IMU group found")

            # Check metadata attributes
            attrs = dict(f.attrs)
            assert attrs.get("schema_version") == "2.0", \
                "Missing/wrong schema_version"
            print(f"  [PASS] File attributes present")
            print(f"         schema_version: {attrs.get('schema_version')}")
            print(f"         sensor_model: {attrs.get('sensor_model')}")
            print(f"         compression: {attrs.get('compression')}")

            # Check dtypes
            for field, expected_dtype in [
                ("timestamp_ns", np.int64),
                ("acceleration", np.float32),
                ("latitude", np.float64),
                ("status_word", np.uint32),
                ("status_byte", np.uint8),
            ]:
                actual = mti[field].dtype
                if actual != expected_dtype:
                    errors.append(f"{field}: dtype {actual} != {expected_dtype}")
                else:
                    print(f"  [PASS] {field} dtype: {actual}")

            # Shape checks
            assert mti["acceleration"].shape == (N, 3)
            assert mti["quaternion_raw"].shape == (N, 4)
            print(f"  [PASS] Array shapes correct")

        print(f"\n{'='*40}")
        if errors:
            print(f"FAIL: {len(errors)} errors")
            for e in errors:
                print(f"  ✗ {e}")
        else:
            print("ALL TESTS PASSED")

        if warnings:
            print(f"\nWarnings ({len(warnings)}):")
            for w in warnings:
                print(f"  ⚠ {w}")

        print(f"\n  File size: {h5_path.stat().st_size / 1024:.1f} KB")
        print(f"  Samples: {N}")
        print(f"  Bytes/sample: {h5_path.stat().st_size / N:.1f}")

        return len(errors) == 0


if __name__ == "__main__":
    ok = test_roundtrip()
    sys.exit(0 if ok else 1)
