"""Record a calibration dataset for camera-IMU extrinsic estimation.

Records synchronized stereo + IMU data while you perform calibration motions.
Target: 2-5 minutes of excitation in all 6 degrees of freedom.

Motions to perform:
  - Slow translations along X, Y, Z
  - Rotations: roll, pitch, yaw
  - Figure-eight patterns
  - Circular motions
  - Gentle tilting

Avoid: violent shaking, long stationary periods, motion blur.

Usage:
    python calibration/record_calibration.py
    python calibration/record_calibration.py --duration 180
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2

from camera.zed_camera import ZEDCamera
from mti8.receiver import MTiReceiver
from fusion.synchronizer import Synchronizer
from dataset.writer import DatasetWriter
from dataset.clock import capture_wall_offset_ns, mono_to_wall_ns


def record(duration_sec: int = 180):
    camera = ZEDCamera()
    mti = MTiReceiver()
    sync = Synchronizer(mti)
    writer = DatasetWriter(root="calibration/dataset", save_depth=False)

    camera.open()
    mti.start()
    writer.save_calibration(camera.calib)
    time.sleep(0.2)

    print()
    print("=" * 60)
    print("  CALIBRATION RECORDING")
    print(f"  Duration : {duration_sec} s")
    print(f"  Output   : {writer.root}")
    print()
    print("  Perform slow, smooth motions:")
    print("    - Translate along X, Y, Z")
    print("    - Rotate: roll, pitch, yaw")
    print("    - Figure-eight patterns")
    print("    - Circular motions")
    print("    - Gentle tilting")
    print()
    print("  Avoid: violent shaking, long pauses")
    print("=" * 60)
    print()

    t0 = time.monotonic()
    last_mti_ns = 0
    wall_offset_ns = capture_wall_offset_ns()

    try:
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= duration_sec:
                break

            # MTi is recorded independently of the camera at its own native
            # rate; offline synchronization (synchronize-dataset) builds the
            # matched pairs Kalibr/extrinsic calibration needs afterward.
            # imu_frame.timestamp_ns is host CLOCK_MONOTONIC — convert it or
            # every sample lands under a 1970-01-01 date.
            for imu_frame in mti.get_new_since(last_mti_ns):
                writer.write_mti_sample(
                    imu_frame,
                    mono_to_wall_ns(imu_frame.timestamp_ns, wall_offset_ns),
                )
                last_mti_ns = imu_frame.timestamp_ns

            frame = camera.grab()
            if frame is None:
                continue

            synced = sync.sync(frame)
            writer.write_camera_frame(frame, synced)

            remaining = duration_sec - elapsed
            fps = writer.frame_count / elapsed if elapsed > 0 else 0

            print(f"\r[{writer.frame_count:5d}]  "
                  f"FPS: {fps:5.1f}  "
                  f"Remaining: {remaining:5.0f}s  "
                  f"drop: {writer.dropped}",
                  end="", flush=True)

    except KeyboardInterrupt:
        pass

    print()
    try:
        writer.close()
    except KeyboardInterrupt:
        pass
    camera.close()
    mti.stop()
    cv2.destroyAllWindows()

    print(f"\nCalibration dataset saved to {writer.root}")
    print(f"Frames: {writer.frame_count}, Dropped: {writer.dropped}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record calibration dataset")
    parser.add_argument("--duration", "-d", type=int, default=180,
                        help="Recording duration in seconds (default: 180)")
    args = parser.parse_args()
    record(args.duration)
