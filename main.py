import time
import cv2
from camera.zed_camera import ZEDCamera
from mti8.receiver import MTiReceiver
from fusion.synchronizer import Synchronizer
from dataset.writer import DatasetWriter
from dataset.clock import capture_wall_offset_ns, mono_to_wall_ns
from config.settings import MAX_FRAMES, DATASET_DIR


def main():
    camera = ZEDCamera()
    mti = MTiReceiver()
    sync = Synchronizer(mti)
    writer = DatasetWriter(root=DATASET_DIR)

    camera.open()
    mti.start()
    writer.save_calibration(camera.calib)
    time.sleep(0.2)

    writer.begin_session(time.time_ns())
    t0 = time.monotonic()
    last_mti_ns = 0
    wall_offset_ns = capture_wall_offset_ns()

    try:
        while True:
            # MTi is recorded independently of the camera at its own native
            # rate — a missing camera frame never removes an IMU sample.
            # mti_frame.timestamp_ns is host CLOCK_MONOTONIC, not a UTC
            # epoch — convert it or every sample lands under 1970-01-01.
            for imu_frame in mti.get_new_since(last_mti_ns):
                writer.write_mti_sample(
                    imu_frame,
                    mono_to_wall_ns(imu_frame.timestamp_ns, wall_offset_ns),
                )
                last_mti_ns = imu_frame.timestamp_ns

            frame = camera.grab()
            if frame is None:
                continue

            # `sync` is used only to print a live residual below — recording
            # the camera frame never depends on it.
            synced = sync.sync(frame)
            writer.write_camera_frame(frame, synced)

            elapsed = time.monotonic() - t0
            fps = writer.frame_count / elapsed if elapsed > 0 else 0
            delta_ms = synced.delta_ns / 1e6 if synced else -1.0

            print(f"\r[{writer.frame_count:5d}/{MAX_FRAMES}]  "
                  f"FPS: {fps:5.1f}  dt: {delta_ms:5.2f}ms  "
                  f"MTi: {mti.rate:.0f}Hz  "
                  f"drop: {writer.dropped}",
                  end="", flush=True)

            if writer.frame_count >= MAX_FRAMES:
                break

    except KeyboardInterrupt:
        pass

    print()
    writer.end_session(time.time_ns())
    try:
        writer.close()
    except KeyboardInterrupt:
        pass
    camera.close()
    mti.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
