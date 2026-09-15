"""Convert dataset to Kalibr format for camera-IMU calibration.

Supports both v1 (CSV/images/) and v2 (HDF5/Parquet/left/) formats.

Converts a recorded dataset into:
    1. Folder structure for kalibr_bagcreater (cam0/, cam1/, imu0.csv)
    2. A ROS1 bag file directly (using rosbags library)
    3. Kalibr config files (imu.yaml, camchain.yaml)

Usage:
    python calibration/convert_for_kalibr.py <dataset_path>
    python calibration/convert_for_kalibr.py <dataset_path> --bag calibration.bag
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import yaml

MTI_IMU_PARAMS = {
    "accelerometer_noise_density": 6.87e-03,
    "accelerometer_random_walk": 4.00e-04,
    "gyroscope_noise_density": 5.24e-05,
    "gyroscope_random_walk": 3.50e-06,
}


from dataset.reader import detect_format, load_v2_frames, load_v2_mti


def _load_v2(dataset_path: Path):
    """Load frames from v2 Parquet index + HDF5 sensor data."""
    import numpy as np

    raw_frames = load_v2_frames(dataset_path)
    mti_raw = load_v2_mti(dataset_path)

    # Convert MTi format: {ts: {ax,ay,az,gx,gy,gz}} -> {ts: {acceleration, gyroscope}}
    mti_data = {}
    for ts, s in mti_raw.items():
        mti_data[ts] = {
            "acceleration": [s["ax"], s["ay"], s["az"]],
            "gyroscope": [s["gx"], s["gy"], s["gz"]],
        }

    frames = []
    for row in raw_frames:
        ts_ns = int(row["timestamp_ns"])
        nearest_mti_ts = int(row.get("nearest_mti_timestamp_ns", 0))
        imu = mti_data.get(nearest_mti_ts)

        left_path = row.get("left_path", "")
        right_path = row.get("right_path", "")

        frames.append({
            "name": str(int(row.get("frame_id", 0))),
            "ts_ns": ts_ns,
            "left_jpg": dataset_path / left_path if left_path else None,
            "right_jpg": dataset_path / right_path if right_path else None,
            "imu": imu,
        })

    return frames


def _load_v1(dataset_path: Path):
    """Load frames from v1 CSV-based format."""
    left_dir = dataset_path / "images" / "left"
    if not left_dir.exists():
        return []

    # Try per-frame JSON metadata first
    frame_files = sorted(left_dir.glob("*.jpg")) + sorted(left_dir.glob("*.png"))
    if not frame_files:
        return []

    frames = []
    for frame_file in frame_files:
        name = frame_file.stem
        meta_file = dataset_path / "metadata" / f"{name}.json"
        imu_file = dataset_path / "imu" / f"{name}.json"

        if meta_file.exists():
            with open(meta_file) as f:
                meta = json.load(f)
            ts_ns = meta["camera_timestamp"]
        else:
            ts_ns = 0

        imu_data = None
        if imu_file.exists():
            with open(imu_file) as f:
                imu_data = json.load(f)

        frames.append({
            "name": name,
            "ts_ns": ts_ns,
            "left_jpg": left_dir / frame_file.name,
            "right_jpg": dataset_path / "images" / "right" / frame_file.name,
            "imu": imu_data,
        })

    return frames


def load_dataset(dataset_path: Path):
    fmt = detect_format(dataset_path)
    if fmt == "v2":
        return _load_v2(dataset_path)
    return _load_v1(dataset_path)


def convert_folder(dataset_path: Path, output_path: Path, calib_path: Path = None):
    frames = load_dataset(dataset_path)
    if not frames:
        print("No frames found")
        return

    cam0_dir = output_path / "cam0"
    cam1_dir = output_path / "cam1"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)

    imu_rows = []
    for frame in frames:
        ts_str = str(frame["ts_ns"])
        if frame["left_jpg"] and frame["left_jpg"].exists():
            shutil.copy2(frame["left_jpg"], cam0_dir / f"{ts_str}.png")
        if frame["right_jpg"] and frame["right_jpg"].exists():
            shutil.copy2(frame["right_jpg"], cam1_dir / f"{ts_str}.png")

        if frame["imu"] and frame["imu"].get("gyroscope") and frame["imu"].get("acceleration"):
            imu = frame["imu"]
            imu_rows.append({
                "timestamp": frame["ts_ns"],
                "omega_x": imu["gyroscope"][0],
                "omega_y": imu["gyroscope"][1],
                "omega_z": imu["gyroscope"][2],
                "alpha_x": imu["acceleration"][0],
                "alpha_y": imu["acceleration"][1],
                "alpha_z": imu["acceleration"][2],
            })

    imu_csv = output_path / "imu0.csv"
    with open(imu_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "timestamp", "omega_x", "omega_y", "omega_z",
            "alpha_x", "alpha_y", "alpha_z",
        ])
        writer.writeheader()
        writer.writerows(imu_rows)

    _write_configs(output_path, calib_path)

    print(f"  cam0: {len(frames)} images")
    print(f"  cam1: {len(frames)} images")
    print(f"  imu0: {len(imu_rows)} measurements")
    print(f"Output: {output_path}")


def convert_bag(dataset_path: Path, bag_path: Path, calib_path: Path = None):
    import numpy as np
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    from rosbags.typesys.stores.ros1_noetic import (
        builtin_interfaces__msg__Time,
        geometry_msgs__msg__Quaternion,
        geometry_msgs__msg__Vector3,
        sensor_msgs__msg__Image,
        sensor_msgs__msg__Imu,
        std_msgs__msg__Header,
    )

    typestore = get_typestore(Stores.ROS1_NOETIC)
    frames = load_dataset(dataset_path)
    bag_path.parent.mkdir(parents=True, exist_ok=True)

    def make_header(frame_id: str, ts_ns: int) -> std_msgs__msg__Header:
        return std_msgs__msg__Header(
            seq=0,
            stamp=builtin_interfaces__msg__Time(
                sec=ts_ns // 1_000_000_000,
                nanosec=ts_ns % 1_000_000_000,
            ),
            frame_id=frame_id,
        )

    with Writer(bag_path) as writer:
        cam0_conn = writer.add_connection("/cam0/image_raw", "sensor_msgs/msg/Image", typestore=typestore)
        cam1_conn = writer.add_connection("/cam1/image_raw", "sensor_msgs/msg/Image", typestore=typestore)
        imu_conn = writer.add_connection("/imu0", "sensor_msgs/msg/Imu", typestore=typestore)

        for frame in frames:
            ts_ns = frame["ts_ns"]

            for conn, img_path in [(cam0_conn, frame["left_jpg"]),
                                   (cam1_conn, frame["right_jpg"])]:
                if not img_path or not img_path.exists():
                    continue
                jpg_data = img_path.read_bytes()
                msg = sensor_msgs__msg__Image(
                    header=make_header("camera", ts_ns),
                    height=720,
                    width=1280,
                    encoding="rgb8",
                    is_bigendian=0,
                    step=1280 * 3,
                    data=np.frombuffer(jpg_data, dtype=np.uint8),
                )
                serialized = typestore.serialize_ros1(msg, "sensor_msgs/msg/Image")
                writer.write(conn, ts_ns, serialized)

            if frame["imu"] and frame["imu"].get("gyroscope") and frame["imu"].get("acceleration"):
                imu = frame["imu"]
                gyro = imu["gyroscope"]
                accel = imu["acceleration"]
                msg = sensor_msgs__msg__Imu(
                    header=make_header("imu_link", ts_ns),
                    orientation=geometry_msgs__msg__Quaternion(x=0, y=0, z=0, w=1),
                    orientation_covariance=np.zeros(9),
                    angular_velocity=geometry_msgs__msg__Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
                    angular_velocity_covariance=np.zeros(9),
                    linear_acceleration=geometry_msgs__msg__Vector3(x=accel[0], y=accel[1], z=accel[2]),
                    linear_acceleration_covariance=np.zeros(9),
                )
                serialized = typestore.serialize_ros1(msg, "sensor_msgs/msg/Imu")
                writer.write(imu_conn, ts_ns, serialized)

    print(f"ROS bag saved: {bag_path}")
    print(f"  {len(frames)} image pairs + IMU")


def _write_configs(output_path: Path, calib_path: Path = None):
    imu_yaml = output_path / "imu.yaml"
    imu_config = dict(MTI_IMU_PARAMS)
    imu_config["rostopic"] = "/imu0"
    imu_config["update_rate"] = 200.0
    with open(imu_yaml, "w") as f:
        yaml.dump(imu_config, f, default_flow_style=False)

    camchain = _generate_camchain(calib_path)
    camchain_yaml = output_path / "camchain.yaml"
    with open(camchain_yaml, "w") as f:
        yaml.dump(camchain, f, default_flow_style=False)

    print(f"  imu.yaml, camchain.yaml generated")


def _generate_camchain(calib_path: Path = None):
    calib = None
    if calib_path and calib_path.exists():
        with open(calib_path) as f:
            calib = yaml.safe_load(f)

    if calib:
        fx = calib["rectified"]["left"]["fx"]
        fy = calib["rectified"]["left"]["fy"]
        cx = calib["rectified"]["left"]["cx"]
        cy = calib["rectified"]["left"]["cy"]
        width = calib["resolution"]["width"]
        height = calib["resolution"]["height"]
    else:
        fx = fy = 521.92
        cx = 625.84
        cy = 371.58
        width, height = 1280, 720

    T_cam_imu = [
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    return {
        "cam0": {
            "camera_model": "pinhole",
            "intrinsics": [fx, fy, cx, cy],
            "distortion_model": "radtan",
            "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
            "T_cam_imu": T_cam_imu,
            "timeshift_cam_imu": 0.0,
            "rostopic": "/cam0/image_raw",
            "resolution": [width, height],
        },
        "cam1": {
            "camera_model": "pinhole",
            "intrinsics": [fx, fy, cx, cy],
            "distortion_model": "radtan",
            "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
            "T_cn_cnm1": [
                [1.0, 0.0, 0.0, -0.1196],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            "T_cam_imu": T_cam_imu,
            "timeshift_cam_imu": 0.0,
            "rostopic": "/cam1/image_raw",
            "resolution": [width, height],
        },
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert dataset for Kalibr")
    parser.add_argument("dataset", type=Path, help="Path to recorded dataset")
    parser.add_argument("-o", "--output", type=Path, default=Path("calibration/kalibr_input"))
    parser.add_argument("-c", "--calibration", type=Path,
                        default=Path("calibration/zed_intrinsics.yaml"))
    parser.add_argument("--bag", type=Path, default=None,
                        help="Also create a ROS1 bag file")
    args = parser.parse_args()

    convert_folder(args.dataset, args.output, args.calibration)

    if args.bag:
        convert_bag(args.dataset, args.bag, args.calibration)
        print(f"\nRun Kalibr with:")
        print(f"  kalibr_calibrate_imu_camera --bag {args.bag} \\")
        print(f"    --cam {args.output}/camchain.yaml --imu {args.output}/imu.yaml \\")
        print(f"    --target april_6x6.yaml --imu-models calibrated")
    else:
        print(f"\nTo create a ROS bag (no ROS needed):")
        print(f"  python calibration/convert_for_kalibr.py {args.dataset} --bag calibration.bag")
