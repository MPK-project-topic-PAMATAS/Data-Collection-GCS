from __future__ import annotations
from dataset.frame import Frame
from pyzed import sl
import cv2
import numpy as np
import time

from config.settings import (
    CAMERA_RESOLUTION,
    CAMERA_FPS,
    CAMERA_FLIP_180,
    DEPTH_MODE,
    UNIT,
    COORDINATE_SYSTEM,
    DEPTH_MIN_M,
    DEPTH_MAX_M,
    DEPTH_STABILIZATION,
    CONFIDENCE_THRESHOLD,
    TEXTURE_CONFIDENCE_THRESHOLD,
    SAVE_DISPARITY,
    SAVE_CONFIDENCE,
    SVO_COMPRESSION_MODE,
)

# Maps config.settings.SVO_COMPRESSION_MODE strings to the
# sl.SVO_COMPRESSION_MODE members actually available in the installed SDK.
# Built lazily (module import already needs pyzed, so this is safe at import
# time) rather than hardcoding a fixed set that could drift from the SDK.
_SVO_COMPRESSION_MODES = {
    name: getattr(sl.SVO_COMPRESSION_MODE, name)
    for name in dir(sl.SVO_COMPRESSION_MODE)
    if not name.startswith("_") and name.isupper()
}


class ZEDCamera:
    def __init__(self):
        self.camera = sl.Camera()

        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = CAMERA_RESOLUTION
        self.init_params.camera_fps = CAMERA_FPS
        self.init_params.depth_mode = DEPTH_MODE
        self.init_params.coordinate_units = UNIT
        self.init_params.coordinate_system = COORDINATE_SYSTEM
        self.init_params.depth_minimum_distance = DEPTH_MIN_M
        self.init_params.depth_maximum_distance = DEPTH_MAX_M
        self.init_params.depth_stabilization = DEPTH_STABILIZATION

        self.runtime = sl.RuntimeParameters()
        self.runtime.confidence_threshold = CONFIDENCE_THRESHOLD
        self.runtime.texture_confidence_threshold = TEXTURE_CONFIDENCE_THRESHOLD

        self.left_mat = sl.Mat()
        self.right_mat = sl.Mat()
        self.depth_mat = sl.Mat()
        self.conf_mat = sl.Mat()
        self.disp_mat = sl.Mat()
        self.view_depth_mat = sl.Mat()
        self.view_conf_mat = sl.Mat()
        self.sensors_data = sl.SensorsData()
        self.frame_id = 0

        self.calib = None
        self._recording = False
        self._svo_path = None

    def is_opened(self) -> bool:
        return self.camera.is_opened()

    def open(self):
        status = self.camera.open(self.init_params)

        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {status}")

        info = self.camera.get_camera_information()
        left = info.camera_configuration.calibration_parameters.left_cam

        self.calib = {
            "model": str(info.camera_model),
            "serial": str(info.serial_number),
            "sdk_version": self._safe_sdk_version(),
            "fx": left.fx,
            "fy": left.fy,
            "cx": left.cx,
            "cy": left.cy,
            "k1": left.disto[0],
            "k2": left.disto[1],
            "p1": left.disto[2],
            "p2": left.disto[3],
            "baseline": (
                info.camera_configuration
                .calibration_parameters
                .get_camera_baseline()
            ),
            "width": info.camera_configuration.resolution.width,
            "height": info.camera_configuration.resolution.height,
            "fps": info.camera_configuration.fps,
        }

        self._print_config(info)

    def _safe_sdk_version(self) -> str:
        try:
            return str(sl.get_sdk_version())
        except Exception:
            return "unknown"

    def _print_config(self, info):
        print("=" * 60)
        print("ZED Camera Connected")
        print(f"Model      : {info.camera_model}")
        print(f"Serial No. : {info.serial_number}")
        print(f"Resolution : {self.calib['width']}x{self.calib['height']}")
        print(f"FPS        : {self.calib['fps']}")
        print(f"fx         : {self.calib['fx']}")
        print(f"baseline   : {self.calib['baseline']}")
        print("-" * 60)
        print("Depth Configuration")
        print(f"  Mode               : {DEPTH_MODE}")
        print(f"  Units              : {UNIT}")
        print(f"  Coordinate System  : {COORDINATE_SYSTEM}")
        print(f"  Min Distance       : {DEPTH_MIN_M} m")
        print(f"  Max Distance       : {DEPTH_MAX_M} m")
        print(f"  Stabilization      : {'ON' if DEPTH_STABILIZATION else 'OFF'}")
        print(f"  Confidence Thresh  : {CONFIDENCE_THRESHOLD}")
        print(f"  Texture Conf Thresh: {TEXTURE_CONFIDENCE_THRESHOLD}")
        print("=" * 60)

    def start_recording(self, svo_path: str):
        if self._recording:
            return
        mode_name = SVO_COMPRESSION_MODE
        if mode_name not in _SVO_COMPRESSION_MODES:
            raise ValueError(
                f"Unknown SVO_COMPRESSION_MODE {mode_name!r}; valid values "
                f"for this ZED SDK: {sorted(_SVO_COMPRESSION_MODES)}"
            )
        rec_params = sl.RecordingParameters()
        rec_params.video_filename = svo_path
        rec_params.compression_mode = _SVO_COMPRESSION_MODES[mode_name]
        err = self.camera.enable_recording(rec_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"SVO recording failed: {err}")
            return
        self._recording = True
        self._svo_path = svo_path
        if self.calib is not None:
            self.calib["svo_compression_mode"] = mode_name
        print(f"SVO recording: {svo_path} (compression={mode_name})")

    def stop_recording(self):
        if not self._recording:
            return
        self.camera.disable_recording()
        self._recording = False
        print("SVO recording stopped")

    def grab(self):
        if self.camera.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return None

        timestamp = time.monotonic_ns()
        wall_timestamp = time.time_ns()

        # Stereo images
        self.camera.retrieve_image(self.left_mat, sl.VIEW.LEFT)
        self.camera.retrieve_image(self.right_mat, sl.VIEW.RIGHT)

        # Raw measures — stored to disk unmodified
        self.camera.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH)
        if SAVE_CONFIDENCE:
            self.camera.retrieve_measure(self.conf_mat, sl.MEASURE.CONFIDENCE)
        if SAVE_DISPARITY:
            self.camera.retrieve_measure(self.disp_mat, sl.MEASURE.DISPARITY)

        # Views — for dashboard visualization only
        self.camera.retrieve_image(self.view_depth_mat, sl.VIEW.DEPTH)
        self.camera.retrieve_image(self.view_conf_mat, sl.VIEW.CONFIDENCE)

        left = cv2.cvtColor(self.left_mat.get_data(), cv2.COLOR_BGRA2BGR)
        right = cv2.cvtColor(self.right_mat.get_data(), cv2.COLOR_BGRA2BGR)

        # Raw measures — float32 for dataset storage (no filtering applied)
        depth = self.depth_mat.get_data().copy()
        confidence = self.conf_mat.get_data().copy() if SAVE_CONFIDENCE else None
        disparity = self.disp_mat.get_data().copy() if SAVE_DISPARITY else None

        # Views — uint8 BGRA for dashboard display
        view_depth = self.view_depth_mat.get_data().copy()
        view_confidence = self.view_conf_mat.get_data().copy()

        if CAMERA_FLIP_180:
            left = cv2.rotate(left, cv2.ROTATE_180)
            right = cv2.rotate(right, cv2.ROTATE_180)
            depth = np.rot90(depth, 2).copy()
            if confidence is not None:
                confidence = np.rot90(confidence, 2).copy()
            if disparity is not None:
                disparity = np.rot90(disparity, 2).copy()
            view_depth = cv2.rotate(view_depth, cv2.ROTATE_180)
            view_confidence = cv2.rotate(view_confidence, cv2.ROTATE_180)

        zed_imu = self._get_imu()
        exposure, gain, temperature = self._get_camera_metrics()

        frame = Frame(
            frame_id=self.frame_id,
            timestamp_ns=timestamp,
            left=left,
            right=right,
            depth=depth,
            zed_imu=zed_imu,
            confidence=confidence,
            disparity=disparity,
            view_depth=view_depth,
            view_confidence=view_confidence,
            exposure=exposure,
            gain=gain,
            temperature=temperature,
            wall_timestamp_ns=wall_timestamp,
        )

        self.frame_id += 1

        return frame

    def _get_imu(self) -> dict | None:
        if self.camera.get_sensors_data(
            self.sensors_data, sl.TIME_REFERENCE.CURRENT
        ) != sl.ERROR_CODE.SUCCESS:
            return None

        imu = self.sensors_data.get_imu_data()
        mag = self.sensors_data.get_magnetometer_data()

        return {
            "angular_velocity": [
                imu.get_angular_velocity()[0],
                imu.get_angular_velocity()[1],
                imu.get_angular_velocity()[2],
            ],
            "linear_acceleration": [
                imu.get_linear_acceleration()[0],
                imu.get_linear_acceleration()[1],
                imu.get_linear_acceleration()[2],
            ],
            "orientation": imu.get_pose().get_orientation().get(),
            "magnetic_field": [
                mag.get_magnetic_field_calibrated()[0],
                mag.get_magnetic_field_calibrated()[1],
                mag.get_magnetic_field_calibrated()[2],
            ] if mag and mag.is_available else None,
            "temperature": self.sensors_data.get_temperature_data().get(
                sl.SENSOR_LOCATION.ONBOARD_LEFT
            ),
            "timestamp": imu.timestamp.get_nanoseconds(),
        }

    def _get_camera_metrics(self) -> tuple:
        """Extract exposure, gain, and temperature from camera metrics."""
        exposure = 0.0
        gain = 0.0
        temperature = 0.0
        try:
            metrics = self.camera.get_camera_metrics()
            exposure = metrics.exposure_value
            gain = metrics.gain_value
        except Exception:
            pass
        try:
            temp_data = self.sensors_data.get_temperature_data()
            temperature = temp_data.get(sl.SENSOR_LOCATION.ONBOARD_LEFT)
        except Exception:
            pass
        return exposure, gain, temperature

    def close(self):
        self.stop_recording()
        self.camera.close()
