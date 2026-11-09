"""Standalone radar + camera live demo — no dashboard, no dataset writer,
no IMU/LiDAR/GNSS, no synchronization gating. Opens the ZED camera and the
AWR1843 radar independently and draws every radar detection directly onto
the ZED left image, projected with the camera's own intrinsics (fx, fy, cx,
cy from camera.calib), colored by Doppler velocity — same convention as
dashboard/radar_panel.py.

Intended for material/transparency sensing demos: a transparent or acrylic
object (e.g. the cup demo) the camera struggles to see/segment cleanly can
still return a solid radar detection, drawn right where it physically is in
the image.

Projection math: the AWR1843's detected-object TLV reports Cartesian points
as x=lateral (+right), y=forward/range, z=up (see radar/parser.py's TLV
docstring; also how dashboard/radar_panel.py's heatmap point overlay plots
them). That is a standard camera-relative frame once relabeled for the
pinhole model (camera x=right, y=down, z=forward):
    Xc, Yc, Zc = p.x, -p.z, p.y
    u = cx + fx * Xc / Zc
    v = cy + fy * Yc / Zc
This only needs the camera's own intrinsics, not the ZED SDK's configured
world coordinate system (config.settings.COORDINATE_SYSTEM) — that setting
governs the ZED's own point-cloud/pose outputs, not this projection.

Applies config.settings.RADAR_TO_CAMERA_OFFSET_M (radar->camera lever arm,
radar frame) before the Xc/Yc/Zc relabeling above — see radar/projection.py.
It defaults to (0, 0, 0) (co-located approximation) until measured; see
docs/RADAR_CAMERA_OVERLAY.md for how to measure and set it.

Usage:
    python radar_camera_demo.py                # camera + radar overlay
    python radar_camera_demo.py --no-radar      # camera only
    python radar_camera_demo.py --no-camera     # radar-only bird's-eye view
    python radar_camera_demo.py --webcam        # force a plain USB webcam
                                                 # instead of the ZED (also
                                                 # used automatically if the
                                                 # ZED SDK fails to open one)

Webcam fallback: a plain UVC webcam has no calibrated intrinsics, so fx/fy
are approximated from --webcam-fov-deg (a guessed horizontal FOV) and cx/cy
are just the image center. Radar points will land in roughly the right place
but not pixel-accurate — for real accuracy, measure the webcam's intrinsics
(e.g. OpenCV chessboard calibration) and pass them via --webcam-fx/--webcam-fy.

Press 'q' in the window (or Ctrl+C in the terminal) to quit.
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

_MAX_VEL_MPS = 5.0

# Bird's-eye fallback canvas (radar-only mode, when there's no camera image
# to overlay onto) — matches dashboard/radar_panel.py's _MAX_X/_MAX_Y.
_MAX_X = 12.0
_MAX_Y = 10.0
_CANVAS_W = 640
_CANVAS_H = 480


def _color_by_velocity(vel_mps: float) -> tuple[int, int, int]:
    """Approaching -> blue/cyan, receding -> orange/red, static -> green.
    Same mapping as dashboard/radar_panel.py's _color_by_velocity, just
    expressed as OpenCV BGR instead of pyqtgraph RGBA."""
    t = float(np.clip(abs(vel_mps) / _MAX_VEL_MPS, 0.0, 1.0))
    if vel_mps < 0:
        b, g, r = 200 + 55 * t, 64 + 120 * t, 32
    elif vel_mps > 0:
        b, g, r = 40, 60, 150 + 105 * t
    else:
        b, g, r = 60, 185, 60
    return (int(min(b, 255)), int(min(g, 255)), int(min(r, 255)))


def _overlay_points(image: np.ndarray, points, calib: dict) -> np.ndarray:
    from config.settings import RADAR_TO_CAMERA_OFFSET_M
    from radar.projection import project_point

    h, w = image.shape[:2]
    fx, fy, cx, cy = calib["fx"], calib["fy"], calib["cx"], calib["cy"]
    for p in points:
        px = project_point(p, fx, fy, cx, cy, RADAR_TO_CAMERA_OFFSET_M)
        if px is None:
            continue
        u, v = px
        if not (0 <= u < w and 0 <= v < h):
            continue
        color = _color_by_velocity(p.velocity_mps)
        cv2.circle(image, (u, v), 8, color, 2)
        cv2.circle(image, (u, v), 2, color, -1)
        cv2.putText(image, f"{p.y:.1f}m", (u + 10, v - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    cv2.putText(image, f"radar points: {len(points)}", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def _to_px(x: float, y: float) -> tuple[int, int]:
    px = int((x + _MAX_X) / (2 * _MAX_X) * _CANVAS_W)
    py = int(_CANVAS_H - (y / _MAX_Y) * _CANVAS_H)
    return px, py


def _draw_radar_canvas(points) -> np.ndarray:
    """Bird's-eye fallback used only when there's no camera image (radar-only
    mode) to overlay onto."""
    canvas = np.full((_CANVAS_H, _CANVAS_W, 3), 24, dtype=np.uint8)

    for gx in range(-int(_MAX_X), int(_MAX_X) + 1, 2):
        cv2.line(canvas, _to_px(gx, 0), _to_px(gx, _MAX_Y), (50, 50, 50), 1)
    for gy in range(0, int(_MAX_Y) + 1, 2):
        cv2.line(canvas, _to_px(-_MAX_X, gy), _to_px(_MAX_X, gy), (50, 50, 50), 1)

    cv2.drawMarker(canvas, _to_px(0, 0), (0, 200, 255),
                    cv2.MARKER_TRIANGLE_UP, 12, 2)

    for p in points:
        if not (-_MAX_X <= p.x <= _MAX_X and 0 <= p.y <= _MAX_Y):
            continue
        cv2.circle(canvas, _to_px(p.x, p.y), 4, _color_by_velocity(p.velocity_mps), -1)

    cv2.putText(canvas, f"radar points: {len(points)}", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return canvas


class _WebcamFrame:
    __slots__ = ("left",)

    def __init__(self, left: np.ndarray):
        self.left = left


class WebcamCamera:
    """Fallback camera source for a plain UVC webcam (no ZED SDK / hardware
    needed). Exposes the same open()/grab()/close()/.calib surface that
    _overlay_points() and main()'s loop expect from ZEDCamera, so the rest
    of the script doesn't need to know which one it's talking to.

    A generic webcam reports no calibrated intrinsics, so fx/fy are derived
    from a guessed horizontal FOV (--webcam-fov-deg) unless overridden via
    --webcam-fx/--webcam-fy, and cx/cy default to the image center. This
    makes the radar overlay approximate, not pixel-accurate.
    """

    def __init__(self, index: int, fov_deg: float,
                 fx: float | None = None, fy: float | None = None):
        self.index = index
        self.fov_deg = fov_deg
        self._fx_override = fx
        self._fy_override = fy
        self.cap = None
        self.calib = None

    def open(self):
        self.cap = cv2.VideoCapture(self.index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open webcam at index {self.index}")

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cx, cy = w / 2.0, h / 2.0
        fx = self._fx_override if self._fx_override is not None else \
            (w / 2.0) / np.tan(np.radians(self.fov_deg) / 2.0)
        fy = self._fy_override if self._fy_override is not None else fx

        self.calib = {
            "model": "webcam (uncalibrated)",
            "fx": fx, "fy": fy, "cx": cx, "cy": cy,
            "width": w, "height": h,
        }

    def grab(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        return _WebcamFrame(left=frame)

    def close(self):
        if self.cap is not None:
            self.cap.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-camera", action="store_true", help="Radar only (bird's-eye view)")
    parser.add_argument("--no-radar", action="store_true", help="Camera only")
    parser.add_argument("--webcam", action="store_true",
                         help="Force a plain USB webcam instead of the ZED")
    parser.add_argument("--webcam-index", type=int, default=0,
                         help="cv2.VideoCapture index for the webcam (default 0)")
    parser.add_argument("--webcam-fov-deg", type=float, default=70.0,
                         help="Guessed horizontal FOV (deg) used to approximate "
                              "fx/fy if --webcam-fx/--webcam-fy aren't given")
    parser.add_argument("--webcam-fx", type=float, default=None,
                         help="Known/calibrated fx (px) for the webcam, if available")
    parser.add_argument("--webcam-fy", type=float, default=None,
                         help="Known/calibrated fy (px) for the webcam, if available")
    args = parser.parse_args()

    if args.no_camera and args.no_radar:
        parser.error("--no-camera and --no-radar together leaves nothing to show")

    camera = None
    radar = None

    if not args.no_camera:
        camera = WebcamCamera(args.webcam_index, args.webcam_fov_deg,
                               args.webcam_fx, args.webcam_fy) if args.webcam else None
        if camera is None:
            try:
                from camera.zed_camera import ZEDCamera
                camera = ZEDCamera()
                camera.open()
                print(f"ZED camera opened. fx={camera.calib['fx']:.1f} "
                      f"fy={camera.calib['fy']:.1f} cx={camera.calib['cx']:.1f} "
                      f"cy={camera.calib['cy']:.1f}")
            except Exception as exc:
                print(f"ZED camera unavailable ({exc}); falling back to webcam "
                      f"index {args.webcam_index}.")
                camera = WebcamCamera(args.webcam_index, args.webcam_fov_deg,
                                       args.webcam_fx, args.webcam_fy)
        if isinstance(camera, WebcamCamera):
            camera.open()
            print(f"Webcam opened (index {camera.index}, uncalibrated). "
                  f"fx={camera.calib['fx']:.1f} fy={camera.calib['fy']:.1f} "
                  f"cx={camera.calib['cx']:.1f} cy={camera.calib['cy']:.1f} "
                  f"— radar overlay will be approximate.")

    if not args.no_radar:
        from radar.receiver import RadarReceiver
        radar = RadarReceiver()
        radar.start()
        print("Radar started.")

    window = "Camera + Radar overlay (standalone demo) — press q to quit"
    try:
        while True:
            left_img = None
            if camera is not None:
                frame = camera.grab()
                if frame is not None:
                    left_img = frame.left

            points = []
            if radar is not None:
                rf = radar.get_frame()
                if rf is not None:
                    points = rf.points

            if left_img is not None:
                display = _overlay_points(left_img.copy(), points, camera.calib) \
                    if radar is not None else left_img
            else:
                display = _draw_radar_canvas(points)

            cv2.imshow(window, display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.close()
        if radar is not None:
            radar.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
