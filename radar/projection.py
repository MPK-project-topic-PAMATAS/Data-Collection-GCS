"""Shared radar->camera-pixel projection, used by radar_camera_demo.py and
dashboard/radar_panel.py's RadarCameraOverlay so the math lives in one place.

The AWR1843's detected-object TLV reports Cartesian points as
x=lateral(+right), y=forward/range, z=up (radar/parser.py's TLV docstring).
Camera intrinsics (fx, fy, cx, cy from camera.calib) are for the ZED's LEFT
eye. A lever arm (config.settings.RADAR_TO_CAMERA_OFFSET_M) translates a
point from the radar's origin to the camera's left-eye origin, both
expressed in the radar's own frame, before relabeling into the pinhole
model (camera x=right, y=down, z=forward):
    Xc, Yc, Zc = (p.x - dx), -(p.z - dz), (p.y - dy)
    u = cx + fx * Xc / Zc
    v = cy + fy * Yc / Zc

RADAR_TO_CAMERA_OFFSET_M defaults to (0, 0, 0) — co-located approximation —
until someone measures the actual mounting gap (see the constant's comment
in config/settings.py and docs/RADAR_CAMERA_OVERLAY.md for how). A zero
offset reproduces the prior co-location behavior exactly.
"""
from __future__ import annotations


def project_point(p, fx: float, fy: float, cx: float, cy: float,
                   offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)):
    """Radar point -> image pixel (u, v), or None if behind the camera.

    offset_m is (dx, dy, dz): the vector from the radar's origin to the
    camera's left-eye origin, in the radar's frame (x=right, y=forward,
    z=up), meters — i.e. config.settings.RADAR_TO_CAMERA_OFFSET_M.
    """
    dx, dy, dz = offset_m
    xc = p.x - dx
    yc = p.y - dy
    zc = p.z - dz
    if yc <= 0:
        return None
    u = cx + fx * xc / yc
    v = cy + fy * (-zc) / yc
    return int(round(u)), int(round(v))
