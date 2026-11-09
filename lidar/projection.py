"""Lidar -> camera-image projection (shared by the dashboard overlay).

Uses the vehicle-frame extrinsics in fusion/transforms.py to move Ouster
sensor-frame points into the ZED left-eye optical frame (x right, y down,
z forward), then the pinhole model with the ZED intrinsics.
"""
from __future__ import annotations

import numpy as np


def project_lidar_to_camera(points, extrinsics, fx, fy, cx, cy, width, height,
                            min_depth: float = 0.5):
    """Return (u, v, depth) int/int/float arrays for points that land inside
    the image in front of the camera.  points is (N, 3) in the lidar frame."""
    empty = (np.empty(0, int), np.empty(0, int), np.empty(0, float))
    if points is None or len(points) == 0:
        return empty
    pc = extrinsics.transform(np.asarray(points, np.float32), "lidar", "camera")
    z = pc[:, 2]
    ok = z > min_depth
    pc, z = pc[ok], z[ok]
    u = np.rint(cx + fx * pc[:, 0] / z).astype(int)
    v = np.rint(cy + fy * pc[:, 1] / z).astype(int)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    return u[inside], v[inside], z[inside]


def draw_depth_dots(img, u, v, depth, max_depth: float = 25.0, size: int = 2):
    """Paint projected points onto an RGB image, red = near, blue = far.
    size=1 is a single pixel, 2 a 3x3 square, 3 a 5x5 square."""
    import cv2
    if len(u) == 0:
        return img
    t = np.clip(depth / max_depth, 0.0, 1.0)
    idx = ((1.0 - t) * 255).astype(np.uint8).reshape(-1, 1)
    rgb = cv2.applyColorMap(idx, cv2.COLORMAP_JET)[:, 0, ::-1]   # BGR->RGB
    # far points first so near points win overlaps
    order = np.argsort(-depth)
    u, v, rgb = u[order], v[order], rgb[order]
    h, w = img.shape[:2]
    for du in range(-size + 1, size):
        for dv in range(-size + 1, size):
            uu = np.clip(u + du, 0, w - 1)
            vv = np.clip(v + dv, 0, h - 1)
            img[vv, uu] = rgb
    return img
