"""High-quality 3D LiDAR visualization using Ouster SDK viz library.

This provides Studio-quality point cloud rendering in the GCS dashboard.
"""

from __future__ import annotations

import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout
from PySide6.QtCore import Qt, QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtGui import QSurfaceFormat

try:
    from ouster.sdk.viz import PointViz, Cloud, CloudMode
    _HAS_OUSTER_VIZ = True
except ImportError:
    _HAS_OUSTER_VIZ = False


class OusterVisualizerWidget(QOpenGLWidget):
    """OpenGL widget using Ouster's PointViz for high-quality 3D rendering."""

    def __init__(self, parent=None):
        super().__init__(parent)

        if not _HAS_OUSTER_VIZ:
            self._viz = None
            self._cloud = None
            return

        # Setup OpenGL format
        fmt = QSurfaceFormat()
        fmt.setVersion(3, 2)
        fmt.setProfile(QSurfaceFormat.CoreProfile)
        self.setFormat(fmt)

        self._viz = None
        self._cloud = None
        self._points = None
        self._colors = None
        self._mode = CloudMode.REFLECTIVITY

    def initializeGL(self):
        """Initialize OpenGL context and Ouster visualizer."""
        if not _HAS_OUSTER_VIZ:
            return

        try:
            self._viz = PointViz("LiDAR Point Cloud")
            self._viz.camera.set_near_plane(0.1)
            self._viz.camera.set_far_plane(500.0)
        except Exception as e:
            print(f"Failed to init PointViz: {e}")
            self._viz = None

    def resizeGL(self, w: int, h: int):
        """Handle window resize."""
        pass

    def paintGL(self):
        """Render the point cloud."""
        if self._viz is None:
            return

        try:
            # Update cloud if we have new data
            if self._points is not None and self._cloud is None:
                self._create_cloud()

            # Render
            self._viz.draw()
        except Exception as e:
            pass  # Fail silently to avoid spamming console

    def set_cloud(self, points: np.ndarray | None, colors: np.ndarray | None = None):
        """Update the point cloud.

        Args:
            points: Nx3 array of (x, y, z) coordinates in meters
            colors: Optional Nx3 array of (r, g, b) colors or N array of intensities
        """
        if not _HAS_OUSTER_VIZ or self._viz is None:
            return

        if points is None or points.size == 0:
            self._points = None
            self._cloud = None
            self._colors = None
            return

        self._points = points.astype(np.float32)
        self._colors = colors
        self._cloud = None  # Force recreation on next paint
        self.update()

    def _create_cloud(self):
        """Create Cloud object from current points and colors."""
        if self._points is None or self._viz is None:
            return

        try:
            # Create cloud with reflectivity-based coloring
            # Colors should be intensity/reflectivity values (0-255 or 0-1)
            if self._colors is not None:
                colors = self._colors
                if colors.ndim == 1:
                    # Single channel intensity - replicate to RGB
                    if colors.max() > 1.0:
                        colors = colors / 255.0
                    colors = np.stack([colors, colors, colors], axis=1)
                colors = np.clip(colors, 0, 1).astype(np.float32)
            else:
                # Default: color by Z (height)
                z = self._points[:, 2]
                z_norm = (z - z.min()) / (z.max() - z.min() + 1e-6)
                colors = np.stack([z_norm, z_norm, z_norm], axis=1)

            self._cloud = Cloud(self._points, colors, self._mode)
            if self._viz.get_cloud():
                self._viz.remove(self._viz.get_cloud())
            self._viz.add(self._cloud)

            # Auto-fit camera to cloud
            if len(self._points) > 0:
                center = self._points.mean(axis=0)
                radius = np.linalg.norm(self._points - center, axis=1).max()
                self._viz.camera.set_target(center.tolist())
                self._viz.camera.set_distance(radius * 2.5)
        except Exception as e:
            print(f"Failed to create cloud: {e}")


class Lidar3DPanel(QWidget):
    """3D LiDAR visualization panel for GCS using Ouster's rendering."""

    def __init__(self, parent=None):
        super().__init__(parent)

        if not _HAS_OUSTER_VIZ:
            # Fallback if Ouster viz not available
            print("WARNING: Ouster SDK viz not available, 3D visualization disabled")
            return

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._viewer = OusterVisualizerWidget()
        layout.addWidget(self._viewer)

        # Render timer for continuous updates
        self._timer = QTimer()
        self._timer.timeout.connect(self._viewer.update)
        self._timer.start(33)  # ~30 FPS

    def update_lidar(self, points: np.ndarray | None, reflectivity: np.ndarray | None = None, **kwargs):
        """Update LiDAR visualization with new point cloud.

        Args:
            points: Nx3 array of (x, y, z) coordinates
            reflectivity: Optional N array of reflectivity values for coloring
            **kwargs: Additional parameters (ignored, for API compatibility)
        """
        if self._viewer:
            self._viewer.set_cloud(points, reflectivity)
