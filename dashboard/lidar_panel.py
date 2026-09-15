"""LidarPanel — Ouster OS-0 point cloud, top-down bird's-eye view.

Projects the XYZ cloud onto the X-Y (ground) plane and colours each point by
height (Z).  Rendered once per frame into a uint8 RGB image shown in a
QLabel, matching the CameraImage render path.
"""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel, QSlider, QSizePolicy, QVBoxLayout, QWidget

from dashboard.theme import COLORS

try:
    from PySide6.QtOpenGLWidgets import QOpenGLWidget
    from PySide6.QtGui import QOpenGLFunctions
    from OpenGL import GL
    _HAS_GL = True
except ImportError:
    _HAS_GL = False


def _render_cloud(points: np.ndarray, h: int = 720, w: int = 720,
                  half: float = 20.0, z_min: float = -2.0, z_max: float = 3.0,
                  max_points: int = 30000) -> np.ndarray:
    """Project points to a top-down Range Image (height-coloured, Ouster Studio style)."""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    if points is None or points.size == 0:
        return img
    pts = points.reshape(-1, 3)
    if pts.shape[0] > max_points:
        step = pts.shape[0] // max_points
        pts = pts[::step]
    # Filter invalid points
    good = np.isfinite(pts).all(axis=1) & (np.abs(pts).sum(axis=1) > 0.1)
    pts = pts[good]
    if pts.shape[0] == 0:
        return img
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    # Auto-zoom: fit data bounds with padding
    x_range = np.max(x) - np.min(x) if x.size > 0 else 1
    y_range = np.max(y) - np.min(y) if y.size > 0 else 1
    data_half = max(x_range, y_range) / 2 + 2  # Add 2m padding
    half = max(data_half, 5)  # Minimum 5m view

    # Clamp to view bounds
    in_view = (np.abs(x) <= half) & (np.abs(y) <= half)
    x, y, z = x[in_view], y[in_view], z[in_view]
    if x.shape[0] == 0:
        return img

    # Screen mapping: X→horizontal, Y→vertical
    sx = ((x + half) / (2 * half) * (w - 1)).astype(int)
    sy = ((half - y) / (2 * half) * (h - 1)).astype(int)
    sx = np.clip(sx, 0, w - 1)
    sy = np.clip(sy, 0, h - 1)

    # Color by height (Z) using Viridis-like colormap
    z_norm = np.clip((z - z_min) / (z_max - z_min), 0, 1)

    # Viridis colormap approximation (purple→blue→green→yellow)
    r = np.where(z_norm < 0.5,
                 np.clip(1.5 - np.abs(4 * z_norm - 1), 0, 1),
                 np.clip(4 * z_norm - 2.5, 0, 1))
    g = np.clip(1.5 - np.abs(4 * z_norm - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * z_norm - 3), 0, 1)

    col = (np.stack([r, g, b], axis=-1) * 255).astype(np.uint8)
    img[sy, sx] = col

    return img


class _LidarGL(QOpenGLWidget if _HAS_GL else object):
    """3D point cloud rendered via OpenGL.  Left-drag = rotate, right-drag = pan, scroll = zoom.

    Studio-quality rendering matching Ouster's visualization style.
    """

    def __init__(self, parent=None):
        if not _HAS_GL:
            return
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._positions = None
        self._colors = None
        self._n = 0
        self._vbo = None
        self._vao = None
        self._shader = None
        self._cx = self._cy = self._cz = 0.0
        # Camera: view from the SIDE (perpendicular to forward axis)
        # This shows X-Z spread (left-right and up-down) with Y depth visible
        self._rot_x = -25.0  # Slight upward tilt to see height separation
        self._rot_y = 180.0  # View from behind, looking forward
        self._dist = 40.0
        self._pan_x = self._pan_y = self._pan_z = 0.0
        self._last = None
        self.setMouseTracking(True)

        # For smooth rendering
        self._frame_count = 0
        self._last_time = 0

    # ── OpenGL lifecycle ────────────────────────────────────────────

    def initializeGL(self):
        GL.glClearColor(0.05, 0.05, 0.07, 1.0)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)

        vs = GL.glCreateShader(GL.GL_VERTEX_SHADER)
        GL.glShaderSource(vs, """#version 120
        attribute vec3 aPos;
        attribute vec3 aCol;
        varying vec3 vCol;
        uniform mat4 uMVP;
        void main() {
            vCol = aCol;
            vec4 pos = uMVP * vec4(aPos, 1.0);
            gl_Position = pos;
            // Point size decreases with depth for proper perspective
            float depth = length(pos.xyz) / pos.w;
            gl_PointSize = max(2.0, 8.0 / (1.0 + depth * 0.1));
        }""")
        GL.glCompileShader(vs)
        if not GL.glGetShaderiv(vs, GL.GL_COMPILE_STATUS):
            print("VS:", GL.glGetShaderInfoLog(vs).decode())

        fs = GL.glCreateShader(GL.GL_FRAGMENT_SHADER)
        GL.glShaderSource(fs, """#version 120
        varying vec3 vCol;
        void main() { gl_FragColor = vec4(vCol, 1.0); }""")
        GL.glCompileShader(fs)
        if not GL.glGetShaderiv(fs, GL.GL_COMPILE_STATUS):
            print("FS:", GL.glGetShaderInfoLog(fs).decode())

        self._shader = GL.glCreateProgram()
        GL.glAttachShader(self._shader, vs)
        GL.glAttachShader(self._shader, fs)
        GL.glLinkProgram(self._shader)
        if not GL.glGetProgramiv(self._shader, GL.GL_LINK_STATUS):
            print("LINK:", GL.glGetProgramInfoLog(self._shader).decode())
        GL.glDeleteShader(vs)
        GL.glDeleteShader(fs)

        self._a_pos = GL.glGetAttribLocation(self._shader, "aPos")
        self._a_col = GL.glGetAttribLocation(self._shader, "aCol")
        self._u_mvp = GL.glGetUniformLocation(self._shader, "uMVP")

        self._vbo = GL.glGenBuffers(1)

    def resizeGL(self, w, h):
        GL.glViewport(0, 0, w, h)

    def paintGL(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LEQUAL)

        if self._n == 0 or self._shader is None or self._vbo is None:
            return

        w, h = self.width(), self.height()
        proj = self._perspective(45.0, w / max(h, 1), 0.1, 500.0)
        view = self._look_at()

        GL.glUseProgram(self._shader)
        GL.glUniformMatrix4fv(self._u_mvp, 1, GL.GL_FALSE, proj @ view)

        # Enable point smoothing and depth testing for better appearance
        GL.glEnable(GL.GL_POINT_SMOOTH)
        GL.glHint(GL.GL_POINT_SMOOTH_HINT, GL.GL_NICEST)
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

        GL.glEnableVertexAttribArray(self._a_pos)
        GL.glEnableVertexAttribArray(self._a_col)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._vbo)
        GL.glVertexAttribPointer(self._a_pos, 3, GL.GL_FLOAT, GL.GL_FALSE, 24, GL.GLvoidp(0))
        GL.glVertexAttribPointer(self._a_col, 3, GL.GL_FLOAT, GL.GL_FALSE, 24, GL.GLvoidp(12))
        GL.glDrawArrays(GL.GL_POINTS, 0, self._n)
        GL.glDisableVertexAttribArray(self._a_pos)
        GL.glDisableVertexAttribArray(self._a_col)

        GL.glDisable(GL.GL_BLEND)
        GL.glDisable(GL.GL_POINT_SMOOTH)

    # ── mouse interaction ───────────────────────────────────────────

    def mousePressEvent(self, e):
        self._last = e.position()

    def mouseMoveEvent(self, e):
        if self._last is None:
            return
        dx = e.position().x() - self._last.x()
        dy = e.position().y() - self._last.y()
        self._last = e.position()
        if e.buttons() & Qt.MouseButton.LeftButton:
            self._rot_y += dx * 0.4
            self._rot_x = max(-89.0, min(89.0, self._rot_x + dy * 0.4))
            self.update()
        elif e.buttons() & Qt.MouseButton.RightButton:
            s = self._dist * 0.002
            self._pan_x -= dx * s
            self._pan_y += dy * s
            self.update()

    def wheelEvent(self, e):
        d = e.angleDelta().y()
        if d != 0:
            self._dist *= 0.9 if d > 0 else 1.1
            self._dist = max(2.0, min(200.0, self._dist))
            self.update()

    # ── data ────────────────────────────────────────────────────────

    def set_cloud(self, points, colors=None):
        if points is None or points.size == 0:
            self._n = 0
            self.update()
            return
        pts = points.reshape(-1, 3).astype(np.float32)
        # Filter out invalid points: non-finite or too close to origin (invalid measurements)
        valid = np.isfinite(pts).all(axis=1) & (np.abs(pts).sum(axis=1) > 0.1)
        pts = pts[valid]
        # Robust center from the median so a few far outliers don't shift the
        # framing (or drag the camera back) and make the 64-channel cloud look
        # sparse.  Requiring finite points only (no left-channel info here).
        self._positions = pts
        self._n = pts.shape[0]
        if self._n == 0:
            self._colors = None
            self._upload()
            self.update()
            return
        med = np.median(pts, axis=0)
        self._cx, self._cy, self._cz = float(med[0]), float(med[1]), float(med[2])
        r = np.linalg.norm(pts - med, axis=1)
        target = float(np.percentile(r, 90))
        # Auto-fit distance: larger multiplier for narrow clouds, smaller for wide ones
        self._dist = max(target * 2.5, 15.0)
        self._pan_x = self._pan_y = self._pan_z = 0.0
        # Colour by height (z): the OS-0-64's beam elevations put each channel
        # at a distinct z, so height-colouring visually separates all 64
        # channels as colour bands instead of one undifferentiated blob.
        z = pts[:, 2]
        lo, hi = float(np.nanpercentile(z, 2)), float(np.nanpercentile(z, 98))
        if hi - lo < 1e-3:
            hi = lo + 1.0
        t = np.clip((z - lo) / (hi - lo), 0, 1)
        r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
        g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
        b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
        self._colors = np.stack([r, g, b], axis=-1).astype(np.float32)
        self._upload()
        self.update()

    def _upload(self):
        if self._vbo is None:
            return
        data = np.hstack([self._positions, self._colors]).astype(np.float32)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_DYNAMIC_DRAW)

    # ── matrix helpers ──────────────────────────────────────────────

    @staticmethod
    def _perspective(fov, aspect, near, far):
        f = 1.0 / np.tan(np.radians(fov) / 2)
        m = np.zeros((4, 4), dtype=np.float32)
        m[0, 0] = f / aspect
        m[1, 1] = f
        m[2, 2] = (far + near) / (near - far)
        m[2, 3] = (2 * far * near) / (near - far)
        m[3, 2] = -1
        return m

    def _look_at(self):
        rx, ry = np.radians(self._rot_x), np.radians(self._rot_y)
        cy, sy = np.cos(ry), np.sin(ry)
        cx, sx = np.cos(rx), np.sin(rx)
        ex = self._cx + self._pan_x + self._dist * sy * cx
        ey = self._cy + self._pan_y - self._dist * sx
        ez = self._cz + self._pan_z + self._dist * cy * cx
        fwd = np.array([self._cx - ex, self._cy - ey, self._cz - ez], dtype=np.float64)
        fwd /= np.linalg.norm(fwd)
        up = np.array([0.0, 1.0, 0.0])
        right = np.cross(fwd, up)
        n = np.linalg.norm(right)
        if n < 1e-6:
            up = np.array([0.0, 0.0, -1.0])
            right = np.cross(fwd, up)
            n = np.linalg.norm(right)
        right /= n
        up = np.cross(right, fwd)
        m = np.eye(4, dtype=np.float32)
        m[0, :3] = right
        m[1, :3] = up
        m[2, :3] = -fwd
        m[0, 3] = -np.dot(right, [ex, ey, ez])
        m[1, 3] = -np.dot(up, [ex, ey, ez])
        m[2, 3] = np.dot(fwd, [ex, ey, ez])
        return m


class LidarPanel(QWidget):
    """3D orientable lidar point-cloud view with zoom slider."""

    def __init__(self, parent=None):
        super().__init__(parent)
        vl = QVBoxLayout(self)
        vl.setContentsMargins(0, 0, 0, 0)
        vl.setSpacing(2)

        self.title = QLabel("Lidar (OS-0-64) — 3D View")
        self.title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title.setStyleSheet(
            f"color: {COLORS['green']}; font-weight: bold; font-size: 12px;"
        )

        if _HAS_GL:
            self._gl = _LidarGL()
            print(f"[LidarPanel] OpenGL available (using 2D Range Image)")
        else:
            self._gl = None
            print(f"[LidarPanel] OpenGL NOT available, using 2D fallback")

        # Always create fallback image for 2D Range Image display
        self._fallback_img = QLabel("No Signal")
        self._fallback_img.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._fallback_img.setMinimumSize(280, 200)
        self._fallback_img.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self._fallback_img.setStyleSheet(
            f"background-color: {COLORS['bg_dark']};"
            f"border: 1px solid {COLORS['border']};"
        )
        self._pixmap = QPixmap()

        self.metrics = QLabel("—")
        self.metrics.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.metrics.setStyleSheet(
            f"color: {COLORS['text_dim']}; font-size: 11px;"
        )

        self._zoom_slider = QSlider(Qt.Orientation.Horizontal)
        self._zoom_slider.setRange(5, 50)
        self._zoom_slider.setValue(20)
        self._zoom_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._zoom_slider.setTickInterval(5)
        self._zoom_slider.setStyleSheet(
            f"QSlider::groove:horizontal {{ height: 4px; background: {COLORS['border']}; }}"
            f"QSlider::handle:horizontal {{ width: 14px; margin: -5px 0;"
            f"  background: {COLORS['green']}; border-radius: 7px; }}"
        )

        vl.addWidget(self.title)
        # Always display 2D Range Image (clearer than 3D for single-frame LiDAR)
        vl.addWidget(self._fallback_img, stretch=1)
        vl.addWidget(self.metrics)
        vl.addWidget(self._zoom_slider)

    def update_data(self, points, num_points: int, rate: float,
                    streaming: bool, frame_id: int, matched: bool = False,
                    sync_delta_ms: float | None = None):
        if not streaming or points is None or points.size == 0:
            if self._gl:
                self._gl.set_cloud(None)
            else:
                self._fallback_img.setText("No Signal")
            self.metrics.setText("offline")
            self.title.setText("Lidar (OS-0-64) — Range Image")
            return

        # Always use 2D range image view (clearer for single-frame visualization)
        rgb = _render_cloud(points)
        qimg = QImage(rgb.data, rgb.shape[1], rgb.shape[0],
                      rgb.shape[1] * 3, QImage.Format.Format_RGB888)
        self._pixmap = QPixmap.fromImage(qimg)
        scaled = self._pixmap.scaled(
            720, 720,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        # Display as image
        self._fallback_img.setPixmap(scaled)

        self.metrics.setText(
            f"{num_points} pts · {rate:.1f} Hz · frame {frame_id} · "
            + (f"matched {sync_delta_ms:.1f} ms" if matched else "live only")
        )
        self.title.setText("Lidar (OS-0-64) — 3D View")
