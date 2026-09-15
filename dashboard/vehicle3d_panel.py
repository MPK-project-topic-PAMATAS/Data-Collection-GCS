"""3D Vehicle Orientation Panel — car body + ZED axes + MTi axes.

Falls back to text display when OpenGL is unavailable.
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QGroupBox, QLabel, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS

try:
    import pyqtgraph.opengl as gl
    HAS_GL = True
except Exception:
    HAS_GL = False


def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    return Rotation.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()


# ---------------------------------------------------------------------------
# Car geometry
#
# The body is built from a side-profile silhouette (nose -> hood -> windshield
# -> roof -> rear window -> trunk -> tail) mirrored to both sides of the car
# and stitched together with a floor loop, cross-ties, wheel arches, and round
# wheels. This reads as an actual vehicle from any viewing angle instead of a
# flat box.
# ---------------------------------------------------------------------------

LNG = 1.30          # half length, bumper to bumper
WID = 0.62           # half width at body/rocker
WID_ROOF = 0.50       # half width at roof (cabin tumblehome)
H_FLOOR = -0.28       # rocker / floor height
H_HOOD = 0.16         # hood & decklid height
H_ROOF = 0.55         # roof height

WHEEL_R = 0.22
WHEEL_X_FRONT = 1.02
WHEEL_X_REAR = -1.02
WHEEL_Y_OFFSET = 0.03  # wheels poke slightly proud of the body for visibility

# Side-profile silhouette points as (x, z, half_width). Traced once per side
# (mirrored by sign) to build the hood/windshield/roof/rear-window/trunk line.
_PROFILE = [
    (LNG, -0.05, WID),          # front bumper, bottom
    (LNG, 0.08, WID),           # front bumper, top
    (0.72, H_HOOD, WID),        # cowl / base of windshield
    (0.32, H_ROOF, WID_ROOF),   # windshield header / roof front
    (-0.42, H_ROOF, WID_ROOF),  # roof rear
    (-0.80, H_HOOD, WID),       # base of rear window / decklid
    (-LNG, 0.08, WID),          # rear bumper, top
    (-LNG, -0.05, WID),         # rear bumper, bottom
]


def _profile_side(sign):
    """Open polyline tracing the car's silhouette on one side (-1 left / +1 right)."""
    return np.array([[x, sign * w, z] for (x, z, w) in _PROFILE])


def _floor_loop():
    """Closed rectangle tracing the rocker/floor pan outline."""
    return np.array([
        [LNG, -WID, H_FLOOR], [LNG, WID, H_FLOOR],
        [-LNG, WID, H_FLOOR], [-LNG, -WID, H_FLOOR], [LNG, -WID, H_FLOOR],
    ])


_TIE_STATIONS = (1, 3, 4, 6)  # bumper tops, windshield header, roof rear


def _cross_ties():
    """Disconnected left/right tie segments at a few profile stations (mode='lines').

    Only bumpers + roof are braced side-to-side; bracing every station (cowl,
    decklid, etc.) made the front/rear corners look like a tangle of lines.
    """
    left = _profile_side(-1)
    right = _profile_side(1)
    segs = []
    for i in _TIE_STATIONS:
        segs.append(left[i])
        segs.append(right[i])
    return np.array(segs)


def _wheel_positions():
    return [
        (WHEEL_X_FRONT, -1), (WHEEL_X_FRONT, 1),
        (WHEEL_X_REAR, -1), (WHEEL_X_REAR, 1),
    ]


def _wheel_circle(cx, sign, r=WHEEL_R, n=20):
    """A round wheel/tire drawn edge-on (vertical circle in the x-z plane)."""
    y = sign * (WID + WHEEL_Y_OFFSET)
    ang = np.linspace(0, 2 * math.pi, n)
    return np.array([[cx + r * math.cos(a), y, H_FLOOR + r * math.sin(a)] for a in ang])


def _axle_lines():
    """Front and rear axle ties, as two disconnected segments (mode='lines')."""
    y = WID + WHEEL_Y_OFFSET
    return np.array([
        [WHEEL_X_FRONT, -y, H_FLOOR], [WHEEL_X_FRONT, y, H_FLOOR],
        [WHEEL_X_REAR, -y, H_FLOOR], [WHEEL_X_REAR, y, H_FLOOR],
    ])


def _make_axis_lines(R, origin, length=1.0, width=2):
    """Create 3 axis line segments from rotation matrix + origin."""
    x_end = origin + R @ np.array([length, 0, 0])
    y_end = origin + R @ np.array([0, length, 0])
    z_end = origin + R @ np.array([0, 0, length])
    return (
        np.array([origin, x_end]),
        np.array([origin, y_end]),
        np.array([origin, z_end]),
    )


# Sensor local positions on the car body
_CAM_LOCAL = np.array([-1.0, 0, H_ROOF - 0.15])
_MTI_LOCAL = np.array([0.0, 0, 0.0])

# Gravity is drawn beside the car (not through its center) so it never
# overlaps the MTi Z axis, which also passes through the origin.
_GRAVITY_XY = np.array([0.0, -1.35])
_GRAVITY_LEN = 1.2

_LABEL_FONT = QFont("Arial", 10, QFont.Bold)


def _hex_to_rgba(hex_color: str, alpha: int = 255):
    """Convert a hex `#RRGGBB` color to an (r,g,b,a) tuple expected by GL items."""
    h = hex_color.lstrip("#")
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return (r, g, b, alpha)


def _make_car_mesh():
    """Return (verts, faces) for a simple low-poly car body mesh.

    The mesh is intentionally low-detail and centered near the origin so
    it can be rotated in place like the existing wireframe body.
    """
    # base corners (clockwise front-left, front-right, rear-right, rear-left)
    flb = np.array([LNG, -WID * 0.9, H_FLOOR + 0.02])
    frb = np.array([LNG, WID * 0.9, H_FLOOR + 0.02])
    rrb = np.array([-LNG, WID * 0.9, H_FLOOR + 0.02])
    rlb = np.array([-LNG, -WID * 0.9, H_FLOOR + 0.02])

    # roof/upper shape
    flt = np.array([LNG * 0.45, -WID_ROOF, H_HOOD + 0.08])
    frt = np.array([LNG * 0.45, WID_ROOF, H_HOOD + 0.08])
    rrt = np.array([-LNG * 0.45, WID_ROOF, H_ROOF])
    rlt = np.array([-LNG * 0.45, -WID_ROOF, H_ROOF])

    verts = np.vstack([flb, frb, rrb, rlb, flt, frt, rrt, rlt])

    # faces (two triangles per rectangular face)
    faces = np.array([
        [0, 1, 4], [1, 5, 4],   # front sloped face
        [1, 2, 5], [2, 6, 5],   # right side
        [2, 3, 6], [3, 7, 6],   # rear
        [3, 0, 7], [0, 4, 7],   # left side
        [4, 5, 6], [4, 6, 7],   # top/roof
        [0, 3, 2], [0, 2, 1],   # bottom (visible if translucent)
    ])

    return verts.astype(float), faces.astype(int)


class _GLWidget(QWidget):
    """3D view with car body, ZED axes, MTi axes, and gravity vector."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.view = gl.GLViewWidget()
        # Slightly lighter background so the vehicle silhouettes pop
        self.view.setBackgroundColor(COLORS["bg_panel"])
        self.view.setCameraPosition(distance=6, elevation=25, azimuth=45)
        layout.addWidget(self.view)

        grid = gl.GLGridItem()
        try:
            grid.setSize(10, 10)
            grid.setSpacing(1, 1)
            grid.setColor(_hex_to_rgba(COLORS["border"], alpha=90))
        except Exception:
            # older pyqtgraph versions may not support setColor/setSpacing
            grid.setSize(10, 10)
        self.view.addItem(grid)

        # --- ZED axes (Electric Cyan at camera position) ---
        # Use theme primary colors with subtle shading for axis lines
        zed_base = _hex_to_rgba(COLORS["zed"], alpha=255)
        zed_colors = [
            zed_base,                              # X cyan (bright)
            (int(zed_base[0] * 0.75), int(zed_base[1] * 0.85), int(zed_base[2] * 0.9), 230),
            (int(zed_base[0] * 0.5), int(zed_base[1] * 0.6), int(zed_base[2] * 0.8), 200),
        ]
        self.zed_axes = []
        for c in zed_colors:
            line = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=c, width=3)
            self.view.addItem(line)
            self.zed_axes.append(line)
        self.zed_x_label = gl.GLTextItem(text="X", color=zed_colors[0], font=_LABEL_FONT)
        self.zed_y_label = gl.GLTextItem(text="Y", color=zed_colors[1], font=_LABEL_FONT)
        self.zed_z_label = gl.GLTextItem(text="Z", color=zed_colors[2], font=_LABEL_FONT)
        for lbl in [self.zed_x_label, self.zed_y_label, self.zed_z_label]:
            self.view.addItem(lbl)

        # --- MTi axes (Amber Orange at center) ---
        mti_base = _hex_to_rgba(COLORS["mti"], alpha=255)
        mti_colors = [
            mti_base,
            (int(mti_base[0] * 0.9), int(mti_base[1] * 0.75), int(mti_base[2] * 0.6), 240),
            (int(mti_base[0] * 0.75), int(mti_base[1] * 0.6), int(mti_base[2] * 0.45), 220),
        ]
        self.mti_axes = []
        for c in mti_colors:
            line = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=c, width=3)
            self.view.addItem(line)
            self.mti_axes.append(line)
        self.mti_x_label = gl.GLTextItem(text="X", color=mti_colors[0], font=_LABEL_FONT)
        self.mti_y_label = gl.GLTextItem(text="Y", color=mti_colors[1], font=_LABEL_FONT)
        self.mti_z_label = gl.GLTextItem(text="Z", color=mti_colors[2], font=_LABEL_FONT)
        for lbl in [self.mti_x_label, self.mti_y_label, self.mti_z_label]:
            self.view.addItem(lbl)

        # --- Car body ---
        # Kept deliberately muted: these provide a refined, modern look.
        body_color = (180, 185, 195, 200)      # silhouette / greenhouse (soft gray)
        floor_color = (120, 125, 135, 180)     # rocker/floor (darker)
        tie_color = (140, 145, 155, 160)       # cross-braces
        wheel_color = (40, 40, 45, 230)        # tires (darker for contrast)
        axle_color = (160, 160, 170, 200)

        # Draw the silhouette-based wireframe body (preferred for clarity)
        self.body_left = gl.GLLinePlotItem(pos=_profile_side(-1), color=body_color, width=2.2)
        self.body_right = gl.GLLinePlotItem(pos=_profile_side(1), color=body_color, width=2.2)
        self.body_floor = gl.GLLinePlotItem(pos=_floor_loop(), color=floor_color, width=2.0)
        self.body_ties = gl.GLLinePlotItem(pos=_cross_ties(), color=tie_color, width=1.2, mode="lines")
        for item in [self.body_left, self.body_right, self.body_floor, self.body_ties]:
            self.view.addItem(item)

        self.axles = gl.GLLinePlotItem(pos=_axle_lines(), color=axle_color, width=2.0, mode="lines")
        self.view.addItem(self.axles)

        self.wheels = []
        for cx, sign in _wheel_positions():
            # Slightly thicker wheel outlines for a premium look
            w = gl.GLLinePlotItem(pos=_wheel_circle(cx, sign), color=wheel_color, width=3.0)
            self.view.addItem(w)
            self.wheels.append(w)

        # Forward arrow (cyan)
        arrow_base = np.array([0, 0, H_HOOD + 0.15])
        arrow_tip = np.array([LNG + 0.5, 0, H_HOOD + 0.15])
        self.forward_arrow = gl.GLLinePlotItem(
            pos=np.array([arrow_base, arrow_tip]),
            color=_hex_to_rgba(COLORS["zed"], alpha=255), width=4,
        )
        self.view.addItem(self.forward_arrow)
        ah1 = np.array([arrow_tip, arrow_tip + np.array([-0.2, 0.12, 0])])
        ah2 = np.array([arrow_tip, arrow_tip + np.array([-0.2, -0.12, 0])])
        self.arrow_head1 = gl.GLLinePlotItem(pos=ah1, color=_hex_to_rgba(COLORS["zed"], alpha=240), width=4)
        self.arrow_head2 = gl.GLLinePlotItem(pos=ah2, color=_hex_to_rgba(COLORS["zed"], alpha=240), width=4)
        self.view.addItem(self.arrow_head1)
        self.view.addItem(self.arrow_head2)

        # Camera marker
        cam_pos = _CAM_LOCAL.reshape(1, 3)
        self.camera_marker = gl.GLLinePlotItem(
            pos=cam_pos, color=_hex_to_rgba(COLORS["zed"], alpha=255), width=6,
        )
        self.view.addItem(self.camera_marker)
        self.camera_label = gl.GLTextItem(text="CAM", color=_hex_to_rgba(COLORS["zed"], alpha=255), font=_LABEL_FONT)
        self.view.addItem(self.camera_label)

        # MTi marker
        mti_pos = _MTI_LOCAL.reshape(1, 3)
        self.mti_marker = gl.GLLinePlotItem(
            pos=mti_pos, color=_hex_to_rgba(COLORS["mti"], alpha=255), width=7,
        )
        self.view.addItem(self.mti_marker)
        self.mti_label = gl.GLTextItem(text="MTi", color=_hex_to_rgba(COLORS["mti"], alpha=255), font=_LABEL_FONT)
        self.view.addItem(self.mti_label)

        # Gravity vector — a fixed world-down reference drawn beside the car
        # (not through its center), so it never overlaps the MTi Z axis,
        # which also originates at (0, 0, 0).
        gx, gy = _GRAVITY_XY
        self.gravity_line = gl.GLLinePlotItem(
            pos=np.array([[gx, gy, 0.3], [gx, gy, 0.3 - _GRAVITY_LEN]]),
            color=_hex_to_rgba(COLORS["yellow"] if "yellow" in COLORS else "#FFB800", alpha=170), width=2,
        )
        self.view.addItem(self.gravity_line)
        self.gravity_label = gl.GLTextItem(
            text="g", color=_hex_to_rgba(COLORS["yellow"] if "yellow" in COLORS else "#FFB800", alpha=200), font=_LABEL_FONT,
            pos=np.array([gx, gy, 0.3 - _GRAVITY_LEN - 0.15]),
        )
        self.view.addItem(self.gravity_label)

    def update_orientation(self, mti_roll=0, mti_pitch=0, mti_yaw=0,
                           zed_roll=0, zed_pitch=0, zed_yaw=0):
        """MTi-8 is sole navigation sensor. zed_rpy is diagnostic only."""
        # Body orientation driven by MTi-8
        R_body = euler_to_rotation_matrix(mti_roll, mti_pitch, mti_yaw)
        R_mti = R_body
        R_zed = euler_to_rotation_matrix(zed_roll, zed_pitch, zed_yaw)

        # Car body (rotated by body R)
        self.body_left.setData(pos=(R_body @ _profile_side(-1).T).T)
        self.body_right.setData(pos=(R_body @ _profile_side(1).T).T)
        self.body_floor.setData(pos=(R_body @ _floor_loop().T).T)
        self.body_ties.setData(pos=(R_body @ _cross_ties().T).T)

        self.axles.setData(pos=(R_body @ _axle_lines().T).T)

        for i, (cx, sign) in enumerate(_wheel_positions()):
            self.wheels[i].setData(pos=(R_body @ _wheel_circle(cx, sign).T).T)

        # Forward arrow
        arrow_base = R_body @ np.array([0, 0, H_HOOD + 0.15])
        arrow_tip = R_body @ np.array([LNG + 0.5, 0, H_HOOD + 0.15])
        self.forward_arrow.setData(pos=np.array([arrow_base, arrow_tip]))
        self.arrow_head1.setData(pos=np.array([
            arrow_tip, arrow_tip + R_body @ np.array([-0.2, 0.12, 0])
        ]))
        self.arrow_head2.setData(pos=np.array([
            arrow_tip, arrow_tip + R_body @ np.array([-0.2, -0.12, 0])
        ]))

        # Camera marker (body-fixed)
        cam_world = R_body @ _CAM_LOCAL
        self.camera_marker.setData(pos=cam_world.reshape(1, 3))
        self.camera_label.setData(pos=cam_world + np.array([0, 0, 0.15]))

        # MTi marker (body-fixed)
        mti_world = R_body @ _MTI_LOCAL
        self.mti_marker.setData(pos=mti_world.reshape(1, 3))
        self.mti_label.setData(pos=mti_world + np.array([0, 0, 0.15]))

        # ZED axes at camera position. Deliberately a different length than
        # MTi's so that when the two sensors agree (the normal, healthy
        # case) their axis tips and labels don't land on top of each other.
        ZED_LEN = 0.85
        zed_origin = R_body @ _CAM_LOCAL
        x_line, y_line, z_line = _make_axis_lines(
            R_zed, zed_origin, length=ZED_LEN, width=3)
        self.zed_axes[0].setData(pos=x_line)
        self.zed_axes[1].setData(pos=y_line)
        self.zed_axes[2].setData(pos=z_line)
        zoff = ZED_LEN * 1.18
        lx = zed_origin + R_zed @ np.array([zoff, 0, 0])
        ly = zed_origin + R_zed @ np.array([0, zoff, 0])
        lz = zed_origin + R_zed @ np.array([0, 0, zoff])
        self.zed_x_label.setData(pos=lx)
        self.zed_y_label.setData(pos=ly)
        self.zed_z_label.setData(pos=lz)

        # MTi axes at center — shorter, since MTi already has its own
        # bright node label right next to it.
        MTI_LEN = 0.55
        mti_origin = R_body @ _MTI_LOCAL
        x_line, y_line, z_line = _make_axis_lines(
            R_mti, mti_origin, length=MTI_LEN, width=3)
        self.mti_axes[0].setData(pos=x_line)
        self.mti_axes[1].setData(pos=y_line)
        self.mti_axes[2].setData(pos=z_line)
        moff = MTI_LEN * 1.25
        mlx = mti_origin + R_mti @ np.array([moff, 0, 0])
        mly = mti_origin + R_mti @ np.array([0, moff, 0])
        mlz = mti_origin + R_mti @ np.array([0, 0, moff])
        self.mti_x_label.setData(pos=mlx)
        self.mti_y_label.setData(pos=mly)
        self.mti_z_label.setData(pos=mlz)

        # Gravity is a fixed world reference beside the car; it doesn't move
        # with orientation, so nothing to update here each frame.


class _FallbackWidget(QWidget):
    """Text fallback when OpenGL is not available."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.label = QLabel("Roll: 0.0\u00b0  Pitch: 0.0\u00b0  Yaw: 0.0\u00b0")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label.setStyleSheet(
            f"color: {COLORS['text_bright']}; font-size: 22px; font-weight: bold;"
            f" background-color: {COLORS['bg_dark']}; padding: 30px;"
        )
        layout.addWidget(self.label)

    def update_orientation(self, mti_roll=0, mti_pitch=0, mti_yaw=0,
                           zed_roll=0, zed_pitch=0, zed_yaw=0):
        self.label.setText(
            f"ZED R:{zed_roll:.1f}\u00b0 P:{zed_pitch:.1f}\u00b0 Y:{zed_yaw:.1f}\u00b0"
            f"  |  MTi R:{mti_roll:.1f}\u00b0 P:{mti_pitch:.1f}\u00b0 Y:{mti_yaw:.1f}\u00b0"
        )


class Vehicle3DPanel(QWidget):
    """3D orientation display with car body, ZED axes, and MTi axes."""

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        group = QGroupBox("Vehicle Orientation (MTi-8 sole nav source)")
        group_layout = QVBoxLayout(group)

        if HAS_GL:
            self._view = _GLWidget()
        else:
            self._view = _FallbackWidget()
        group_layout.addWidget(self._view, stretch=1)

        self.info_label = QLabel("")
        self.info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.info_label.setStyleSheet(
            f"color: {COLORS['text_bright']}; font-size: 13px; font-weight: bold;"
        )
        group_layout.addWidget(self.info_label)

        layout.addWidget(group)

    def update_orientation(self, zed_roll=0, zed_pitch=0, zed_yaw=0,
                           mti_roll=0, mti_pitch=0, mti_yaw=0):
        # MTi-8 is sole body orientation source
        self._view.update_orientation(
            mti_roll=mti_roll, mti_pitch=mti_pitch, mti_yaw=mti_yaw,
            zed_roll=zed_roll, zed_pitch=zed_pitch, zed_yaw=zed_yaw,
        )
        self.info_label.setText(
            f'<span style="color:{COLORS["mti"]};">'
            f'MTi-8 R:{mti_roll:+.1f}\u00b0 '
            f'P:{mti_pitch:+.1f}\u00b0 Y:{mti_yaw:+.1f}\u00b0</span>'
        )