"""Mini OpenStreetMap panel — raster tile view centered on the GNSS fix.

Mirrors MTi8-Navigation/visualization/map.html: zoom 15, red accuracy
circle (fillOpacity 0.25) and a green trail polyline capped at 1000 pts,
Fetches OSM tiles over HTTPS in a background thread; the GUI thread only
draws cached tiles, so an offline link just shows the loading placeholder.
"""

from __future__ import annotations
import math
import threading
import urllib.request

from PySide6.QtCore import QPointF, QRect, QTimer, Qt
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen
from PySide6.QtWidgets import QWidget

from dashboard.theme import COLORS, FONT_SANS

ZOOM = 15
TILE = 256
MAX_CACHE = 256
TRAIL_MAX = 1000
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "stereo-dataset-gcs/0.1 (ground control station)"
# ponytail: HDOP->meters proxy (matches map.html accuracy-in-meters).
# Replace with true horizontal_accuracy when a UBX/accuracy source exists.
HDOP_TO_METERS = 5.0
ACC_CIRCLE_COLOR = QColor(255, 68, 68)            # #ff4444
TRAIL_COLOR = QColor(0, 204, 136, 178)            # #00cc88 @ opacity .7


def _tile_xy(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Web-Mercator position in fractional tile coordinates."""
    n = 2 ** zoom
    x = (lon + 180.0) / 360.0 * n
    lat = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(lat)) / math.pi) / 2.0 * n
    return x, y


def _pixel_offset(lat: float, lon: float, clat: float, clon: float) -> tuple[float, float]:
    """Pixel offset of (lat,lon) from center (clat,clon) at ZOOM."""
    n = 2 ** ZOOM
    x = (lon + 180.0) / 360.0 * n * TILE
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n * TILE
    cx = (clon + 180.0) / 360.0 * n * TILE
    cy = (1.0 - math.asinh(math.tan(math.radians(clat))) / math.pi) / 2.0 * n * TILE
    return x - cx, y - cy


def _meters_to_px(lat: float, m: float) -> float:
    """Ground meters to pixels at ZOOM (Web-Mercator, m/px scales with cos(lat))."""
    mpp = 156543.03392 * math.cos(math.radians(lat)) / (2 ** ZOOM)
    return m / mpp if mpp else 0.0


class MapPanel(QWidget):
    """Small corner map: OSM tiles + a marker at the current position."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(240, 170)
        self._lat = None
        self._lon = None
        self._label = "No GNSS fix"
        self._color = COLORS["red"]
        self._accuracy_m = 0.0
        self._trail = []
        self._cache = {}       # (z,x,y) -> PNG bytes, written by workers
        self._decoded = {}     # (z,x,y) -> QImage, GUI thread only
        self._in_flight = set()

    def set_position(self, lat, lon, label: str, color: str,
                     accuracy_m: float = 0.0):
        self._lat = lat
        self._lon = lon
        self._label = label
        self._color = color
        self._accuracy_m = accuracy_m
        if lat is not None and lon is not None:
            self._trail.append((lat, lon))
            if len(self._trail) > TRAIL_MAX:
                self._trail.pop(0)
        self._schedule_missing()
        self.update()

    # ── tile management ────────────────────────────────────────────

    def _needed_keys(self):
        if self._lat is None:
            return []
        x, y = _tile_xy(self._lat, self._lon, ZOOM)
        tx0 = math.floor(x - self.width() / 2 / TILE)
        tx1 = math.floor(x + self.width() / 2 / TILE)
        ty0 = math.floor(y - self.height() / 2 / TILE)
        ty1 = math.floor(y + self.height() / 2 / TILE)
        return [(ZOOM, tx, ty)
                for tx in range(tx0, tx1 + 1)
                for ty in range(ty0, ty1 + 1)]

    def _schedule_missing(self):
        for key in self._needed_keys():
            if key in self._cache or key in self._decoded \
                    or key in self._in_flight:
                continue
            self._in_flight.add(key)
            threading.Thread(target=self._fetch, args=(key,), daemon=True).start()

    def _fetch(self, key):
        try:
            z, tx, ty = key
            req = urllib.request.Request(
                TILE_URL.format(z=z, x=tx, y=ty),
                headers={"User-Agent": USER_AGENT},
            )
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = resp.read()
            if len(self._cache) >= MAX_CACHE:
                self._cache.clear()
            if len(self._decoded) >= MAX_CACHE:
                self._decoded.clear()
            self._cache[key] = data
        except Exception:
            pass
        finally:
            self._in_flight.discard(key)
            QTimer.singleShot(0, self.update)

    def _tile_image(self, key):
        img = self._decoded.get(key)
        if img is not None:
            return img
        data = self._cache.get(key)
        if data is None:
            return None
        img = QImage()
        if img.loadFromData(data, "PNG"):
            self._decoded[key] = img
            return img
        return None

    # ── drawing ────────────────────────────────────────────────────

    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(COLORS["bg_dark"]))
        p.setPen(QColor(COLORS["border"]))
        p.drawRect(self.rect().adjusted(0, 0, -1, -1))

        if self._lat is None:
            p.setPen(QColor(COLORS["text_dim"]))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self._label)
            return

        x, y = _tile_xy(self._lat, self._lon, ZOOM)
        w, h = self.width(), self.height()
        tx0 = math.floor(x - w / 2 / TILE)
        tx1 = math.floor(x + w / 2 / TILE)
        ty0 = math.floor(y - h / 2 / TILE)
        ty1 = math.floor(y + h / 2 / TILE)
        ox = (x - tx0) * TILE - w / 2
        oy = (y - ty0) * TILE - h / 2

        any_tile = False
        for tx in range(tx0, tx1 + 1):
            for ty in range(ty0, ty1 + 1):
                img = self._tile_image((ZOOM, tx, ty))
                if img is None:
                    continue
                any_tile = True
                p.drawImage(QPointF((tx - tx0) * TILE - ox,
                                    (ty - ty0) * TILE - oy), img)
        if not any_tile:
            p.setPen(QColor(COLORS["text_dim"]))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                       "Loading map…")

        # Trail polyline (green #00cc88 @ opacity .7, like map.html)
        if self._trail:
            p.setRenderHint(QPainter.Antialiasing, True)
            pen = QPen(TRAIL_COLOR)
            pen.setWidth(3)
            p.setPen(pen)
            pts = [QPointF(w / 2 + dx, h / 2 + dy) for dx, dy in
                   (_pixel_offset(la, lo, self._lat, self._lon)
                    for la, lo in self._trail)]
            p.drawPolyline(pts)

        # Accuracy circle (#ff4444, radius = accuracy, fillOpacity 0.25)
        if self._accuracy_m > 0:
            r = _meters_to_px(self._lat, self._accuracy_m)
            r = max(r, 1.5)
            ap = QPen(ACC_CIRCLE_COLOR)
            ap.setWidth(2)
            p.setPen(ap)
            fill = QColor(ACC_CIRCLE_COLOR)
            fill.setAlpha(64)  # fillOpacity 0.25
            p.setBrush(fill)
            p.drawEllipse(QPointF(w / 2, h / 2), r, r)

        # Position marker (always centered)
        cx, cy = w / 2, h / 2
        p.setBrush(QColor(self._color))
        p.setPen(QPen(QColor(COLORS["bg_dark"]), 2))
        p.drawEllipse(QPointF(cx, cy), 5, 5)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), 11, 11)

        # Status chip
        chip = QRect(4, 4, min(w - 8, 220), 18)
        p.fillRect(chip, QColor(0, 0, 0, 160))
        p.setPen(QColor(self._color))
        p.setFont(QFont(FONT_SANS, 9, QFont.Weight.Bold))
        p.drawText(chip, Qt.AlignmentFlag.AlignCenter, self._label)
