"""Dataset playback tool.

Replays a recorded dataset at real speed with stereo images, depth,
disparity, and sensor overlays.

Usage:
    python playback.py <dataset_dir> [--speed 1.0] [--no-depth]

Controls:
    Space   Pause / Resume
    a f     Step one frame left/right (while paused)
    + -     Change speed (0.25x to 4x)
    d       Toggle depth overlay
    g       Toggle GNSS overlay
    i       Toggle IMU overlay
    q/Esc   Quit
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from dataset.reader import detect_format, load_v2_frames, load_v2_mti


# ── Common ───────────────────────────────────────────────────────────────

def find_nearest(target_ns, lookup, max_delta_ns=50_000_000):
    if not lookup:
        return None
    best_key = min(lookup.keys(), key=lambda k: abs(k - target_ns))
    if abs(best_key - target_ns) > max_delta_ns:
        return None
    return lookup[best_key]


def load_image(root: Path, path_str: str) -> np.ndarray | None:
    p = root / path_str
    if not p.exists():
        return None
    return cv2.imread(str(p))


def load_depth(root: Path, path_str: str) -> np.ndarray | None:
    if not path_str:
        return None
    p = root / path_str
    if p.exists():
        return np.load(str(p))
    return None


def colorize_depth(depth, max_depth=20.0):
    d = np.clip(depth, 0, max_depth)
    d = (d / max_depth * 255).astype(np.uint8)
    return cv2.applyColorMap(d, cv2.COLORMAP_JET)


def overlay_text(img, lines, x=10, y=20, color=(255, 255, 255), scale=0.45, thickness=1):
    for i, line in enumerate(lines):
        cv2.putText(img, line, (x, y + i * 16),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 1,
                    cv2.LINE_AA)
        cv2.putText(img, line, (x, y + i * 16),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness,
                    cv2.LINE_AA)


# ── Playback ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Dataset playback")
    parser.add_argument("dataset", help="Path to dataset directory")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument("--no-depth", action="store_true", help="Disable depth overlay")
    parser.add_argument("--max-depth", type=float, default=20.0, help="Max depth for colormap (m)")
    args = parser.parse_args()

    root = Path(args.dataset)
    if not root.exists():
        print(f"Dataset not found: {root}")
        sys.exit(1)

    fmt = detect_format(root)
    print(f"Format: {fmt}")

    if fmt != "v2":
        print("Unsupported dataset format — only v2 is supported")
        sys.exit(1)

    frames = load_v2_frames(root)
    imu_data = load_v2_mti(root)
    gnss_data = {}

    if not frames:
        print("No frames found")
        sys.exit(1)

    total = len(frames)
    print(f"Dataset: {root.name} — {total} frames")
    print(f"IMU entries: {len(imu_data)}, GNSS entries: {len(gnss_data)}")

    speed = args.speed
    show_depth = not args.no_depth
    show_imu = True
    show_gnss = True
    paused = False
    frame_idx = 0
    window = "Dataset Playback"

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 480)

    last_real_time = time.monotonic()
    last_frame_ns = int(frames[0]["timestamp_ns"])

    while 0 <= frame_idx < total:
        ts = frames[frame_idx]

        camera_ns = int(ts["timestamp_ns"])
        left_path = ts.get("left_path", "")
        right_path = ts.get("right_path", "")
        depth_path = ts.get("depth_path", "")
        left = load_image(root, left_path)
        right = load_image(root, right_path)
        frame_id = int(ts.get("frame_id", frame_idx))

        if left is None or right is None:
            frame_idx += 1
            continue

        h = min(left.shape[0], right.shape[0])
        left = left[:h]
        right = right[:h]
        stereo = np.hstack([left, right])

        # Depth overlay
        if show_depth:
            depth = load_depth(root, depth_path) if depth_path else None
            if depth is not None:
                depth_color = colorize_depth(depth, args.max_depth)
                if depth_color.shape[:2] != stereo.shape[:2]:
                    depth_color = cv2.resize(depth_color, (stereo.shape[1], stereo.shape[0]))
                stereo = cv2.addWeighted(stereo, 0.7, depth_color, 0.3, 0)

        # Text overlays
        lines = [f"Frame {frame_idx}/{total}  |  ID {frame_id:06d}  |  Speed {speed:.2f}x"]
        if paused:
            lines[0] += "  |  PAUSED"

        # IMU overlay
        if show_imu and imu_data:
            imu = find_nearest(camera_ns, imu_data)
            if imu:
                try:
                    ax = float(imu["ax"])
                    ay = float(imu["ay"])
                    az = float(imu["az"])
                    gx = float(imu["gx"])
                    gy = float(imu["gy"])
                    gz = float(imu["gz"])
                    mag = math.sqrt(ax**2 + ay**2 + az**2)
                    lines.append(f"IMU: |a|={mag:.2f} m/s²  g=({gx:.3f},{gy:.3f},{gz:.3f}) rad/s")
                except (ValueError, KeyError):
                    pass

        # GNSS overlay
        if show_gnss and gnss_data:
            gnss = find_nearest(camera_ns, gnss_data)
            if gnss:
                try:
                    lat, lon = float(gnss["lat"]), float(gnss["lon"])
                    if lat != 0:
                        lines.append(f"GPS: {lat:.6f}° {lon:.6f}°  sats={gnss.get('satellites','?')}")
                except (ValueError, KeyError):
                    pass

        overlay_text(stereo, lines)
        cv2.imshow(window, stereo)

        # Timing
        if not paused:
            delta_ns = camera_ns - last_frame_ns
            target_dt = delta_ns / 1e9 / speed
            now = time.monotonic()
            elapsed = now - last_real_time
            wait_ms = max(1, int((target_dt - elapsed) * 1000))
        else:
            wait_ms = 30

        key = cv2.waitKey(wait_ms) & 0xFF

        if key == ord("q") or key == 27:
            break
        elif key == ord(" "):
            paused = not paused
            if not paused:
                last_real_time = time.monotonic()
                last_frame_ns = camera_ns
        elif key == ord("+") or key == ord("="):
            speed = min(4.0, speed * 1.5)
        elif key == ord("-"):
            speed = max(0.25, speed / 1.5)
        elif key == ord("d"):
            show_depth = not show_depth
        elif key == ord("g"):
            show_gnss = not show_gnss
        elif key == ord("i"):
            show_imu = not show_imu
        elif key == ord("a"):
            if paused:
                frame_idx = max(0, frame_idx - 1)
                continue
        elif key == ord("f"):
            if paused:
                frame_idx = min(total - 1, frame_idx + 1)
                continue

        if not paused:
            last_real_time = time.monotonic()
            last_frame_ns = camera_ns
            frame_idx += 1

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
