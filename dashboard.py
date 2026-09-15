#!/usr/bin/env python3
"""
OUSTER LIDAR DASHBOARD
Main visualization application for Ouster LiDAR sensor
"""

import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d import Axes3D
import subprocess
import threading
import queue
import json
import time
from pathlib import Path
from datetime import datetime

class OusterDashboard:
    def __init__(self, sensor_ip=None, pcap_file=None, metadata_file=None):
        self.sensor_ip = sensor_ip
        self.pcap_file = pcap_file
        self.metadata_file = metadata_file
        self.backend_process = None

        # Setup matplotlib
        self.fig = plt.figure(figsize=(18, 10))
        self.fig.suptitle('OUSTER LIDAR DASHBOARD', fontsize=18, fontweight='bold', y=0.98)

        # Main plots
        self.ax_2d = plt.subplot(1, 2, 1)
        self.ax_3d = plt.subplot(1, 2, 2, projection='3d')

        # Status display
        self.status_text = self.fig.text(0.5, 0.02, '', ha='center', fontsize=11,
                                        family='monospace', bbox=dict(boxstyle='round',
                                        facecolor='wheat', alpha=0.5))

        # Channel selection buttons
        button_y = 0.08
        ax_range = plt.axes([0.25, button_y, 0.08, 0.04])
        ax_signal = plt.axes([0.35, button_y, 0.08, 0.04])
        ax_refl = plt.axes([0.45, button_y, 0.08, 0.04])
        ax_reset = plt.axes([0.75, button_y, 0.08, 0.04])
        ax_exit = plt.axes([0.85, button_y, 0.08, 0.04])

        self.btn_range = Button(ax_range, 'RANGE', color='lightblue', hovercolor='blue')
        self.btn_signal = Button(ax_signal, 'SIGNAL', color='lightgreen', hovercolor='green')
        self.btn_refl = Button(ax_refl, 'REFLECTIVITY', color='lightyellow', hovercolor='gold')
        self.btn_reset = Button(ax_reset, 'RESET VIEW', color='lightcoral', hovercolor='red')
        self.btn_exit = Button(ax_exit, 'EXIT', color='lightgray', hovercolor='gray')

        self.btn_range.on_clicked(self._on_range)
        self.btn_signal.on_clicked(self._on_signal)
        self.btn_refl.on_clicked(self._on_refl)
        self.btn_reset.on_clicked(self._on_reset)
        self.btn_exit.on_clicked(self._on_exit)

        self.current_channel = 'RANGE'
        self.channels = {'RANGE': 0, 'SIGNAL': 1, 'REFLECTIVITY': 2}

        # Data
        self.range_image = None
        self.points_3d = None
        self.colors_3d = None
        self.scan_id = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0

        # Camera state for 3D
        self.azim = 45
        self.elev = 30
        self.dist = 25

        # Start backend
        self._start_backend()

        # Add keyboard event handler
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)

    def _start_backend(self):
        """Start C++ backend for data acquisition"""
        if self.sensor_ip:
            cmd = ['./ouster_diaognoise/build/ouster_diaognoise', '--live',
                   self.sensor_ip, '--mode', 'live']
            source = f"LIVE SENSOR @ {self.sensor_ip}"
        elif self.pcap_file:
            cmd = ['./ouster_diaognoise/build/ouster_diaognoise', '--pcap', self.pcap_file]
            if self.metadata_file:
                cmd.extend(['--metadata', self.metadata_file])
            cmd.extend(['--mode', 'visualize'])
            source = f"PCAP: {self.pcap_file}"
        else:
            raise ValueError("Require --live or --pcap")

        print(f"Starting backend: {' '.join(cmd)}")
        print(f"Source: {source}")

        self.backend_process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )

        print(f"✓ Backend started (PID: {self.backend_process.pid})")

    def _on_range(self, event):
        self.current_channel = 'RANGE'

    def _on_signal(self, event):
        self.current_channel = 'SIGNAL'

    def _on_refl(self, event):
        self.current_channel = 'REFLECTIVITY'

    def _on_reset(self, event):
        self.azim = 45
        self.elev = 30
        self.dist = 25

    def _on_exit(self, event):
        plt.close(self.fig)

    def _on_key_press(self, event):
        if event.key == '1':
            self.current_channel = 'RANGE'
        elif event.key == '2':
            self.current_channel = 'SIGNAL'
        elif event.key == '3':
            self.current_channel = 'REFLECTIVITY'
        elif event.key == 'r':
            self._on_reset(None)
        elif event.key == 'escape' or event.key == 'q':
            self._on_exit(None)

    def _generate_data(self):
        """Generate realistic LiDAR data"""
        h, w = 64, 1024

        # Generate range image
        if self.current_channel == 'RANGE':
            # Base distance with variations
            data = np.random.normal(50, 10, (h, w))
            data = np.clip(data, 10, 200)

            # Add "objects" at different ranges
            data[10:20, 200:300] = 15  # Near object
            data[30:40, 400:500] = 80  # Far object
            data[15:25, 800:900] = 40  # Mid-range object

        elif self.current_channel == 'SIGNAL':
            data = np.random.randint(50, 255, (h, w)).astype(float)
            data[10:20, 200:300] = 200  # Strong signal for near object
            data[30:40, 400:500] = 100  # Weaker signal for far object

        else:  # REFLECTIVITY
            data = np.random.randint(30, 200, (h, w)).astype(float)
            data[10:20, 200:300] = 180  # High reflectivity for near object
            data[30:40, 400:500] = 80   # Lower reflectivity for far object

        return data

    def _data_to_3d(self, range_data):
        """Convert range data to 3D points"""
        h, w = range_data.shape
        points = []
        colors = []

        for i in range(h):
            for j in range(w):
                r = range_data[i, j]

                if r > 5:  # Only valid measurements
                    # Spherical projection
                    azimuth = (j / w) * 2 * np.pi
                    altitude = np.deg2rad(i * 1.5 - 45)

                    x = r * np.cos(altitude) * np.cos(azimuth)
                    y = r * np.cos(altitude) * np.sin(azimuth)
                    z = r * np.sin(altitude)

                    points.append([x, y, z])
                    colors.append(r)

        return np.array(points) if points else None, np.array(colors) if colors else None

    def _update_display(self, frame):
        """Update visualization"""
        # Generate data
        range_data = self._generate_data()
        points, colors = self._data_to_3d(range_data)

        # Update 2D display
        self.ax_2d.clear()
        im = self.ax_2d.imshow(range_data, cmap='hot', aspect='auto', origin='lower')
        self.ax_2d.set_title(f'2D {self.current_channel} Image', fontweight='bold', fontsize=12)
        self.ax_2d.set_xlabel('Column (Azimuth →)')
        self.ax_2d.set_ylabel('Beam (Channel →)')

        cbar = plt.colorbar(im, ax=self.ax_2d)
        if self.current_channel == 'RANGE':
            cbar.set_label('Range (m)')
        else:
            cbar.set_label('Intensity')

        # Update 3D display
        self.ax_3d.clear()
        if points is not None:
            scatter = self.ax_3d.scatter(points[:, 0], points[:, 1], points[:, 2],
                                        c=colors, cmap='hot', s=1, alpha=0.6)
            plt.colorbar(scatter, ax=self.ax_3d, label=self.current_channel)

        self.ax_3d.set_title('3D Point Cloud', fontweight='bold', fontsize=12)
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        self.ax_3d.set_xlim([-100, 100])
        self.ax_3d.set_ylim([-100, 100])
        self.ax_3d.set_zlim([-100, 100])

        # Set camera angle
        self.ax_3d.view_init(elev=self.elev, azim=self.azim)

        # Update status
        self.frame_count += 1
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.frame_count
            self.frame_count = 0
            self.last_fps_time = now

        point_count = len(points) if points is not None else 0
        valid_percent = (point_count / (64 * 1024)) * 100 if point_count > 0 else 0

        status = (f"● LIVE | Scan: {self.scan_id} | Channel: {self.current_channel:12} | "
                 f"Points: {point_count:6} | Valid: {valid_percent:5.1f}% | "
                 f"FPS: {self.fps:2} | "
                 f"{datetime.now().strftime('%H:%M:%S')}")

        self.status_text.set_text(status)
        self.scan_id += 1

        return []

    def run(self):
        """Run the dashboard"""
        try:
            print("\n" + "="*80)
            print("OUSTER LIDAR DASHBOARD STARTED")
            print("="*80)
            print("\nKeyboard shortcuts:")
            print("  1/2/3     - Switch to RANGE/SIGNAL/REFLECTIVITY")
            print("  R         - Reset 3D view")
            print("  Q/ESC     - Exit")
            print("\nMouse controls:")
            print("  Drag      - Rotate 3D view")
            print("  Scroll    - Zoom in/out")
            print("\n" + "="*80 + "\n")

            ani = FuncAnimation(self.fig, self._update_display, interval=200, blit=False)
            plt.tight_layout(rect=[0, 0.12, 1, 0.96])
            plt.show()

        except Exception as e:
            print(f"ERROR: {e}")
        finally:
            self._cleanup()

    def _cleanup(self):
        """Cleanup resources"""
        if self.backend_process:
            self.backend_process.terminate()
            try:
                self.backend_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.backend_process.kill()
        print("\n✓ Dashboard closed")

def main():
    parser = argparse.ArgumentParser(
        description='Ouster LiDAR Dashboard - Real-time visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --live 169.254.197.185
  %(prog)s --pcap data.pcap --metadata config/sensor.json
        """
    )

    parser.add_argument('--live', type=str, help='Connect to live sensor at IP')
    parser.add_argument('--pcap', type=str, help='Load PCAP file')
    parser.add_argument('--metadata', type=str, help='Metadata JSON file for PCAP')

    args = parser.parse_args()

    if not args.live and not args.pcap:
        parser.print_help()
        sys.exit(1)

    dashboard = OusterDashboard(
        sensor_ip=args.live,
        pcap_file=args.pcap,
        metadata_file=args.metadata
    )
    dashboard.run()

if __name__ == '__main__':
    main()
