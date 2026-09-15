#!/usr/bin/env python3
"""Playback and visualize recorded multi-sensor dataset.

Usage:
    python playback_dataset.py [--dataset output] [--format pcap]

Supports:
  * LiDAR: PCAP + JSON visualization
  * Camera: SVO2 playback (with depth)
  * IMU: HDF5 time series
  * Radar: ROS bag visualization
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def print_dataset_info(root: Path):
    """Print dataset structure and statistics.

    Every recording session lives in its own self-contained subfolder,
    <root>/<session_id>/ (see DatasetWriter.begin_session()). This lists
    every session found under root, then shows a detailed breakdown of the
    most recently modified one.
    """
    from dataset.reader import list_sessions

    print(f"\n{'='*60}")
    print(f"Dataset: {root}")
    print(f"{'='*60}")

    sessions = list_sessions(root)
    if not sessions:
        print("\nNo sessions found (no <root>/<session_id>/raw/ subfolders).")
        return
    print(f"\n🗂  Sessions ({len(sessions)}): most recently modified first")
    for sid in sessions:
        print(f"   • {sid}")

    session_id = sessions[0]
    session_root = root / session_id
    print(f"\nDetail for most recent session: {session_id}")
    raw_dir = session_root / "raw"

    # Check for LiDAR data — exactly one pcap + one config JSON per session.
    # Scans are decoded from the pcap on demand, never read from per-frame
    # files (there are none).
    pcap_file = raw_dir / "lidar.pcap"
    if pcap_file.exists():
        print(f"\n📍 LiDAR:")
        size_mb = pcap_file.stat().st_size / (1024 * 1024)
        pcap_json = raw_dir / "lidar_0.json"
        print(f"   • {pcap_file.name} ({size_mb:.1f} MB) "
              f"— config JSON present: {pcap_json.exists()}")
        from dataset.reader import iter_lidar_frames
        if pcap_json.exists():
            try:
                sample = next(iter(iter_lidar_frames(pcap_file, pcap_json)))
                pts = sample.points
                if pts.size > 0:
                    print(f"   • First scan (frame_id {sample.frame_id}):")
                    print(f"     Points: {len(pts)}")
                    print(f"     X range: [{pts[:, 0].min():.2f}, {pts[:, 0].max():.2f}] m")
                    print(f"     Y range: [{pts[:, 1].min():.2f}, {pts[:, 1].max():.2f}] m")
                    print(f"     Z range: [{pts[:, 2].min():.2f}, {pts[:, 2].max():.2f}] m")
            except StopIteration:
                print("   • No decodable scans found in the pcap")
            except Exception as e:
                print(f"   • Could not decode pcap: {e}")

    # Check for camera data
    svo_files = sorted(raw_dir.glob("*.svo2")) if raw_dir.exists() else []
    if svo_files:
        print(f"\n📹 Camera:")
        for svo in svo_files:
            size_mb = svo.stat().st_size / (1024 * 1024)
            print(f"   • {svo.name}: {size_mb:.1f} MB")

    # Check for IMU data
    sensors_dir = session_root / "sensors"
    if sensors_dir.exists():
        h5_files = list(sensors_dir.glob("*.h5"))
        if h5_files:
            print(f"\n📊 IMU:")
            for h5 in h5_files:
                size_mb = h5.stat().st_size / (1024 * 1024)
                print(f"   • {h5.name}: {size_mb:.1f} MB")

    # Check for Radar data
    bag_files = sorted(raw_dir.glob("radar*.bag")) if raw_dir.exists() else []
    if bag_files:
        print(f"\n📡 Radar:")
        for bag in bag_files:
            size_mb = bag.stat().st_size / (1024 * 1024)
            print(f"   • {bag.name}: {size_mb:.1f} MB")

    # Check metadata
    meta_dir = session_root / "metadata"
    if meta_dir.exists():
        parquet_files = list((meta_dir / "frames").glob("**/*.parquet"))
        if parquet_files:
            print(f"\n📋 Metadata:")
            print(f"   • Frame index: {len(parquet_files)} parquet files")
            print(f"   • Sessions: {len(list((meta_dir / 'sessions').glob('**/*.parquet')))}")
            print(f"   • Events: {len(list((meta_dir / 'events').glob('**/*.parquet')))}")


def load_lidar_sample(root: Path, session_id: str | None = None) -> bool:
    """Decode and display one sample LiDAR scan from a session's pcap.

    Defaults to the most recently modified session's pcap when session_id
    is not given — see dataset.reader.resolve_lidar_paths.
    """
    from dataset.reader import DatasetReader

    reader = DatasetReader(root, session_id=session_id)
    if reader.num_lidar_frames == 0:
        print("No LiDAR scans found (no <session_id>/raw/lidar.pcap + "
              "its config JSON)")
        return False

    sample = reader.lidar_frames()[0]

    print(f"\n✓ Decoded LiDAR scan {sample.frame_id}:")
    print(f"  • Points: {len(sample.points)}")
    print(f"  • Reflectivity: {sample.reflectivity is not None}")
    print(f"  • Timestamp: {sample.timestamp_ns} ns (from pcap packet capture time)")

    return True


def main():
    parser = argparse.ArgumentParser(description="Playback multi-sensor dataset")
    parser.add_argument(
        "--dataset",
        type=str,
        default="output",
        help="Dataset root directory (default: output)",
    )
    parser.add_argument(
        "--format",
        type=str,
        choices=["pcap", "lidar", "camera", "imu", "radar"],
        help="Specific format to examine (default: all)",
    )
    args = parser.parse_args()

    root = Path(args.dataset)
    if not root.exists():
        print(f"Error: Dataset not found at {root}")
        sys.exit(1)

    # Show dataset info
    print_dataset_info(root)

    # Load sample if LiDAR data exists
    from dataset.reader import resolve_lidar_paths
    pcap_path, _ = resolve_lidar_paths(root)
    if pcap_path is not None:
        load_lidar_sample(root)

    print(f"\n{'='*60}")
    print("To use in your code:")
    print("  from dataset.reader import DatasetReader")
    print("  reader = DatasetReader('output', session_id='...')  # or omit for latest")
    print("  frame = reader.get_lidar_frame(0)  # decodes that session's pcap on demand")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
