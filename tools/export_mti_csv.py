"""Export MTi HDF5 sensor data to CSV for inspection.

Usage:
    python tools/export_mti_csv.py sensors/20260727_151423_mti.h5 output.csv
    python tools/export_mti_csv.py sensors/20260727_151423_mti.h5 output.csv --start 1000000000 --end 2000000000
"""

import argparse
import csv
import sys
from pathlib import Path

import h5py
import numpy as np


def export(h5_path: str, csv_path: str, start_ns: int = None, end_ns: int = None):
    h5_path = Path(h5_path)
    if not h5_path.exists():
        print(f"File not found: {h5_path}")
        sys.exit(1)

    with h5py.File(str(h5_path), "r") as f:
        mti = f.get("mti")
        if mti is None:
            print("No /mti group found in file")
            sys.exit(1)

        timestamps = mti["timestamp_ns"][:]
        mask = np.ones(len(timestamps), dtype=bool)
        if start_ns is not None:
            mask &= timestamps >= start_ns
        if end_ns is not None:
            mask &= timestamps <= end_ns

        indices = np.where(mask)[0]
        print(f"Exporting {len(indices)} of {len(timestamps)} samples...")

        with open(csv_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            header = list(mti.keys())
            writer.writerow(header)
            for i in indices:
                row = [mti[k][i] for k in header]
                writer.writerow(row)

    print(f"Exported to {csv_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export HDF5 MTi data to CSV")
    parser.add_argument("h5_file", help="Path to .h5 sensor file")
    parser.add_argument("csv_file", help="Output CSV path")
    parser.add_argument("--start", type=int, default=None,
                        help="Start timestamp_ns (inclusive)")
    parser.add_argument("--end", type=int, default=None,
                        help="End timestamp_ns (inclusive)")
    args = parser.parse_args()
    export(args.h5_file, args.csv_file, args.start, args.end)
