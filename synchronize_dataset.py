#!/usr/bin/env python3
"""CLI: build the derived synchronized index for a recorded session.

Every recording session lives in its own self-contained subfolder,
<dataset_root>/<session_id>/. Reads the independent native-rate per-sensor
streams that one session already wrote and produces
<session_id>/metadata/synchronization/ — nearest-neighbour matches plus
residuals for each camera frame. Never modifies source data; safe to re-run.

Usage:
    python synchronize_dataset.py <dataset_root> [--session SESSION_ID]

A dataset root can hold many recorded sessions, each its own subfolder.
Pass --session to pick one by its session_id (a directory name under
dataset_root); omit it to use the most recently modified session.
"""

import argparse
import sys

from fusion.offline_sync import synchronize_session
from dataset.reader import list_sessions


def main():
    parser = argparse.ArgumentParser(
        description="Build the derived synchronized index for a recorded dataset."
    )
    parser.add_argument("dataset_root", help="Path to the dataset root directory")
    parser.add_argument("--session", dest="session_id", default=None,
                        help="Session ID to process (a subfolder name under "
                             "dataset_root). Omit to use the most recently "
                             "modified session.")
    args = parser.parse_args()

    session_id = args.session_id
    if session_id is None:
        sessions = list_sessions(args.dataset_root)
        if not sessions:
            print(f"No sessions found under {args.dataset_root}")
            return 1
        session_id = sessions[0]

    n = synchronize_session(args.dataset_root, session_id=session_id)
    print(f"Wrote {n} synchronized rows to "
          f"{args.dataset_root}/{session_id}/metadata/synchronization/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
