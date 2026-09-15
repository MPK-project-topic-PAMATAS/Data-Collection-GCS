"""PyTorch dataset adapter — lightweight, optionally loads depth/MTi.

Usage:
    # Image-only training (no HDF5 opened):
    dataset = StereoDataset("dataset/", load_depth=False, load_mti=False)
    left, label = dataset[0]

    # Multimodal training:
    dataset = StereoDataset("dataset/", load_depth=True, load_mti=True)
    left, depth, mti_acc, mti_gyro = dataset[0]
"""

from __future__ import annotations
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq

try:
    import torch
    from torch.utils.data import Dataset
    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False


class StereoDataset:
    """PyTorch-compatible dataset for stereo camera data.

    Reads from the Parquet frame index — no HDF5 needed for image-only mode.
    """

    def __init__(self, root: str, load_depth: bool = False,
                 load_disparity: bool = False, load_mti: bool = False,
                 transform=None):
        self.root = Path(root)
        self.load_depth = load_depth
        self.load_disparity = load_disparity
        self.load_mti = load_mti
        self.transform = transform

        # Load frame index
        frames_dir = self.root / "metadata" / "frames"
        tables = []
        for part in sorted(frames_dir.rglob("*.parquet")):
            tables.append(pq.read_table(str(part)))
        if tables:
            self._index = pq.concat_tables(tables).to_pandas()
        else:
            self._index = None

    def __len__(self):
        if self._index is None:
            return 0
        return len(self._index)

    def __getitem__(self, idx):
        if self._index is None:
            raise IndexError("Dataset is empty")
        row = self._index.iloc[idx]

        left_path_str = row.get("left_path", "")
        if not left_path_str:
            # SVO2-only recording mode: no per-frame colour images are
            # written to disk (see dataset/writer.py) — stereo frames must
            # be extracted from the session's recording.svo2 instead.
            raise ValueError(
                "This frame has no left_path — the session was recorded in "
                "SVO2-only mode (no per-frame images on disk); extract "
                "frames from raw/recording.svo2 instead of StereoDataset."
            )

        # Load left RGB
        left_path = self.root / left_path_str
        left = cv2.imread(str(left_path), cv2.IMREAD_COLOR)
        left = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)

        if self.transform:
            left = self.transform(left)

        result = {"left": left, "frame_id": int(row["frame_id"])}

        if self.load_depth and row.get("depth_path"):
            depth_path = self.root / row["depth_path"]
            if depth_path.exists():
                result["depth"] = np.load(str(depth_path))

        if self.load_disparity and row.get("disparity_path"):
            disp_path = self.root / row["disparity_path"]
            if disp_path.exists():
                result["disparity"] = np.load(str(disp_path))

        # For PyTorch: convert numpy to tensors if torch available
        if _HAS_TORCH:
            for k in ["left", "depth", "disparity"]:
                if k in result:
                    result[k] = torch.from_numpy(
                        result[k].transpose(2, 0, 1) if result[k].ndim == 3
                        else result[k]
                    ).float() if result[k].dtype in (np.float32, np.float64) else \
                        torch.from_numpy(
                            result[k].transpose(2, 0, 1) if result[k].ndim == 3
                            else result[k]
                        )

        return result
