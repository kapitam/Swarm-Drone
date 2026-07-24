"""SEC1 dataset: parse SD session logs, derive sector labels, PyTorch Dataset.

Record layout (little-endian, 9,439 B) — must match src/vision/vision_task.cpp
LogRecord exactly:

    u32  magic 'SEC1' (0x53454331)
    u16  seq
    u32  t_cap_ms
    u32  t_tof_ms
    u8   img[96*96]          grayscale, inference input
    u16  tof_mm[64]          raw VL53L5CX 8x8 distances, row-major
    u8   tof_status[64]      target_status per zone (5/9 = valid)
    i16  roll_mrad, pitch_mrad, yaw_mrad
    u8   battery_dv
    u8   model_ver
    u8   pred_bins[8]        live model shadow output (0xFF = none)
    u8   crc8                over the post-img tail

Labels (research docs 08 s4 / 09 s2.1): per sensor column, min distance over
middle rows 2..5 of zones with status in {5,9}; sector masked (-1) when the
column has <2 valid zones or ToF/frame skew exceeds 40 ms. Bin edges
[500, 1000, 2000] mm -> 4 ordinal bins (<0.5 / 0.5-1 / 1-2 / >2 m-or-free).
Raw zones are on disk so these edges can be re-tuned without re-flying.
"""

from __future__ import annotations

import io
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

IMG_W = IMG_H = 96
RECORD_SIZE = 9439
MAGIC = 0x53454331
HEADER = struct.Struct("<IHII")           # magic, seq, t_cap, t_tof
TAIL = struct.Struct("<hhhBB8sB")         # rpy, batt, model_ver, pred_bins, crc
BIN_EDGES_MM = (500, 1000, 2000)
VALID_STATUS = (5, 9)
ROW_MIN, ROW_MAX = 2, 5
MIN_VALID_ZONES = 2
MAX_SKEW_MS = 40


@dataclass
class Record:
    seq: int
    t_cap_ms: int
    t_tof_ms: int
    img: np.ndarray        # (96, 96) uint8
    tof_mm: np.ndarray     # (8, 8) uint16
    tof_status: np.ndarray # (8, 8) uint8
    roll: float
    pitch: float
    yaw: float
    pred_bins: np.ndarray  # (8,) uint8, 0xFF = n/a


def iter_records(path: Path):
    """Yield Records from one session .bin file (skips corrupt tails)."""
    data = Path(path).read_bytes()
    n = len(data) // RECORD_SIZE
    for i in range(n):
        chunk = data[i * RECORD_SIZE:(i + 1) * RECORD_SIZE]
        magic, seq, t_cap, t_tof = HEADER.unpack_from(chunk, 0)
        if magic != MAGIC:
            continue
        off = HEADER.size
        img = np.frombuffer(chunk, np.uint8, IMG_W * IMG_H, off)
        off += IMG_W * IMG_H
        tof_mm = np.frombuffer(chunk, np.uint16, 64, off).reshape(8, 8)
        off += 128
        tof_status = np.frombuffer(chunk, np.uint8, 64, off).reshape(8, 8)
        off += 64
        roll, pitch, yaw, batt, mv, pred, _crc = TAIL.unpack_from(chunk, off)
        yield Record(seq, t_cap, t_tof, img.reshape(IMG_H, IMG_W).copy(),
                     tof_mm.copy(), tof_status.copy(),
                     roll * 1e-3, pitch * 1e-3, yaw * 1e-3,
                     np.frombuffer(pred, np.uint8).copy())


def sector_min_mm(tof_mm: np.ndarray, tof_status: np.ndarray) -> np.ndarray:
    """(8,) float: min valid distance per column over rows 2..5; NaN = masked."""
    valid = np.isin(tof_status, VALID_STATUS)
    band_valid = valid[ROW_MIN:ROW_MAX + 1]           # (4, 8)
    band_mm = tof_mm[ROW_MIN:ROW_MAX + 1].astype(np.float32)
    band_mm[~band_valid] = np.nan
    counts = band_valid.sum(axis=0)                   # per column
    with np.errstate(all="ignore"):
        mins = np.nanmin(band_mm, axis=0)
    mins[counts < MIN_VALID_ZONES] = np.nan
    return mins


def bins_from_mm(mm: np.ndarray) -> np.ndarray:
    """(8,) int64: ordinal bin per sector; -1 = masked (no valid teacher)."""
    out = np.full(8, -1, dtype=np.int64)
    ok = ~np.isnan(mm)
    v = mm[ok]
    b = np.digitize(v, BIN_EDGES_MM)  # 0..3
    out[ok] = b
    return out


def depth_target_12(tof_mm: np.ndarray, tof_status: np.ndarray):
    """(12,12) meters + mask, nearest-zone projection (uPyD-Net-lite fork)."""
    valid = np.isin(tof_status, VALID_STATUS)
    d = tof_mm.astype(np.float32) / 1000.0
    idx = (np.arange(12) * 8) // 12
    grid = d[np.ix_(idx, idx)]
    mask = valid[np.ix_(idx, idx)]
    return grid, mask


def load_sessions(data_dir: Path):
    """{session_name: [Record, ...]} for every sess_*.bin under data_dir."""
    sessions = {}
    for f in sorted(Path(data_dir).glob("**/sess_*.bin")):
        recs = [r for r in iter_records(f)
                if abs(int(r.t_cap_ms) - int(r.t_tof_ms)) <= MAX_SKEW_MS]
        if recs:
            sessions[f.stem] = recs
    return sessions


# ------------------------------------------------------------- PyTorch ----
try:
    import torch
    from torch.utils.data import Dataset

    class Sec1Dataset(Dataset):
        """mode='bins' (SectorNet fork) or 'depth' (uPyD-Net-lite fork)."""

        def __init__(self, records, mode="bins", augment=False):
            self.records = records
            self.mode = mode
            self.augment = augment

        def __len__(self):
            return len(self.records)

        def _augment(self, img: np.ndarray) -> np.ndarray:
            rng = np.random
            f = img.astype(np.float32)
            f = f * rng.uniform(0.6, 1.4) + rng.uniform(-25, 25)   # exposure
            if rng.rand() < 0.3:                                    # motion blur
                k = rng.choice((3, 5))
                f = np.apply_along_axis(
                    lambda r: np.convolve(r, np.ones(k) / k, mode="same"),
                    rng.choice((0, 1)), f)
            f += rng.normal(0.0, rng.uniform(0, 6), f.shape)        # sensor noise
            return np.clip(f, 0, 255)

        def __getitem__(self, i):
            r = self.records[i]
            img = self._augment(r.img) if self.augment else r.img.astype(np.float32)
            x = torch.from_numpy(np.ascontiguousarray(img / 255.0)).float()
            x = x.unsqueeze(0)  # (1, 96, 96)
            if self.mode == "bins":
                y = torch.from_numpy(bins_from_mm(
                    sector_min_mm(r.tof_mm, r.tof_status)))
                return x, y
            grid, mask = depth_target_12(r.tof_mm, r.tof_status)
            return (x, torch.from_numpy(grid).float(),
                    torch.from_numpy(mask.astype(np.float32)))

except ImportError:  # torch-free usage (eval tooling on the .bin files only)
    pass
