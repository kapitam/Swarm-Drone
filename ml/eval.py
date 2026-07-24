#!/usr/bin/env python3
"""Evaluate an int8 .tflite against SEC1 logs — the fork-decision numbers.

    python eval.py --tflite export/tflite/xxx_full_integer_quant.tflite \
                   --data /path/to/sd_dump

Reports (per research docs 07 s7 / 09 s6): sector bin accuracy, off-by-one
rate, near-bin recall (safety-critical), per-bin confusion. The same metrics
gate deployment (>70% sector agreement) and decide SectorNet vs uPyD-Net.
"""

import argparse
from pathlib import Path

import numpy as np

from dataset import load_sessions, sector_min_mm, bins_from_mm

BIN_EDGES_M = (0.5, 1.0, 2.0)


def make_interpreter(path):
    try:
        from ai_edge_litert.interpreter import Interpreter  # doc 09 pin
    except ImportError:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            raise SystemExit("pip install ai-edge-litert (see requirements.txt)")
    it = Interpreter(model_path=str(path))
    it.allocate_tensors()
    return it


def quantize_input(img_u8, detail):
    scale, zp = detail["quantization"]
    x = img_u8.astype(np.float32) / 255.0
    q = np.round(x / scale + zp).astype(detail["dtype"])
    return q.reshape(detail["shape"])


def infer_bins(it, img):
    inp = it.get_input_details()[0]
    out = it.get_output_details()[0]
    it.set_tensor(inp["index"], quantize_input(img, inp))
    it.invoke()
    y = it.get_tensor(out["index"]).squeeze()
    if y.size == 32:                       # SectorNet logits (8,4)
        return y.reshape(8, 4).argmax(-1)
    # Dense depth map -> dequantize -> column min-pool -> bins.
    scale, zp = out["quantization"]
    d = (y.astype(np.float32) - zp) * scale
    d = d.reshape(d.shape[-2], d.shape[-1])
    h, w = d.shape
    mins = np.full(8, np.inf)
    for s in range(8):
        c0, c1 = (s * w) // 8, max(((s + 1) * w) // 8, (s * w) // 8 + 1)
        band = d[h // 4:(3 * h) // 4, c0:c1]
        pos = band[band > 0.01]
        if pos.size:
            mins[s] = pos.min()
    return np.digitize(mins, BIN_EDGES_M)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tflite", required=True)
    ap.add_argument("--data", required=True)
    args = ap.parse_args()

    it = make_interpreter(args.tflite)
    sessions = load_sessions(Path(args.data))
    conf = np.zeros((4, 4), dtype=np.int64)
    hit = off1 = total = near_tp = near_fn = 0

    for name, recs in sessions.items():
        for r in recs:
            y = bins_from_mm(sector_min_mm(r.tof_mm, r.tof_status))
            p = infer_bins(it, r.img)
            for s in range(8):
                if y[s] < 0:
                    continue
                total += 1
                conf[y[s], p[s]] += 1
                hit += int(p[s] == y[s])
                off1 += int(abs(int(p[s]) - int(y[s])) <= 1)
                if y[s] == 0:
                    near_tp += int(p[s] == 0)
                    near_fn += int(p[s] != 0)
        print(f"[{name}] running acc {hit / max(total, 1):.3f}")

    print("\n=== fork-decision metrics ===")
    print(f"sectors evaluated : {total}")
    print(f"sector accuracy   : {hit / max(total, 1):.3f}  (gate: >0.70)")
    print(f"off-by-one-or-less: {off1 / max(total, 1):.3f}")
    print(f"near-bin recall   : {near_tp / max(near_tp + near_fn, 1):.3f}"
          "  (safety-critical)")
    print("confusion (rows=ToF truth bins 0..3, cols=predicted):")
    print(conf)


if __name__ == "__main__":
    main()
