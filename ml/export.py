#!/usr/bin/env python3
"""Export a trained checkpoint to full-int8 .tflite (research doc 09 s3).

Path A: PyTorch -> ONNX -> onnx2tf `-oiqt` (per-channel weights, int8
activations, int8 I/O) with a representative calibration set sampled across
all sessions/lighting.

    python export.py --model sectornet_s --ckpt runs/s1/best.pt \
                     --data /path/to/sd_dump --out export/

Produces export/<model>_full_integer_quant.tflite; validate with eval.py,
then embed with gen_c_array.py.
"""

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch

from dataset import load_sessions
from models import build


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True,
                    choices=["sectornet_s", "sectornet_m", "mupyd"])
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--data", required=True,
                    help="session logs for the calibration tensor")
    ap.add_argument("--out", default="export")
    ap.add_argument("--calib-samples", type=int, default=1000,
                    help="500-2000 recommended (doc 09 s3.2)")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    model, _ = build(args.model)
    model.load_state_dict(torch.load(args.ckpt, map_location="cpu"))
    model.eval()

    # 1) ONNX (opset 13, static shape — MCU-friendly).
    onnx_path = out / f"{args.model}.onnx"
    torch.onnx.export(model, torch.zeros(1, 1, 96, 96), str(onnx_path),
                      input_names=["input"], output_names=["output"],
                      opset_version=13)
    print(f"wrote {onnx_path}")

    # 2) Calibration tensor: NHWC float32 in the network's input scale (0..1),
    #    sampled across ALL sessions (doc 09 s3.2).
    sessions = load_sessions(Path(args.data))
    recs = [r for s in sessions.values() for r in s]
    if not recs:
        raise SystemExit(f"no records under {args.data} for calibration")
    step = max(1, len(recs) // args.calib_samples)
    imgs = np.stack([recs[i].img for i in range(0, len(recs), step)])
    calib = (imgs.astype(np.float32) / 255.0)[..., None]   # NHWC
    calib_path = out / "calib.npy"
    np.save(calib_path, calib)
    print(f"wrote {calib_path} ({len(calib)} samples)")

    # 3) onnx2tf full-integer quantization (doc 09 s3.2 incantation).
    cmd = [
        sys.executable, "-m", "onnx2tf",
        "-i", str(onnx_path), "-o", str(out / "tflite"),
        "-oiqt",                      # output integer-quantized tflite
        "-iqd", "int8", "-oqd", "int8",
        "-cind", "input", str(calib_path), "[[[[0.0]]]]", "[[[[1.0]]]]",
    ]
    print("running:", " ".join(cmd))
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError:
        raise SystemExit("onnx2tf not installed: pip install -r requirements.txt")
    except subprocess.CalledProcessError as e:
        raise SystemExit(
            f"onnx2tf failed ({e.returncode}). Fallback path C (Keras port) "
            "is documented in docs/research/09-v2-ml-pipeline.md s3.1.")

    tfl = sorted((out / "tflite").glob("*full_integer_quant.tflite"))
    if not tfl:
        raise SystemExit("no full_integer_quant.tflite produced — see onnx2tf log")
    print(f"\nOK: {tfl[-1]}")
    print("next: python eval.py --tflite", tfl[-1], "--data", args.data)
    print("then: python gen_c_array.py", tfl[-1], "--version 1")


if __name__ == "__main__":
    main()
