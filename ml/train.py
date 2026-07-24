#!/usr/bin/env python3
"""Train either model fork on SEC1 session logs.

    python train.py --data /path/to/sd_dump --model sectornet_s --out runs/s1
    python train.py --data ... --model mupyd --out runs/d1

Both forks are kept alive deliberately (owner decision pending); they train
from the same logs and export through the same pipeline. Split is BY SESSION,
never by shuffled frame (adjacent frames are near-duplicates, doc 09 s5).
Recipe per doc 09 s5: Adam 3e-3 cosine -> 3e-5, batch 64, ~60 epochs base /
~120 fine-tune with early stopping on held-out-session loss.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from dataset import Sec1Dataset, load_sessions, bins_from_mm, sector_min_mm
from models import build
from models.sectornet import sector_loss
from models.mupydnet_lite import berhu_loss, MAX_DEPTH_M

BIN_EDGES_M = (0.5, 1.0, 2.0)


def depth_to_bins(depth12: np.ndarray) -> np.ndarray:
    """(12,12) m -> (8,) bins via column-band min-pool (mirrors firmware)."""
    mins = np.full(8, np.inf)
    for s in range(8):
        c0, c1 = (s * 12) // 8, ((s + 1) * 12) // 8
        band = depth12[3:9, c0:max(c1, c0 + 1)]
        if band.size:
            mins[s] = band.min()
    return np.digitize(mins, BIN_EDGES_M)


@torch.no_grad()
def evaluate(model, loader, mode, device):
    model.eval()
    hit = off1 = total = 0
    near_tp = near_fn = 0
    for batch in loader:
        if mode == "bins":
            x, y = batch
            pred = model(x.to(device)).argmax(-1).cpu().numpy()
            y = y.numpy()
        else:
            x, grid, mask = batch
            d = model(x.to(device)).cpu().numpy()
            pred = np.stack([depth_to_bins(di) for di in d])
            y = np.stack([
                bins_from_mm(np.where(mask[i].numpy()[3:9].any(0),
                                      grid[i].numpy().min(0) * 1000, np.nan))
                if False else _grid_bins(grid[i].numpy(), mask[i].numpy())
                for i in range(len(d))
            ])
        ok = y >= 0
        total += int(ok.sum())
        hit += int(((pred == y) & ok).sum())
        off1 += int(((np.abs(pred - y) <= 1) & ok).sum())
        near_tp += int(((y == 0) & (pred == 0)).sum())
        near_fn += int(((y == 0) & (pred != 0)).sum())
    return {
        "sector_acc": hit / max(total, 1),
        "off_by_one_ok": off1 / max(total, 1),
        "near_recall": near_tp / max(near_tp + near_fn, 1),
        "n_sectors": total,
    }


def _grid_bins(grid12: np.ndarray, mask12: np.ndarray) -> np.ndarray:
    m = np.where(mask12 > 0, grid12, np.nan)
    mins = np.full(8, np.nan)
    for s in range(8):
        c0, c1 = (s * 12) // 8, ((s + 1) * 12) // 8
        band = m[3:9, c0:max(c1, c0 + 1)]
        if np.isfinite(band).any():
            mins[s] = np.nanmin(band) * 1000.0
    return bins_from_mm(mins)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--model", default="sectornet_s",
                    choices=["sectornet_s", "sectornet_m", "mupyd"])
    ap.add_argument("--out", default="runs/latest")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--val-sessions", type=int, default=1,
                    help="hold out the N most recent sessions for validation")
    ap.add_argument("--finetune", default=None,
                    help="checkpoint to fine-tune (per-environment step)")
    args = ap.parse_args()

    sessions = load_sessions(Path(args.data))
    if not sessions:
        raise SystemExit(f"no sess_*.bin files under {args.data}")
    names = sorted(sessions)
    val_names = names[-args.val_sessions:] if len(names) > 1 else names[:1]
    train_recs = [r for n in names if n not in val_names for r in sessions[n]]
    val_recs = [r for n in val_names for r in sessions[n]]
    print(f"sessions: {len(names)} | train {len(train_recs)} recs, "
          f"val {len(val_recs)} recs (held out: {val_names})")

    model, mode = build(args.model)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model.to(device)
    if args.finetune:
        model.load_state_dict(torch.load(args.finetune, map_location=device))
        print(f"fine-tuning from {args.finetune}")

    train_dl = DataLoader(Sec1Dataset(train_recs, mode, augment=True),
                          batch_size=args.batch, shuffle=True, num_workers=2)
    val_dl = DataLoader(Sec1Dataset(val_recs, mode), batch_size=args.batch)

    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(
        opt, T_max=args.epochs, eta_min=3e-5)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    best = 0.0
    patience, bad = 15, 0
    for epoch in range(args.epochs):
        model.train()
        running = 0.0
        for batch in train_dl:
            opt.zero_grad()
            if mode == "bins":
                x, y = batch
                loss = sector_loss(model(x.to(device)), y.to(device))
            else:
                x, grid, mask = batch
                loss = berhu_loss(model(x.to(device)), grid.to(device),
                                  mask.to(device))
            loss.backward()
            opt.step()
            running += float(loss)
        sched.step()
        metrics = evaluate(model, val_dl, mode, device)
        print(f"epoch {epoch:3d} loss {running / max(len(train_dl),1):.4f} "
              f"| val {json.dumps(metrics)}")
        score = metrics["sector_acc"]
        if score > best:
            best, bad = score, 0
            torch.save(model.state_dict(), out / "best.pt")
        else:
            bad += 1
            if bad >= patience:
                print("early stop")
                break

    (out / "metrics.json").write_text(json.dumps(
        {"model": args.model, "best_sector_acc": best}, indent=2))
    print(f"best sector_acc {best:.3f} -> {out / 'best.pt'}")
    print("next: python export.py --model", args.model,
          "--ckpt", str(out / "best.pt"), "--data", args.data)


if __name__ == "__main__":
    main()
