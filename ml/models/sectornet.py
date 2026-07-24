"""SectorNet-8 (fork A, research docs 07/09): tiny MobileNet-style classifier.

96x96x1 grayscale in -> 8 sectors x 4 distance bins logits out.
Constraints (doc 09 s1.5): only ESP-NN-accelerated int8 ops — conv2d,
depthwise conv2d, avg-pool/mean, fully-connected, reshape, softmax; ReLU6
activations (quantize well); BatchNorm folds into convs at conversion.

width=1.0 -> "S" (~13k params); width=1.5 -> "M" (~28k params). Both are
well inside the 64 KB flash / 96 KB arena budget.
"""

import torch
import torch.nn as nn

SECTORS = 8
BINS = 4


def _dw_sep(cin, cout, stride):
    return nn.Sequential(
        nn.Conv2d(cin, cin, 3, stride, 1, groups=cin, bias=False),
        nn.BatchNorm2d(cin),
        nn.ReLU6(inplace=True),
        nn.Conv2d(cin, cout, 1, 1, 0, bias=False),
        nn.BatchNorm2d(cout),
        nn.ReLU6(inplace=True),
    )


class SectorNet8(nn.Module):
    def __init__(self, width: float = 1.0):
        super().__init__()
        c = [max(8, int(round(w * width))) for w in (8, 16, 24, 32, 48)]
        self.stem = nn.Sequential(               # 96 -> 48
            nn.Conv2d(1, c[0], 3, 2, 1, bias=False),
            nn.BatchNorm2d(c[0]),
            nn.ReLU6(inplace=True),
        )
        self.body = nn.Sequential(
            _dw_sep(c[0], c[1], 1),              # 48
            _dw_sep(c[1], c[2], 2),              # 48 -> 24
            _dw_sep(c[2], c[2], 1),
            _dw_sep(c[2], c[3], 2),              # 24 -> 12
            _dw_sep(c[3], c[3], 1),
            _dw_sep(c[3], c[4], 2),              # 12 -> 6
        )
        self.pool = nn.AdaptiveAvgPool2d(1)      # -> MEAN op in tflite
        self.head = nn.Sequential(
            nn.Linear(c[4], 64),
            nn.ReLU6(inplace=True),
            nn.Linear(64, SECTORS * BINS),
        )

    def forward(self, x):
        x = self.body(self.stem(x))
        x = self.pool(x).flatten(1)
        return self.head(x).view(-1, SECTORS, BINS)  # logits


def smoothed_targets(bins: torch.Tensor) -> torch.Tensor:
    """Distance-aware label smoothing (doc 09 s2.1): mass on adjacent bins
    only. bins: (B, 8) int64 with -1 = masked. Returns (B, 8, 4) probs."""
    b, s = bins.shape
    t = torch.zeros(b, s, BINS, device=bins.device)
    valid = bins >= 0
    idx = bins.clamp(min=0)
    t.scatter_(2, idx.unsqueeze(-1), 0.8)
    lo = (idx - 1).clamp(min=0)
    hi = (idx + 1).clamp(max=BINS - 1)
    t.scatter_add_(2, lo.unsqueeze(-1), torch.full_like(t[..., :1], 0.1))
    t.scatter_add_(2, hi.unsqueeze(-1), torch.full_like(t[..., :1], 0.1))
    t = t / t.sum(-1, keepdim=True)
    t[~valid] = 0.0
    return t


def sector_loss(logits: torch.Tensor, bins: torch.Tensor,
                near_weight: float = 3.0) -> torch.Tensor:
    """Masked CE against smoothed targets; near-bin (0) rows weighted up —
    near-recall is the safety-critical metric (doc 09 s2.1)."""
    t = smoothed_targets(bins)
    logp = torch.log_softmax(logits, dim=-1)
    ce = -(t * logp).sum(-1)                    # (B, 8)
    w = torch.where(bins == 0, torch.full_like(ce, near_weight),
                    torch.ones_like(ce))
    mask = (bins >= 0).float()
    return (ce * w * mask).sum() / mask.sum().clamp(min=1.0)
