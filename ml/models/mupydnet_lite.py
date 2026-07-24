"""uPyD-Net-lite (fork B, research doc 09 s2.2): dense-depth pyramid, reduced.

Lineage: PyDNet / uPyD-Net (Poggi et al.; MCU 'micro' variants, TCSVT 2022),
cut to 2 pyramid levels with ReLU6 (leaky-ReLU is not ESP-NN-accelerated and
quantizes worse, doc 09 s2.2). 96x96x1 in -> 12x12 metric depth (meters) out;
firmware min-pools columns into the same 8-sector contract as fork A.
"""

import torch
import torch.nn as nn

MAX_DEPTH_M = 4.0   # VL53L5CX teacher range; sigmoid output scales to this


def _enc(cin, cout):
    return nn.Sequential(
        nn.Conv2d(cin, cout, 3, 2, 1, bias=False),
        nn.BatchNorm2d(cout),
        nn.ReLU6(inplace=True),
        nn.Conv2d(cout, cout, 3, 1, 1, bias=False),
        nn.BatchNorm2d(cout),
        nn.ReLU6(inplace=True),
    )


class MuPyDNetLite(nn.Module):
    def __init__(self):
        super().__init__()
        self.e1 = _enc(1, 16)     # 96 -> 48
        self.e2 = _enc(16, 32)    # 48 -> 24
        self.e3 = _enc(32, 64)    # 24 -> 12
        # Decoder at 1/8 resolution (12x12), fed by e3 + upsampled context.
        self.d1 = nn.Sequential(
            nn.Conv2d(64, 32, 3, 1, 1, bias=False),
            nn.BatchNorm2d(32),
            nn.ReLU6(inplace=True),
            nn.Conv2d(32, 16, 3, 1, 1, bias=False),
            nn.BatchNorm2d(16),
            nn.ReLU6(inplace=True),
            nn.Conv2d(16, 1, 3, 1, 1),
        )

    def forward(self, x):
        z = self.e3(self.e2(self.e1(x)))
        d = torch.sigmoid(self.d1(z)) * MAX_DEPTH_M
        return d.squeeze(1)       # (B, 12, 12) meters


def berhu_loss(pred: torch.Tensor, target: torch.Tensor,
               mask: torch.Tensor) -> torch.Tensor:
    """Masked reverse-Huber (berHu) — standard for depth regression."""
    diff = (pred - target).abs() * mask
    c = 0.2 * diff.max().clamp(min=1e-6)
    l2 = (diff ** 2 + c ** 2) / (2 * c)
    loss = torch.where(diff <= c, diff, l2)
    return loss.sum() / mask.sum().clamp(min=1.0)
