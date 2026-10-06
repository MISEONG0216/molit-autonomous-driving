import torch
from torch import nn
import torch.nn.functional as F
import pytorch_lightning as pl
import torchvision.models as models


class NIA_SEGNet_module(pl.LightningModule):
    batch_size = 1

    def __init__(self):
        super().__init__()
        # Checkpoint loading restores the full trained state_dict. Avoid an
        # unnecessary ImageNet download at runtime/offline competition.
        self.fcn = models.mobilenet_v2(weights=None)
        self.fcn.classifier = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(self.fcn.last_channel, 5),
        )

    def forward(self, x):
        return self.fcn(x)

    def configure_optimizers(self):
        return torch.optim.Adam(self.parameters(), lr=1e-3)
