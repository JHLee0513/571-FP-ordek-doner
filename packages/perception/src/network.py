import torch
import torch.nn as nn
import torchvision.models as models

class LocalizationModel(nn.Module):
    def __init__(self):
        super(LocalizationModel, self).__init__()
        self.net = models.resnet18(pretrained=True)
        num_features = self.net.fc.in_features
        # model predicts (x,y,theta)
        self.net.fc = nn.Linear(in_features=num_features, out_features=3, bias=True)

    def forward(self, x):
        return self.net(x)
