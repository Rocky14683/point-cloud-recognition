import torch
import torch.nn as nn
import torch.nn.functional as F

'''
         [Voxel Grid Input: B x 3 x D x H x W]
                        |
                [3D CNN Encoder]
                        |
             [Global Feature: B x F] ← AdaptiveAvgPool3d
                        |
      +-----------------+------------------+
      |                                    |
[B x F]                         [View Direction (B x 2) + Centroid (B x 3): B x 5]
                                       |
                            [View MLP → B x 64]
                                       |
      +-----------------+------------------+
                        |
         [Fused Feature: B x (F + 64)]
                        |
      +-----------------+------------------+
      |                                    |
[Classifier Head]                   [Bounding Box Head]
      |                                    |
 [B x num_classes]         [B x 5] → [w, h, l, sin(yaw), cos(yaw)]
 
 
 

data input:
{
    voxels=[3, D, H, W],            #voxel grid input
    view_angle=[2,],        #[sin(az), cos(az)]
    centroid=[3,]       #[x, y, z]
    label=scalar,             #class index
    bbox=[5,]              #[w, h, l, sin(yaw), cos(yaw)]
)


data output:
{
    label=scalar
    bbox=[5,] #[w, h, l, sin(yaw), cos(yaw)]
}

'''

NUM_CLASSES: int = 5
TRAINING_MODE: bool = True


class VoxelMultiTaskNet(nn.Module):

    def __init__(self, input_channels=3, num_classes=NUM_CLASSES):
        super().__init__()

        # 3D CNN, this should be similar to processing 2D data. Use CNN to extract the special features of the grids
        self.encoder = nn.Sequential(
            nn.Conv3d(input_channels, 32, kernel_size=3, padding=1),
            nn.BatchNorm3d(32),
            nn.ReLU(),

            nn.Conv3d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm3d(64),
            nn.ReLU(),

            nn.Conv3d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm3d(128),
            nn.ReLU(),

            nn.AdaptiveAvgPool3d((1, 1, 1))
        )
        # output neuron -> 128

        # combining view direction with centroid
        self.view_mlp = nn.Sequential(
            nn.Linear(5, 64),
            nn.ReLU(),
            nn.Linear(64, 128),
            nn.ReLU()
        )
        # output neuron -> 128

        # Classifier head
        self.classifier_head = nn.Sequential(
            nn.Linear(128 + 128, 128),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(128, num_classes)
        )
        # output neuron -> num_classes

        # Bounding box regressor head: [w, h, l, sin(yaw), cos(yaw)]
        self.bbox_head = nn.Sequential(
            nn.Linear(128 + 128, 128),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(128, 5)
        )
        # output neuron -> 5

    def forward(self, voxels, view_azimuth_sincos):
        # voxels: [B, 3, D, H, W]
        # view_azimuth_sin,cos: [B, 2]

        x = self.encoder(voxels)  # → [B, 3, 1, 1, 1]
        x = x.view(x.size(0), -1)  # → [B, 128]

        v = self.view_mlp(view_azimuth_sincos)  # → [B, 64]
        fused = torch.cat([x, v], dim=1)  # → [B, 192]

        class_logits = self.classifier_head(fused)  # → [B, num_classes]
        bbox_pred = self.bbox_head(fused)  # → [B, 6]

        return class_logits, bbox_pred

    def compute_loss(self, class_logits, bbox_pred, label, bbox_target, lambda_bbox=1.0):
        loss_cls = F.cross_entropy(class_logits, label)

        # Optionally normalize yaw angle to unit circle
        norm_pred = F.normalize(bbox_pred[:, 3:5], dim=1)
        norm_target = F.normalize(bbox_target[:, 3:5], dim=1)

        # Bounding box loss (SmoothL1 is often more stable than MSE)
        loss_bbox_dims = F.smooth_l1_loss(bbox_pred[:, 0:3], bbox_target[:, 0:3])  # [w, h, l]
        loss_bbox_yaw = F.mse_loss(norm_pred, norm_target)  # [sin(yaw), cos(yaw)]
        loss_bbox_dist = F.mse_loss(bbox_pred[:, 5], bbox_target[:, 5])  # distance

        loss_bbox = loss_bbox_dims + loss_bbox_yaw + loss_bbox_dist

        # Total loss (weighted sum)
        total_loss = loss_cls + lambda_bbox * loss_bbox
        return total_loss, {
            "loss_cls": loss_cls.item(),
            "loss_bbox_dims": loss_bbox_dims.item(),
            "loss_bbox_yaw": loss_bbox_yaw.item(),
            "loss_bbox_dist": loss_bbox_dist.item(),
            "loss_total": total_loss.item()
        }

    def siou_loss(self):
        pass


'''
    voxels=[3, D, H, W],            #voxel grid input
    view_angle=[2,],        #[sin(az), cos(az)]
    centroid=[3,]       #[x, y, z]
'''


def export_model(model: nn.Module) -> None:
    dummy_voxels = torch.randn(1, 4, 32, 32, 32)  # [B, C, D, H, W]
    dummy_view = torch.randn(1, 2)  # [B, 2]
    dummy_centroid = torch.randn(1, 3)  # [B, 3]
    torch.onnx.export(
        model,
        (dummy_voxels, dummy_view, dummy_centroid),
        "model.onnx",
        input_names=["voxels", "view_angle", "centroid"],
        output_names=["class_logits", "bbox_pred"],
        dynamo=True,
        opset_version=11
    )


def main() -> int:
    model = VoxelMultiTaskNet()
    model.train(TRAINING_MODE)
    export_model(model)

    return 0


if __name__ == '__main__':
    ret = main()
    exit(ret)
