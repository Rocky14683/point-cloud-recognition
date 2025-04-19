# Point Cloud Processing

## Kitti dataset visualization w/ Rerun
<img src="asset/vis.gif" width="550" height="250"/>

## Segmentation
<img src="asset/segmentation.gif" width="550" height="250"/>

## Clustering
<img src="asset/clustering.gif" width="550" height="250"/>

## Voxel Grid filter
<img src="asset/voxel_grid_filter.png" width="550" height=""/>

## Training data collector
Use [labelCloud](https://github.com/ch-sa/labelCloud) to label the raw pointcloud. We will also be looking into the bounding box size which will later on be pass into the neural network to help bounding box size prediction.

After labeling, we will parse the data and pre-process all pointclouds like we did before. Then check if there are any cluster in the labeled bounding box. If yes, then it will be stored into a ```cluser{i}.npz``` file.

**Labels**
- NONE : 0
- CAR : 1
- PEDESTRIAN : 2
- CYCLIST : 3

With:
```
    voxels=[3, D, H, W],            #voxel grid input
    view_angle=[2,],        #[sin(az), cos(az)]
    centroid=[3,]       #[x, y, z]
    label=scalar,             #class index
    bbox=[5,]              #[w, h, l, sin(yaw), cos(yaw)]
```
## Voxel Multitask Net
### Architecture
```
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
```
**input:**
```
    voxels=[3, D, H, W],            #voxel grid input
    view_angle=[2,],        #[sin(az), cos(az)]
    centroid=[3,]       #[x, y, z]
```
**output:**
```
    label=scalar
    bbox=[5,] #[w, h, l, sin(yaw), cos(yaw)]
```


## TODO:
- [x] Point cloud segementation
- [x] Clustering
- [x] Filters
- [ ] Training data collecting pipeline
- [ ] Implement point cloud recognition (mmdetection3D or Pytorch or VoxelNet)
- [ ] Project camera data to point clouds
