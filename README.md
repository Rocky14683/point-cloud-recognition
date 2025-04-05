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
Raw design: display each cluster * frames in rerun, then use user console input as the label.
- NONE : -1
- CAR : 0
- PEDESTRIAN : 1
- CYCLIST : 2

Store each cluster as a individual ```cluser{i}.npz``` file

With:
- points : (N, 3)
- Orientation of view to the center of cluster: (1, 4)
- label : 1

## TODO:
- [x] Point cloud segementation
- [x] Clustering
- [x] Filters
- [ ] Training data collecting pipeline
- [ ] Implement point cloud recognition (mmdetection3D or Pytorch or VoxelNet)
- [ ] Project camera data to point clouds
