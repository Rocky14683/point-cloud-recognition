import numpy as np
import open3d as o3d
import os
import glob
import time

def load_lidar_bin(file_path):
    #Loads lidar data from a .bin file
    scan = np.fromfile(file_path, dtype=np.float32)
    points = scan.reshape((-1, 4))  # x, y, z, reflectance
    return points

def visualize_points(points):
    #Visualizes the point cloud directly
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # Use only x, y, z coordinates
    o3d.visualization.draw_geometries([pcd])

def process_lidar_video(lidar_folder):
    #Processes a folder of lidar .bin files and visualizes them sequentially
    file_list = sorted(glob.glob(os.path.join(lidar_folder, "*.bin")))

    for file_path in file_list:
        print(f"Processing: {file_path}")
        points = load_lidar_bin(file_path)
        visualize_points(points)
        time.sleep(0.1)  # Adjust delay for desired speed
        o3d.visualization.destroy_window()

#Folder Path
lidar_folder = r"2011_09_26_drive_0052_sync\velodyne_points\data"
process_lidar_video(lidar_folder)