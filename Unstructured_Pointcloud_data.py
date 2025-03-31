import numpy as np
import open3d as o3d
import os
import glob
import time

def load_lidar_txt(file_path):
    """Loads lidar data from a .txt file."""
    points = np.loadtxt(file_path)
    return points

def visualize_points(points):
    """Visualizes the point cloud directly."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # Use only x, y, z coordinates
    o3d.visualization.draw_geometries([pcd])

def process_lidar_video(lidar_folder):
    """Processes a folder of lidar .txt files and visualizes them sequentially."""
    file_list = sorted(glob.glob(os.path.join(lidar_folder, "*.txt")))

    for file_path in file_list:
        print(f"Processing: {file_path}")
        points = load_lidar_txt(file_path)
        visualize_points(points)
        time.sleep(0.1)  # Adjust delay for desired speed
        o3d.visualization.destroy_window()

# Example Usage (replace with your folder path)
lidar_folder = r"C:\Users\greer\Desktop\LIDAR_DATA\2011_09_26_drive_0001_extract\velodyne_points\data"

#make the folder if it does not exist.
if not os.path.exists(lidar_folder):
    os.makedirs(lidar_folder)
    print("Please place lidar txt files into the lidar_data folder")
else:
    process_lidar_video(lidar_folder)