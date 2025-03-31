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

def remove_noise(points, nb_neighbors=30, std_ratio=1.0, radius=0.5, max_nn=20, distance_threshold=1.0, min_points=50):
    #Removes noise and small clusters using statistical and radius outlier removal
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    # Statistical outlier removal
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    pcd = pcd.select_by_index(ind)

    # Radius outlier removal
    cl, ind = pcd.remove_radius_outlier(nb_points=max_nn, radius=radius)
    pcd = pcd.select_by_index(ind)

    # Clustering to remove small clusters
    labels = np.array(pcd.cluster_dbscan(eps=distance_threshold, min_points=min_points, print_progress=False))
    max_label = labels.max()

    if max_label >= 0:
        cluster_point_counts = [np.sum(labels == i) for i in range(max_label + 1)]
        filtered_indices = np.isin(labels, np.where(np.array(cluster_point_counts) >= min_points)[0])
        filtered_points = np.asarray(pcd.points)[filtered_indices]
    else:
        filtered_points = np.asarray(pcd.points) #return all points if no clusters are found

    return filtered_points

def cluster_and_bounding_boxes(points, distance_threshold=1, min_points=50):
    #Clusters points, creates bounding boxes, and removes points from small clusters
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    labels = np.array(pcd.cluster_dbscan(eps=distance_threshold, min_points=min_points, print_progress=False))

    max_label = labels.max()
    print(f"PointCloud has {max_label + 1} clusters")

    bounding_boxes = []
    filtered_points = []  # Store points from large enough clusters
    filtered_labels = []  # Store labels for the filtered points

    if max_label >= 0:
        for i in range(max_label + 1):
            cluster_points = points[labels == i]
            if len(cluster_points) > 0:
                pcd_cluster = o3d.geometry.PointCloud()
                pcd_cluster.points = o3d.utility.Vector3dVector(cluster_points)

                # Post bounding box point count check
                points_in_box = np.asarray(pcd_cluster.points)
                if points_in_box.shape[0] > 30:  # Filter clusters with few points
                    bounding_box = pcd_cluster.get_axis_aligned_bounding_box()
                    bounding_boxes.append(bounding_box)

                    # Add points and labels from valid clusters
                    filtered_points.extend(cluster_points)
                    filtered_labels.extend([i] * len(cluster_points))

        # Convert to numpy arrays
        filtered_points = np.array(filtered_points)
        filtered_labels = np.array(filtered_labels)

        return filtered_points, bounding_boxes, filtered_labels
    else:
        return np.array([]), [], np.array([])  # Return empty arrays if no clusters found

def add_fixed_markers(geometries, z_level=-1, grid_size=30, grid_spacing=1, marker_radius=0.5, marker_height=0.5, marker_color=[1, 0, 0], grid_color=[0.7, 0.7, 0.7]):
    #Adds grid lines and an origin marker to the geometries list

    # Add origin marker (cylinder)
    origin_marker = o3d.geometry.TriangleMesh.create_cylinder(radius=marker_radius, height=marker_height)
    origin_marker.paint_uniform_color(marker_color)
    geometries.append(origin_marker)

    # Add grid lines
    grid_lines = create_grid_lines(z_level=z_level, size=grid_size, spacing=grid_spacing, color=grid_color)
    geometries.append(grid_lines)

    return geometries

def create_grid_lines(z_level=-1, size=20, spacing=1, color=[0.7, 0.7, 0.7]):
    #Creates a grid of lines at a specified z-level
    lines = []
    points = []
    line_indices = []

    for i in range(-size, size + 1, spacing):
        # Horizontal lines
        points.append([i, -size, z_level])
        points.append([i, size, z_level])
        line_indices.append([len(points) - 2, len(points) - 1])

        # Vertical lines
        points.append([-size, i, z_level])
        points.append([size, i, z_level])
        line_indices.append([len(points) - 2, len(points) - 1])

    line_set = o3d.geometry.LineSet(
        o3d.utility.Vector3dVector(points),
        o3d.utility.Vector2iVector(line_indices)
    )

    line_set.paint_uniform_color(color)
    return line_set

def visualize_bounding_boxes(points, bounding_boxes, labels):
    #Visualizes the point cloud with bounding boxes and fixed markers
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    colors = np.random.uniform(0, 1, size=(len(np.unique(labels)), 3))
    point_colors = colors[labels]
    pcd.colors = o3d.utility.Vector3dVector(point_colors)
    geometries = [pcd]

    # Add fixed markers (grid lines and cylinder)
    geometries = add_fixed_markers(geometries)

    for i, box in enumerate(bounding_boxes):
        box.color = colors[i % len(colors)]
        lines = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(box)
        lines.paint_uniform_color(box.color)
        geometries.append(lines)

    o3d.visualization.draw_geometries(geometries)
def remove_ground_points(points, height_threshold=-1.4):
    #Removes ground points based on height threshold
    return points[points[:, 2] > height_threshold]

def process_lidar_video(lidar_folder):
    #Processes a folder of lidar .bin files and visualizes them sequentially
    file_list = sorted(glob.glob(os.path.join(lidar_folder, "*.bin")))

    for file_path in file_list:
        print(f"Processing: {file_path}")
        points = load_lidar_bin(file_path)
        filtered_points = remove_noise(points)
        no_ground_points = remove_ground_points(filtered_points)
        final_points, bounding_boxes, final_labels = cluster_and_bounding_boxes(no_ground_points)  # Get filtered points
        if final_points.size > 0:
            visualize_bounding_boxes(final_points, bounding_boxes, final_labels)
        time.sleep(0.1)
        o3d.visualization.destroy_window()

# Example Usage (replace with your folder path)
lidar_folder = r"2011_09_26_drive_0052_sync\velodyne_points\data"

#make the folder if it does not exist.
if not os.path.exists(lidar_folder):
    os.makedirs(lidar_folder)
    print("Please place lidar bin files into the lidar_data folder")
else:
    process_lidar_video(lidar_folder)