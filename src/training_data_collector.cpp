#include <print>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <pcl/io/pcd_io.h>
#include <cnpy.h>
#include <vector>
#include <ranges>
#include <rerun.hpp>

#include "nn_helper.hpp"
#include "lidar_parser.hpp"
#include "ground_plane_fitting.h"
#include "DBSCAN.hpp"
#include "collection_adapter.hpp"


inline const rerun::Color red = rerun::Color(255, 0, 0);
inline const rerun::Color green = rerun::Color(0, 255, 0);
inline const rerun::Color blue = rerun::Color(0, 0, 255);


std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>, pcl::PointXYZ>> process_pcl_subroutine(size_t num_process_frame,
                                                                    const std::filesystem::path& path) {
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>, pcl::PointXYZ>> clusters_and_centers;
    auto lidar_out = lidar_parser::pcl_process_all_frames_from_bin_multithreading(num_process_frame, path);
    // do ground plane segmentation
    std::vector<pcl::PointCloud<pcl::PointXYZI>> lidars_notground;
    lidars_notground.reserve(lidar_out.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);
    GroundPlaneFit ground_fitting;
    for (auto& frame : lidar_out) {
        notground_points->clear();
        ground_points->clear();

        auto frame_ptr = frame.makeShared(); // Only necessary heap allocation
        ground_fitting.mainLoop(frame_ptr, notground_points, ground_points);

        lidars_notground.emplace_back(std::move(*notground_points));
    }

    std::vector<arma::Row<size_t>> assignments_per_frame(lidars_notground.size());
    std::vector<size_t> cluster_counts_each_frame(lidars_notground.size());
    double epsilon = 0.8;
    int min_ptrs = 30;
    dbscan::apply_DBSCAN_multithreading(lidars_notground, epsilon, min_ptrs, assignments_per_frame,
                                        cluster_counts_each_frame);

    auto cluster_id_2_pts_all_frames = dbscan::get_cluster_map_multithreading(assignments_per_frame);

    cluster_counts_each_frame.clear();
    assignments_per_frame.clear();


    for(size_t i = 0; i < num_process_frame; i++) {
        pcl::PointCloud<pcl::PointXYZI> cluster_points;
        pcl::PointCloud<pcl::PointXYZI> voxel_points;
        pcl::PointXYZ center;
        for (auto& [cluster_id, pt_idxs] : cluster_id_2_pts_all_frames.at(i)) {
            pcl::CentroidPoint<pcl::PointXYZI> centroid;
            for (const auto& pt_idx : pt_idxs) {
                centroid.add(lidars_notground.at(i).points.at(pt_idx));
                cluster_points.push_back(lidars_notground.at(i).points.at(pt_idx));
            }
            if (cluster_id != SIZE_MAX) {
                voxel_points = lidar_parser::voxel_filter(cluster_points, 0.1, 0.1, 0.1);
                centroid.get(center);
            }
            clusters_and_centers.push_back({voxel_points, center});
            cluster_points.clear();
            voxel_points.clear();
        }
    }

    return clusters_and_centers;
}

constexpr size_t frame_wanted = 1;
int main() {
    auto voxel_clusters = process_pcl_subroutine(frame_wanted, "../bin_datas/velodyne_points/data");

    rerun::RecordingStream rec("cluster_data_collector");
    rec.spawn().exit_on_failure();
    size_t i = 0;
    for(const auto& [cluster, center] : voxel_clusters) {
        rec.log("lidar/voxel",
                rerun::Boxes3D::from_centers_and_sizes(rerun::Collection<rerun::Position3D>(cluster),
                                                       {{0.15, 0.15, 0.15}})
                    .with_colors({{rerun::Color(255, 255, 255)}})
                    .with_radii(rerun::Radius(0.01f)));

        rec.log("lidar/voxel/center",
                rerun::Points3D(rerun::Position3D(center.x, center.y, center.z))
                    .with_colors(red)
                    .with_radii(rerun::Radius(0.1f)));

        auto azimuth = std::atan2(center.y, center.x);
        rec.log("lidar/voxel/azimuth",
                rerun::Arrows3D::from_vectors({{std::cos(azimuth) * 5, std::sin(azimuth) * 5, 0}})
                    .with_colors(green)
                    .with_radii(rerun::Radius(0.1f)));
//        InputFeatures feature;

        //input 0 ~ 2 or enter
        std::string input;
        std::getline(std::cin, input);
    }


    return 0;
}