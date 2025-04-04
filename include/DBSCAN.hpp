#pragma once
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <mlpack/core.hpp>
#include <armadillo>
#include <map>

namespace dbscan {

size_t apply_DBSCAN(const pcl::PointCloud<pcl::PointXYZI>& cloud, double epsilon, int minPts,
                    arma::Row<size_t>& assignments) {
    arma::mat data(3, cloud.points.size());
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        data(0, i) = cloud.points[i].x;
        data(1, i) = cloud.points[i].y;
        data(2, i) = cloud.points[i].z;
    }

    mlpack::DBSCAN<> dbscan(epsilon, minPts);
    return dbscan.Cluster(data, assignments);
}

void apply_DBSCAN_multithreading(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud, double epsilon, int minPts,
                                 std::vector<arma::Row<size_t>>& assignments_per_frame,
                                 std::vector<size_t>& cluster_counts_each_frame) {
    std::vector<std::future<void>> futures;
    futures.reserve(cloud.size());
    cluster_counts_each_frame.resize(cloud.size());

    size_t cluster_size = 0;
    for (size_t i = 0; i < cloud.size(); ++i) {
        futures.emplace_back(std::async(std::launch::async, [&, i]() {
            cluster_size = apply_DBSCAN(cloud[i], epsilon, minPts, assignments_per_frame[i]);
            cluster_counts_each_frame.at(i) = cluster_size;
        }));
    }

    for (auto& future : futures) { future.wait(); }
}

std::map<size_t, std::vector<size_t>> get_cluster_map(const arma::Row<size_t>& assignments) {
    std::map<size_t, std::vector<size_t>> cluster_map; // cluster id : point cloud index
    for (size_t i = 0; i < assignments.n_elem; ++i) {
        cluster_map[assignments[i]].push_back(i); //MAX_SIZE would be noises
    }
    return cluster_map;
}

std::vector<std::map<size_t, std::vector<size_t>>> get_cluster_map_multithreading(
    const std::vector<arma::Row<size_t>>& assignments_per_frame) {
    std::vector<std::map<size_t, std::vector<size_t>>> cluster_map_per_frame(assignments_per_frame.size());
    std::vector<std::future<void>> futures;
    futures.reserve(assignments_per_frame.size());

    for (size_t i = 0; i < assignments_per_frame.size(); ++i) {
        futures.emplace_back(std::async(
            std::launch::async, [&, i]() { cluster_map_per_frame.at(i) = get_cluster_map(assignments_per_frame[i]); }));
    }

    for (auto& future : futures) { future.wait(); }
    return cluster_map_per_frame;
}

}; // namespace dbscan