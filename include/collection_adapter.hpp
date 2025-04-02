#pragma once

#include <rerun.hpp>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cassert>
#include <pcl/io/pcd_io.h>
#include "lidar_parser.hpp"

// Adapters so we can log eigen vectors as rerun positions:
template <>
struct rerun::CollectionAdapter<rerun::Position3D, std::vector<Eigen::Vector3f>> {
        /// Borrow for non-temporary.
        Collection<rerun::Position3D> operator()(const std::vector<Eigen::Vector3f>& container) {
            return Collection<rerun::Position3D>::borrow(container.data(), container.size());
        }

        // Do a full copy for temporaries (otherwise the data might be deleted when the temporary is destroyed).
        Collection<rerun::Position3D> operator()(std::vector<Eigen::Vector3f>&& container) {
            std::vector<rerun::Position3D> positions(container.size());
            memcpy(positions.data(), container.data(), container.size() * sizeof(Eigen::Vector3f));
            return Collection<rerun::Position3D>::take_ownership(std::move(positions));
        }
};

// Adapters so we can log an eigen matrix as rerun positions:
template <>
struct rerun::CollectionAdapter<rerun::Position3D, Eigen::Matrix3Xf> {
        // Sanity check that this is binary compatible.
        static_assert(
            sizeof(rerun::Position3D) ==
            sizeof(Eigen::Matrix3Xf::Scalar) * Eigen::Matrix3Xf::RowsAtCompileTime
        );

        /// Borrow for non-temporary.
        Collection<rerun::Position3D> operator()(const Eigen::Matrix3Xf& matrix) {
            static_assert(alignof(rerun::Position3D) <= alignof(Eigen::Matrix3Xf::Scalar));
            return Collection<rerun::Position3D>::borrow(
                // Cast to void because otherwise Rerun will try to do above sanity checks with the wrong type (scalar).
                reinterpret_cast<const void*>(matrix.data()),
                matrix.cols()
            );
        }

        // Do a full copy for temporaries (otherwise the data might be deleted when the temporary is destroyed).
        Collection<rerun::Position3D> operator()(Eigen::Matrix3Xf&& matrix) {
            std::vector<rerun::Position3D> positions(matrix.cols());
            memcpy(positions.data(), matrix.data(), matrix.size() * sizeof(rerun::Position3D));
            return Collection<rerun::Position3D>::take_ownership(std::move(positions));
        }
};

// Adapters so we can borrow an OpenCV image easily into Rerun images without copying:
template <>
struct rerun::CollectionAdapter<uint8_t, cv::Mat> {
        /// Borrow for non-temporary.
        Collection<uint8_t> operator()(const cv::Mat& img) {
            assert(
                "OpenCV matrix was expected have bit depth CV_U8" && CV_MAT_DEPTH(img.type()) == CV_8U
            );

            return Collection<uint8_t>::borrow(img.data, img.total() * img.channels());
        }

        // Do a full copy for temporaries (otherwise the data might be deleted when the temporary is destroyed).
        Collection<uint8_t> operator()(cv::Mat&& img) {
            assert(
                "OpenCV matrix was expected have bit depth CV_U8" && CV_MAT_DEPTH(img.type()) == CV_8U
            );

            std::vector<uint8_t> img_vec(img.total() * img.channels());
            img_vec.assign(img.data, img.data + img.total() * img.channels());
            return Collection<uint8_t>::take_ownership(std::move(img_vec));
        }
};

template <>
struct rerun::CollectionAdapter<rerun::Position3D, pcl::PointCloud<pcl::PointXYZI>> {
        /// Borrow for non-temporary (unsafe - PCL doesn't guarantee contiguous XYZ)
        Collection<rerun::Position3D> operator()(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
            std::vector<rerun::Position3D> positions;
            positions.reserve(cloud.size());
            for (const auto& point : cloud) {
                positions.emplace_back(point.x, point.y, point.z);
            }
            return Collection<rerun::Position3D>::take_ownership(std::move(positions));
        }

        // dont use
        Collection<rerun::Position3D> operator()(pcl::PointCloud<pcl::PointXYZI>&& cloud) {
            assert(false && "dont move.");
            return (*this)(cloud); // same implementation
        }
};

template <>
struct rerun::CollectionAdapter<rerun::Color, pcl::PointCloud<pcl::PointXYZI>> {
        /// Borrow for non-temporary (unsafe - PCL doesn't guarantee contiguous XYZ)
        Collection<rerun::Color> operator()(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
            std::vector<rerun::Color> colors;
            colors.reserve(cloud.size());
            for (const auto& point : cloud) {
                colors.emplace_back(lidar_parser::convert_intensity_to_color(point.intensity));
            }
            return Collection<rerun::Color>::take_ownership(std::move(colors));
        }

        // dont use
        Collection<rerun::Color> operator()(pcl::PointCloud<pcl::PointXYZI>&& cloud) {
            assert(false && "dont move.");
            return (*this)(cloud); // same implementation
        }
};