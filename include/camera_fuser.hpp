#pragma once
#include <filesystem>
#include <future>
#include <vector>
#include <thread>

#include "collection_adapter.hpp"

namespace camera_fuser {
inline cv::Mat load_image_png(const std::filesystem::path& path) { return cv::imread(path.string(), cv::IMREAD_COLOR); }

std::vector<cv::Mat> load_images_from_dir_name(const std::filesystem::path& image_file_name, int n) {
    std::vector<cv::Mat> images;
    std::vector<std::future<void>> futures;
    images.resize(n);
    for (int i = 0; i < n; i++) {
        futures.emplace_back(std::async(std::launch::async, [&, i]() {
            try {
                std::filesystem::path path = std::format("../bin_datas/{}/data/0000000", image_file_name.string()) +
                                             std::format("{:03}", i) + ".png";
                images.at(i) = load_image_png(path);
            } catch (const std::exception& e) { std::println("Error processing frame {}: {}", i, e.what()); }
        }));
    }

    for (auto& future : futures) { future.wait(); }

    return images;
}

}; // namespace camera_fuser

namespace calib_parser {
using namespace Eigen;

/**
- S_xx: 1x2 size of image xx before rectification {w, h}
- K_xx: 3x3 calibration matrix of camera xx before rectification
- D_xx: 1x5 distortion vector of camera xx before rectification
- R_xx: 3x3 rotation matrix of camera xx (extrinsic)
- T_xx: 3x1 translation vector of camera xx (extrinsic)
- S_rect_xx: 1x2 size of image xx after rectification {w, h}
- R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
- P_rect_xx: 3x4 projection matrix after rectification

Note: When using this dataset you will most likely need to access only
P_rect_xx, as this matrix is valid for the rectified image sequences.
*/

using R_MAT = Eigen::Matrix<float, 3, 3>;
using T_MAT = Eigen::Matrix<float, 3, 1>;
using R_RECT_MAT = Eigen::Matrix<float, 3, 3>;

// rotate -90deg on z axis
const R_MAT
    rotation_z_n45({{cosf(-M_PI_2), -sinf(-M_PI_2), 0.f}, {sinf(-M_PI_2), cosf(-M_PI_2), 0.f}, {0.f, 0.f, 1.f}});

// rotate -90deg on x axis
const R_MAT
    rotation_x_n45({{1.f, 0.f, 0.f}, {0.f, cosf(-M_PI_2), -sinf(-M_PI_2)}, {0.f, sinf(-M_PI_2), cosf(-M_PI_2)}});

const R_MAT cams_rotation = rotation_z_n45 * rotation_x_n45;

// T_02: 5.956621e-02 2.900141e-04 2.577209e-03
T_MAT T_02({{5.956621e-02}, {2.900141e-04}, {2.577209e-03}});

// T_03: -4.731050e-01 5.551470e-03 -5.250882e-03
T_MAT T_03({{-4.731050e-01}, {5.551470e-03}, {-5.250882e-03}});

// R_02:
// 9.999758e-01 -5.267463e-03 -4.552439e-03
// 5.251945e-03 9.999804e-01 -3.413835e-03
// 4.570332e-03 3.389843e-03 9.999838e-01
R_MAT R_02({{9.999758e-01, -5.267463e-03, -4.552439e-03},
            {5.251945e-03, 9.999804e-01, -3.413835e-03},
            {4.570332e-03, 3.389843e-03, 9.999838e-01}});

// R_03:
// 9.995599e-01 1.699522e-02 -2.431313e-02
// -1.704422e-02 9.998531e-01 -1.809756e-03
//  2.427880e-02 2.223358e-03 9.997028e-01
R_MAT R_03({{9.995599e-01, 1.699522e-02, -2.431313e-02},
            {-1.704422e-02, 9.998531e-01, -1.809756e-03},
            {2.427880e-02, 2.223358e-03, 9.997028e-01}});

/*
 * Correct camera transformation formula:
 * X_i = (R_0i)^T * (X_0 - T_0i)
 */
inline Eigen::Matrix<float, 3, 1> transform_camera(const R_MAT& R_0i, const T_MAT& T_0i) {
    static const Eigen::Matrix<float, 3, 1> X_0 = Eigen::Matrix<float, 3, 1>::Zero();
    auto X_i = R_0i.transpose() * (X_0 - T_0i);
    return X_i;
}

}; // namespace calib_parser