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

struct CalibConfig {
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
         https://www.cvlibs.net/publications/Geiger2013IJRR.pdf
         https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html
         */
        using S_MAT = Matrix<float, 1, 2>;
        using K_MAT = Matrix<float, 3, 3>;
        using D_MAT = Matrix<float, 1, 5>;
        using R_MAT = Matrix<float, 3, 3>;
        using T_MAT = Matrix<float, 3, 1>;
        using S_RECT_MAT = Matrix<float, 1, 2>;
        using R_RECT_MAT = Matrix<float, 3, 3>;
        using P_RECT_MAT = Matrix<float, 3, 4>;

        std::array<T_MAT, 2> T;
        std::array<R_MAT, 2> R;
        std::array<P_RECT_MAT, 4> R_rect;
};

}; // namespace calib_parser