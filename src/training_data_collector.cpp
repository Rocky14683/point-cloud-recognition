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
#include "nn_helper.hpp"


std::vector<pcl::PointCloud<pcl::PointXYZI>> process_pcl_subroutine(size_t num_process_frame,
                                                                    const std::filesystem::path& path) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>> datas(num_process_frame);
    return {};
}


int main() {
    InputFeatures features = {
        {{1.f, 2.f, 3.f}, {2.f, 5.f, 1.f}, {7.f, 8.f, 9.f}},
        Eigen::Quaternion<float> {0, 1, 0, 0}, // q, x, y, z
        Label_classes::CYCLIST,
    };
    store_input_features(0, features);
    auto out = parse_npz_features(0);
    return 0;
}