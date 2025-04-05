#pragma once
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


enum class Label_classes : int8_t {
    NONE = -1,
    CAR = 0,
    PEDESTRIAN = 1,
    CYCLIST = 2,
};

struct InputFeatures {
        std::vector<Eigen::Matrix<float, 3, 1>> points;
        Eigen::Quaternion<float> orientation_of_view;
        Label_classes label;
};

inline const std::filesystem::path training_data_path = "../training_data/";


void store_input_features(size_t id, const InputFeatures& input_features) {
    auto path = std::format("{}cluster{:03}.npz", training_data_path.string(), id);
    cnpy::npz_save(path, "points", input_features.points.data(), {input_features.points.size(), 3}, "w");
    cnpy::npz_save(path, "orientation_of_view", input_features.orientation_of_view.coeffs().data(), {4}, "a");
    cnpy::npz_save(path, "label", &input_features.label, {1}, "a");
}

InputFeatures parse_npz_features(size_t id) {
    auto path = std::format("{}cluster{:03}.npz", training_data_path.string(), id);
    auto data = cnpy::npz_load(path);
    auto points_raw = data["points"].as_vec<float>();
    std::vector<Eigen::Matrix<float, 3, 1>> points(points_raw.size() / 3);
    for (size_t i = 0; i < points_raw.size(); i += 3) {
        points[i / 3] = Eigen::Matrix<float, 3, 1> {points_raw[i], points_raw[i + 1], points_raw[i + 2]};
    }
    auto orientation_of_view_raw = data["orientation_of_view"].as_vec<float>();
    auto label = data["label"].data<int8_t>();
    return {
        .points = points,
        .orientation_of_view = Eigen::Quaternion<float> {orientation_of_view_raw[3],
                                                         orientation_of_view_raw[0],
                                                         orientation_of_view_raw[1],
                                                         orientation_of_view_raw[2]},
        .label = static_cast<Label_classes>(label[0]),
    };
}