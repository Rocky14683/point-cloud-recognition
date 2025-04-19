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
#include <mdspan>

#include "eigen_formatter.hpp"
#include "helper.hpp"

/*
data input:
{
    voxels=[3, D, H, W],            #voxel grid input
    view_angle=[2,],        #[sin(az), cos(az)]
    centroid=[3,]       #[x, y, z]
    label=scalar,             #class index
    bbox=[5,]              #[w, h, l, sin(yaw), cos(yaw)]
)
 */
namespace features {
enum class Label_classes : int8_t {
    UNKNOWN = 0,
    CAR = 1,
    PEDESTRIAN = 2,
    CYCLIST = 3,
};

std::unordered_map<std::string_view, Label_classes> label_map = {
    {"UNKNOWN", Label_classes::UNKNOWN},
    {"CAR", Label_classes::CAR},
    {"PEDESTRIAN", Label_classes::PEDESTRIAN},
    {"CYCLIST", Label_classes::CYCLIST},
};

// 3 x D x H x W
using VoxelGrid = std::mdspan<const float, std::extents<size_t, 3, 32, 32, 32>>;
//[sin(az), cos(az)]
using Azimuth = std::array<float, 2>;

// [w, h, l, sin(yaw), cos(yaw)]
struct BoundingBox {
        float width;
        float height;
        float length;
        float sin_yaw;
        float cos_yaw;

        [[nodiscard]] std::array<float, 5> to_array() const { return {width, height, length, sin_yaw, cos_yaw}; }

        [[nodiscard]] float* data() const {
            static auto arr = this->to_array();
            return arr.data();
        }
};

struct InputFeatures {
        VoxelGrid voxel_grid;
        Azimuth view_angle;
        pcl::PointXYZ centroid;
        Label_classes label;
        BoundingBox bbox;
};

}; // namespace features

inline const std::filesystem::path training_data_path = "../training_data";

namespace npz {

void store_input_features(size_t id, const features::InputFeatures& input_features) {
    auto path = training_data_path / std::format("cluster{:03}.npz", id);
    auto flat_voxel_mat = helper::flatten_mdspan(input_features.voxel_grid);
    cnpy::npz_save(path, "voxel_grid", flat_voxel_mat.data(), {3, 32, 32, 32}, "w");
    cnpy::npz_save(path, "view_angle", input_features.view_angle.data(), {2}, "a");
    cnpy::npz_save(
        path, "centroid",
        std::to_array({input_features.centroid.x, input_features.centroid.y, input_features.centroid.z}).data(), {3},
        "a");
    cnpy::npz_save(path, "label", &input_features.label, {1}, "a");
    cnpy::npz_save(path, "bbox", input_features.bbox.data(), {5}, "a");
}

features::InputFeatures parse_npz_features(size_t id) {
    auto path = training_data_path / std::format("cluster{:03}.npz", id);
    auto data = cnpy::npz_load(path);

    auto voxel_grid_raw = data["voxel_grid"].as_vec<float>();
    auto azimuth_raw = data["view_angle"].as_vec<float>();
    auto points_raw = data["centroid"].as_vec<float>();
    auto label = data["label"].data<int8_t>();
    auto bbox_raw = data["bbox"].as_vec<float>();

    return {
        .voxel_grid = std::mdspan<const float, std::extents<size_t, 3, 32, 32, 32>>(voxel_grid_raw.data()),
        .view_angle = {azimuth_raw[0], azimuth_raw[1]},
        .centroid = pcl::PointXYZ(points_raw[0], points_raw[1], points_raw[2]),
        .label = static_cast<features::Label_classes>(label[0]),
        .bbox = {.width = bbox_raw[0],
                 .height = bbox_raw[1],
                 .length = bbox_raw[2],
                 .sin_yaw = bbox_raw[3],
                 .cos_yaw = bbox_raw[4]},
    };
}

}; // namespace npz