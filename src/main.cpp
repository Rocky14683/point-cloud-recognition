#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <ranges>
#include <cmath>

#include "helper.hpp"
#include "lidar_parser.hpp"
#include "oxts_parser.hpp"
#include "camera_fuser.hpp"
#include "collection_adapter.hpp"
#include "ground_plane_fitting.h"
#include "DBSCAN.hpp"

constexpr size_t frame_wanted = 108;
// footage configuration
constexpr const char* footage_path = "../footages/full_rendering.rrd";
constexpr float s_per_frame = 11.f / 114.f;

inline const rerun::Color red = rerun::Color(255, 0, 0);
inline const rerun::Color green = rerun::Color(0, 255, 0);
inline const rerun::Color blue = rerun::Color(0, 0, 255);

namespace constants {

constexpr size_t cam_w = 1242;
constexpr size_t cam_h = 375;

constexpr float velo_scalar = 3.f;
constexpr float accel_scalar = 15.f;
constexpr float G_Accel = 9.81f;
}; // namespace constants

int main() {
    using namespace calib_parser;


    //    auto out = process_all_frames_from_files_multithreading(50);
    //    auto lidar_out = lidar_parser::process_all_frames_from_bin_multithreading(frame_wanted);

    auto lidar_out = lidar_parser::pcl_process_all_frames_from_bin_multithreading(frame_wanted);

    std::puts("lidar parsing done");

    // do ground plane segmentation
    std::vector<pcl::PointCloud<pcl::PointXYZI>> lidars_notground;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> lidars_ground;
    lidars_notground.reserve(lidar_out.size());
    lidars_ground.reserve(lidar_out.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);
    GroundPlaneFit ground_fitting;
    for (auto& frame : lidar_out) {
        notground_points->clear();
        ground_points->clear();

        auto frame_ptr = frame.makeShared(); // Only necessary heap allocation
        ground_fitting.mainLoop(frame_ptr, notground_points, ground_points);

        lidars_notground.emplace_back(std::move(*notground_points));
        lidars_ground.emplace_back(std::move(*ground_points));
    }

    std::puts("lidar segmenting done");
    // use not ground element to do DBSCAN
//    std::vector<arma::Row<size_t>> assignments_per_frame(lidars_notground.size());
//    double epsilon = 0.8;
//    int min_ptrs = 30;
//    dbscan::apply_DBSCAN_multithreading(lidars_notground, epsilon, min_ptrs, assignments_per_frame);
//
//    auto cluster_id_2_pts_all_frames = dbscan::get_cluster_map_multithreading(assignments_per_frame);

    std::puts("lidar clustering done");

    // do voxel grid filtering
    //    auto voxel_notground_elements = lidar_parser::voxel_filter_all(lidars_notground, 0.1f, 0.1f, 0.1f);
    //    std::puts("lidar voxelization done");

    auto oxts_out = oxts_parser::process_all_oxts_files(frame_wanted);
    std::puts("oxts done");

    auto camera1_out = camera_fuser::load_images_from_dir_name("image_02", frame_wanted); // 02 is color on camera1
    std::puts("camera1 done");
    auto camera2_out = camera_fuser::load_images_from_dir_name("image_03", frame_wanted); // 03 is color on camera1
    std::puts("camera2 done");

    const auto rec = rerun::RecordingStream("pointcloud");
    rec.save(footage_path).exit_on_failure();

    // camera frame rotation first
    rec.log("camera", rerun::Transform3D::from_mat3x3(rerun::Mat3x3(cams_rotation.data())));

    rec.log("camera/cam1/image", rerun::Pinhole::from_focal_length_and_resolution(
                                     {9.597910e+02, 9.569251e+02}, {constants::cam_w, constants::cam_h}));

    rec.log("camera/cam2/image", rerun::Pinhole::from_focal_length_and_resolution(
                                     {9.037596e+02, 9.019653e+02}, {constants::cam_w, constants::cam_h}));

    // camera frame translation from camera 0
    auto cam2_transformed = transform_camera(R_02, T_02);
    auto cam3_transformed = transform_camera(R_03, T_03);

    rec.log("camera/cam1/image", rerun::Transform3D::from_translation(rerun::Vec3D(cam2_transformed.data())));

    rec.log("camera/cam2/image", rerun::Transform3D::from_translation(rerun::Vec3D(cam3_transformed.data())));

    for (size_t i = 0; i < frame_wanted; i++) {
        rec.log("lidar/not_ground", rerun::Points3D(lidars_notground.at(i))
                                        .with_colors(lidars_notground.at(i))
                                        .with_radii(rerun::Radius(0.05f)));

//        pcl::PointCloud<pcl::PointXYZI> cluster_points;
//        pcl::PointCloud<pcl::PointXYZI> voxel_points;
//        for (auto& [cluster_id, pt_idxs] : cluster_id_2_pts_all_frames.at(i)) {
//            for (const auto& pt_idx : pt_idxs) { cluster_points.push_back(lidars_notground.at(i).points.at(pt_idx)); }
//            rec.log(std::format("lidar/cluster/{}", cluster_id),
//                    rerun::Points3D(cluster_points).with_radii(rerun::Radius(0.1f)));
//            if (cluster_id != SIZE_MAX) {
//                voxel_points = lidar_parser::voxel_filter(cluster_points, 0.3, 0.3, 1);
//                rec.log(std::format("lidar/voxel/{}", cluster_id),
//                        rerun::Boxes3D::from_centers_and_sizes(rerun::Collection<rerun::Position3D>(voxel_points),
//                                                               {{0.15, 0.15, 0.15}})
//                            .with_colors({{rerun::Color(255, 255, 255)}})
//                            .with_radii(rerun::Radius(0.01f)));
//            }
//            cluster_points.clear();
//            voxel_points.clear();
//        }

        rec.log("lidar/ground",
                rerun::Points3D(lidars_ground.at(i)).with_colors(lidars_ground.at(i)).with_radii(rerun::Radius(0.05f)));

        const auto& oxts_frame = oxts_out.at(i);

        std::vector<std::vector<rerun::Position3D>> velo_axes = {
            {{0.f, 0.f, 0.f}, {oxts_frame.vf * constants::velo_scalar, 0.f, 0.f}},
            {{0.f, 0.f, 0.f}, {0.f, oxts_frame.vl * constants::velo_scalar, 0.f}},
            {{0.f, 0.f, 0.f}, {0.f, 0.f, oxts_frame.vu * constants::velo_scalar}}};

        //        rec.log("oxts/acceleration/x", rerun::Scalar(oxts_frame.ax));
        //        rec.log("oxts/acceleration/y", rerun::Scalar(oxts_frame.ay));
        //        rec.log("oxts/acceleration/z", rerun::Scalar(oxts_frame.az));

        rec.log("oxts/velocity",
                rerun::LineStrips3D(velo_axes).with_radii(rerun::Radius::ui_points(2)).with_colors({red, green, blue}));

        rec.log("camera/cam1/image",
                rerun::Image::from_rgb24(camera1_out.at(i), {static_cast<uint32_t>(constants::cam_w),
                                                             static_cast<uint32_t>(constants::cam_h)}));

        rec.log("camera/cam2/image",
                rerun::Image::from_rgb24(camera2_out.at(i), {static_cast<uint32_t>(constants::cam_w),
                                                             static_cast<uint32_t>(constants::cam_h)}));

        std::println("frame {} logged: ", i);
    }

#if defined(WIN32) || defined(__APPLE__)
    // https://ref.rerun.io/docs/cpp/stable/classrerun_1_1RecordingStream.html#a555a7940a076c93d951de5b139d14918
    // this explains why u need to run this lol
    system(std::format("rerun {}", footage_path).c_str());
#else
    rec.spawn({.executable_path = footage_path}).exit_on_failure();
#endif

    std::puts("done");

    return 0;
}
