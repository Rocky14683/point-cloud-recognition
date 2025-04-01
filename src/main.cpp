#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <ranges>
#include <cmath>


#include "lidar_parser.hpp"
#include "oxts_parser.hpp"
#include "camera_fuser.hpp"
#include "collection_adapter.hpp"

constexpr size_t frame_wanted = 20;
// footage configuration
constexpr const char* footage_path = "../footage2.rrd";
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


using R_MAT = Eigen::Matrix<float, 3, 3>;
using T_MAT = Eigen::Matrix<float, 3, 1>;
using R_RECT_MAT = Eigen::Matrix<float, 3, 3>;

//rotate -90deg on z axis
const R_MAT rotation_z_n45({
    {cosf(-M_PI_2), -sinf(-M_PI_2), 0.f},
    {sinf(-M_PI_2), cosf(-M_PI_2), 0.f},
    {0.f, 0.f, 1.f}
});

//rotate -90deg on x axis
const R_MAT rotation_x_n45({
    {1.f, 0.f, 0.f},
    {0.f, cosf(-M_PI_2), -sinf(-M_PI_2)},
    {0.f, sinf(-M_PI_2), cosf(-M_PI_2)}
});


int main() {
    // T_02: 5.956621e-02 2.900141e-04 2.577209e-03
    T_MAT cam2_translation({{5.956621e-02}, {2.900141e-04}, {2.577209e-03}});

    // T_03: -4.731050e-01 5.551470e-03 -5.250882e-03
    T_MAT cam3_translation({{-4.731050e-01}, {5.551470e-03}, {-5.250882e-03}});

    // R_02:
    // 9.999758e-01 -5.267463e-03 -4.552439e-03
    // 5.251945e-03 9.999804e-01 -3.413835e-03
    // 4.570332e-03 3.389843e-03 9.999838e-01
    R_MAT cam2_rotation({{9.999758e-01, -5.267463e-03, -4.552439e-03},
                         {5.251945e-03, 9.999804e-01, -3.413835e-03},
                         {4.570332e-03, 3.389843e-03, 9.999838e-01}});

    // R_03:
    // 9.995599e-01 1.699522e-02 -2.431313e-02
    // -1.704422e-02 9.998531e-01 -1.809756e-03
    //  2.427880e-02 2.223358e-03 9.997028e-01
    R_MAT cam3_rotation({{9.995599e-01, 1.699522e-02, -2.431313e-02},
                         {-1.704422e-02, 9.998531e-01, -1.809756e-03},
                         {2.427880e-02, 2.223358e-03, 9.997028e-01}});

    /*
    // K_02:
    //  9.597910e+02 0.000000e+00 6.960217e+02
    //  0.000000e+00 9.569251e+02 2.241806e+02
    //  0.000000e+00 0.000000e+00 1.000000e+00


    //K_03:
    // 9.037596e+02 0.000000e+00 6.957519e+02
    // 0.000000e+00 9.019653e+02 2.242509e+02
    // 0.000000e+00 0.000000e+00 1.000000e+00
    */

    /*
     * R_rect_02:
     * 9.998817e-01 1.511453e-02 -2.841595e-03
     * -1.511724e-02 9.998853e-01 -9.338510e-04
     * 2.827154e-03 9.766976e-04 9.999955e-01
     */

    R_RECT_MAT R_rect_02({
        {9.998817e-01, 1.511453e-02, -2.841595e-03},
        {-1.511724e-02, 9.998853e-01, -9.338510e-04},
        {2.827154e-03, 9.766976e-04, 9.999955e-01}
    });

    /*
     * R_rect_03:
     * 9.998321e-01 -7.193136e-03 1.685599e-02
     * 7.232804e-03 9.999712e-01 -2.293585e-03
     * -1.683901e-02 2.415116e-03 9.998553e-01
     */
    R_RECT_MAT R_rect_03({
        {9.998321e-01, -7.193136e-03, 1.685599e-02},
        {7.232804e-03, 9.999712e-01, -2.293585e-03},
        {-1.683901e-02, 2.415116e-03, 9.998553e-01}
    });


//    auto img_rotate = rerun::Rotation3D(rerun::components::RotationAxisAngle{{1.f, 2.f, 3.f}});

    const auto rec = rerun::RecordingStream("pointcloud");
    rec.save(footage_path).exit_on_failure();

    //    auto out = process_all_frames_from_files_multithreading(50);
    auto lidar_out = lidar_parser::process_all_frames_from_bin_multithreading(frame_wanted);
    std::puts("lidar done");
    auto oxts_out = oxts_parser::process_all_oxts_files(frame_wanted);
    std::puts("oxts done");
    auto camera1_out = camera_fuser::load_images_from_dir_name("image_02", frame_wanted); // 02 is color on camera1
    std::puts("camera1 done");
    auto camera2_out = camera_fuser::load_images_from_dir_name("image_03", frame_wanted); // 03 is color on camera1
    std::puts("camera2 done");

    rec.log("camera1/image", rerun::Pinhole::from_focal_length_and_resolution({9.597910e+02, 9.569251e+02},
                                                                              {constants::cam_w, constants::cam_h}));

    rec.log("camera2/image", rerun::Pinhole::from_focal_length_and_resolution({9.037596e+02, 9.019653e+02},
                                                                              {constants::cam_w, constants::cam_h}));

    R_MAT cam2_new_rotation = R_rect_02 * cam2_rotation;
    R_MAT cam3_new_rotation = R_rect_03 * cam3_rotation;

    T_MAT cam2_new_translation = R_rect_02 * cam2_translation;
    T_MAT cam3_new_translation = R_rect_03 * cam3_translation;



    R_MAT cam2_rot_out = cam2_new_rotation * rotation_z_n45;
    cam2_rot_out = cam2_rot_out * rotation_x_n45;

    R_MAT cam3_rot_out = cam3_new_rotation * rotation_z_n45;
    cam3_rot_out = cam3_rot_out * rotation_x_n45;


    rec.log("camera1/image",
            rerun::Transform3D(
                rerun::Vec3D(cam2_new_translation.data()),
                rerun::Mat3x3(cam2_rot_out.data())));

    rec.log("camera2/image",
            rerun::Transform3D(
                rerun::Vec3D(cam3_new_translation.data()),
                rerun::Mat3x3(cam3_rot_out.data())));

    for (size_t i = 0; i < frame_wanted; i++) {
        rec.log("lidars/points", rerun::Points3D(lidar_out.at(i).positions)
                                     .with_colors(lidar_parser::convert_intensity_to_color(lidar_out.at(i).intensities))
                                     .with_radii(rerun::Radius(0.1f)));

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

        rec.log("camera1/image",
                rerun::Image::from_rgb24(camera1_out.at(i), {static_cast<uint32_t>(constants::cam_w),
                                                             static_cast<uint32_t>(constants::cam_h)}));

        rec.log("camera2/image",
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
