#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <ranges>

#include "lidar_parser.hpp"
#include "oxts_parser.hpp"

inline constexpr size_t frame_wanted = 50;
// footage configuration
inline constexpr const char* footage_path = "../footage1.rrd";
inline constexpr float s_per_frame = 11.f / 114.f;

inline const rerun::Color red = rerun::Color(255, 0, 0);
inline const rerun::Color green = rerun::Color(0, 255, 0);
inline const rerun::Color blue = rerun::Color(0, 0, 255);

namespace constants {
inline constexpr float accel_scalar = 15.f;
inline constexpr float G_Accel = 9.81f;
}; // namespace constants

int main() {
    const auto rec = rerun::RecordingStream("pointcloud");
    rec.save(footage_path).exit_on_failure();

    //    auto out = process_all_frames_from_files_multithreading(50);
    auto lidar_out = lidar_parser::process_all_frames_from_bin_multithreading(frame_wanted);
    std::puts("lidar done");
    auto oxts_out = oxts_parser::process_all_oxts_files(frame_wanted);
    std::puts("oxts done");

    for (size_t i = 0; i < frame_wanted; i++) {
        rec.log("lidars/points", rerun::Points3D(lidar_out.at(i).positions)
                                     .with_colors(lidar_parser::convert_intensity_to_color(lidar_out.at(i).intensities))
                                     .with_radii(rerun::Radius(0.1f)));

        const auto& oxts_frame = oxts_out.at(i);

        std::vector<std::vector<rerun::Position3D>> accel_axes = {
            {{0.f, 0.f, 0.f}, {-oxts_frame.ax * constants::accel_scalar, 0.f, 0.f}},
            {{0.f, 0.f, 0.f}, {0.f, oxts_frame.ay * constants::accel_scalar, 0.f}},
            {{0.f, 0.f, 0.f}, {0.f, 0.f, (oxts_frame.az - constants::G_Accel) * constants::accel_scalar}}};

        rec.log(
            "oxts/acceleration",
            rerun::LineStrips3D(accel_axes).with_radii(rerun::Radius::ui_points(2)).with_colors({red, green, blue}));

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
