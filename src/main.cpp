//#include <print>
//#include <regex>
//#include <string>
//#include <vector>
//#include <tuple>
//#include <thread>
//#include <future>

#include <opencv2/opencv.hpp>
#include <rerun.hpp>


//#include "datas.hpp"
#include "parser.hpp"


constexpr float s_per_frame = 11.f / 114.f;




int main() {
    const auto rec = rerun::RecordingStream("pointcloud");
    rec.spawn().exit_on_failure();

    auto out = process_all_frames_from_files_multithreading(50);

    size_t i = 0;
    for(auto& frame: out) {
        rec.log("points", rerun::Points3D(frame.positions).with_colors(convert_intensity_to_color(frame.intensities)).with_radii(rerun::Radius(0.5f)));
        std::println("frame {} done", i++);
    }

    std::println("done\n");

    return 0;
}
