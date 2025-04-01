#!/usr/bin/env python3
import os
from pathlib import Path


def generate_cpp_code(output_file, data_dir, num_frames=112):
    template = """#include <rerun.hpp>
#include <print>
#include "parser.hpp"  // Contains your process_single_frame and other functions

int subprocess() {{
    const auto rec = rerun::RecordingStream("pointcloud");
    rec.spawn().exit_on_failure();

    // Process all frames
{frame_code}
    
    std::println("All frames processed");
    return 0;
}}
"""

    frame_template = """
    {{
        constexpr char DATA_{frame_num}[] = {{
#embed "{data_path}"
        }};
        std::puts("Processing frame {frame_num}");
        auto data = process_single_frame(DATA_{frame_num}, 121000);
        std::println("{frame_num} size: {{}}", data.positions.size());
        rec.set_time_sequence("frame", {frame_num_int});
        rec.log("points", rerun::Points3D()
                          .with_positions(data.positions)
                          .with_colors(convert_intensity_to_color(data.intensities))
                          .with_radii({{rerun::Radius(0.05)}}));
    }}"""

    frame_code = ""
    for i in range(num_frames):
        frame_num = f"{i:03d}"
        data_path = str(Path(data_dir) / f"{i:010d}.txt")  # 10-digit format with leading zeros
        frame_code += frame_template.format(
            frame_num=frame_num,
            frame_num_int=i,
            data_path=data_path.replace('\\', '/')  # Ensure forward slashes for C++
        )

    with open(output_file, 'w') as f:
        f.write(template.format(frame_code=frame_code))


if __name__ == "__main__":
    data_directory = "../datas/velodyne_points/data"  # Update this path
    output_cpp_file = "../include/subprocess.hpp"
    generate_cpp_code(output_cpp_file, data_directory)

    print(f"Generated {output_cpp_file}")
