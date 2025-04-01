#pragma once

#include <print>
#include <regex>
#include <string>
#include <vector>
#include <tuple>
#include <thread>
#include <future>
#include <filesystem>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <rerun.hpp>

namespace lidar_parser {

struct FrameResult {
        std::vector<rerun::Position3D> positions;
        std::vector<float> intensities;

        void reserve(size_t point_count) {
            // this is extremely bad lol
            positions.reserve(point_count);
            intensities.reserve(point_count);
        }

        void resize(size_t point_count) {
            // this is extremely bad lol
            positions.resize(point_count);
            intensities.resize(point_count);
        }
};

FrameResult process_single_frame(const char* frame_data, size_t approx_points = 50000) {
    FrameResult result;
    result.reserve(approx_points);

    std::istringstream stream(frame_data);
    std::string line;

    while (std::getline(stream, line)) {
        float x, y, z, intensity;
        if (sscanf(line.c_str(), "%f %f %f %f", &x, &y, &z, &intensity) == 4) {
            result.positions.emplace_back(x, y, z);
            result.intensities.push_back(intensity);
        }
    }

    return result;
}

FrameResult process_single_frame(const std::string& frame_data, size_t approx_points = 121000) {
    FrameResult result;
    result.reserve(approx_points);

    std::istringstream stream(frame_data);
    std::string line;

    while (std::getline(stream, line)) {
        float x, y, z, intensity;
        if (sscanf(line.c_str(), "%f %f %f %f", &x, &y, &z, &intensity) == 4) {
            result.positions.emplace_back(x, y, z);
            result.intensities.push_back(intensity);
        }
    }

    return result;
}

template <int limit = -1, size_t N> auto process_all_frames(const std::array<const char*, N>& inputs) {
    static_assert(limit <= N, "limit must be less than or equal to N");

    constexpr auto limit_ = limit == -1 ? N : limit;

    std::array<FrameResult, limit_> datas;
    constexpr size_t pt_per_frame = 121000;

    std::array<std::future<void>, limit_> futures;
    for (size_t i = 0; i < limit_; i++) {
        //        futures.at(i) = (std::async(std::launch::async, [&, i]() {
        std::println("{} begin.", i);
        //            datas[i] = process_single_frame(inputs[i]);

        //        }));
        datas.at(i) = process_single_frame(inputs[i], pt_per_frame);
        std::println("{} done: {}.", i, datas.at(i).positions.size());
    }

    //    for (auto& f : futures) {
    //        f.get();
    //    }

    return datas;
}

inline rerun::Color convert_intensity_to_color(float intensity) {
    int value = static_cast<int>(intensity * 255);
    cv::Mat colorMat(1, 1, CV_8UC1, cv::Scalar(value));
    cv::Mat colorMapped;
    cv::applyColorMap(colorMat, colorMapped, cv::COLORMAP_JET);
    return rerun::Color(colorMapped.at<cv::Vec3b>(0, 0)[0], colorMapped.at<cv::Vec3b>(0, 0)[1],
                        colorMapped.at<cv::Vec3b>(0, 0)[2]);
}

std::vector<rerun::Color> convert_intensity_to_color(const std::vector<float>& intensity_data) {
    std::vector<rerun::Color> colors;
    colors.reserve(intensity_data.size());
    for (const auto& intensity : intensity_data) { colors.push_back(convert_intensity_to_color(intensity)); }
    return colors;
}

std::string read_entire_file(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file.is_open()) { throw std::runtime_error("Failed to open file: " + path.string()); }
    std::stringstream buffer;
    buffer << file.rdbuf(); // Read entire file at once
    return buffer.str();
}

FrameResult parse_frame_bin(const std::filesystem::path& path) noexcept {
    std::FILE* pFile = fopen(path.c_str(), "rb");
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    long size = file.tellg();
    file.seekg(0, std::ios::beg);
    long number_of_points = size / 4 / sizeof(float);
//    std::println("pts number: {}", number_of_points);

    file.seekg(0);
    std::vector<float> rawData(4 * number_of_points);
    file.read(reinterpret_cast<char*>(rawData.data()), size);

    FrameResult result;
    result.resize(number_of_points);

    for (size_t i = 0; i < number_of_points; ++i) {
        const size_t offset = i * 4;
        result.positions[i] = rerun::Position3D {rawData[offset], rawData[offset + 1], rawData[offset + 2]};
        result.intensities[i] = rawData[offset + 3];
    }

    return result;
}

std::vector<FrameResult> process_all_frames_from_files(int n) {
    std::vector<FrameResult> datas;
    datas.reserve(n);

    for (size_t i = 0; i < n; i++) {
        auto input = read_entire_file(
            std::filesystem::path("../datas/velodyne_points/data/0000000" + std::format("{:03}", i) + ".txt"));
        datas.push_back(process_single_frame(input, 121000));
        std::println("frame {} done: {}", i, datas.back().positions.size());
    }

    return datas;
}

std::vector<FrameResult> process_all_frames_from_files_multithreading(int n) {
    std::vector<FrameResult> datas(n); // Pre-allocate exact size
    std::vector<std::future<void>> futures;

    for (size_t i = 0; i < n; i++) {
        futures.emplace_back(std::async(std::launch::async, [&, i]() {
            try {
                auto path =
                    std::filesystem::path("../datas/velodyne_points/data/0000000" + std::format("{:03}", i) + ".txt");
                auto input = read_entire_file(path);

                datas[i] = process_single_frame(input, 121000);
            } catch (const std::exception& e) { std::println("Error processing frame {}: {}", i, e.what()); }
        }));
    }

    for (auto& future : futures) { future.wait(); }

    return datas;
}

std::vector<FrameResult> process_all_frames_from_bin_multithreading(size_t n) {
    std::vector<FrameResult> datas(n); // Pre-allocate exact size
    std::vector<std::future<void>> futures;

    for (size_t i = 0; i < n; i++) {
        futures.emplace_back(std::async(std::launch::async, [&, i]() {
            try {
                auto path = std::filesystem::path("../bin_datas/velodyne_points/data/0000000" +
                                                  std::format("{:03}", i) + ".bin");

                datas[i] = parse_frame_bin(path);
            } catch (const std::exception& e) { std::println("Error processing frame {}: {}", i, e.what()); }
        }));
    }

    for (auto& future : futures) { future.wait(); }

    return datas;
}

};