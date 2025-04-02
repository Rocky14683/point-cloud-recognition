#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>

#include "lidar_parser.hpp"
#include "oxts_parser.hpp"

// Demonstrate some basic assertions.
TEST(HelloTest, BasicAssertions) {
    // Expect two strings not to be equal.
    EXPECT_STRNE("hello", "world");
    // Expect equality.
    EXPECT_EQ(7 * 6, 42);
}


TEST(MultithreadCore, BasicAssertions) {
    std::println("{}", std::thread::hardware_concurrency());
}

TEST(Embed, BasicAssertions) {
    constexpr char DATA_110[] = {
#embed "../datas/velodyne_points/data/0000000000.txt"
    };
    printf("%s", DATA_110);
}

TEST(Parser, BasicAssertions) {
    constexpr char DATA_110[] = {
    #embed "../datas/velodyne_points/data/0000000000.txt"
    };
    auto out = lidar_parser::process_single_frame(DATA_110, 121000);

    for(auto &pos: out.positions) {
        std::println("{} {} {}", pos.x(), pos.y(), pos.z());
    }

    std::println("size: {}", out.positions.size());
}

TEST(EmbedSizeCheck1, BasicAssertions) {
    constexpr char DATA_000[] = {
#embed "../datas/velodyne_points/data/0000000000.txt"
    };
    auto out = lidar_parser::process_single_frame(DATA_000, 121000);
    std::println("size: {}", out.positions.size());
    EXPECT_EQ(out.positions.size(), 120574);
}

TEST(EmbedSizeCheck2, BasicAssertions) {
    constexpr char DATA_000[] = {
#embed "../datas/velodyne_points/data/0000000000.txt"
    };
    auto out = lidar_parser::process_single_frame(DATA_000, 121000);

    std::println("size: {}", out.positions.size());
    EXPECT_EQ(out.positions.size(), 120574);

    constexpr char DATA_001[] = {
#embed "../datas/velodyne_points/data/0000000001.txt"
    };
    auto out1 = lidar_parser::process_single_frame(DATA_001, 121000);

    std::println("size: {}", out1.positions.size());
    EXPECT_EQ(out1.positions.size(), 120831);

    constexpr char DATA_002[] = {
#embed "../datas/velodyne_points/data/0000000002.txt"
    };
    auto out2 = lidar_parser::process_single_frame(DATA_002, 121000);

    std::println("size: {}", out2.positions.size());
    EXPECT_EQ(out2.positions.size(), 121015);
}

TEST(Parser, BasicAssertions2) {
    std::filesystem::path path = "../datas/velodyne_points/data/0000000000.txt";
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + path.string());
    }
    std::stringstream buffer;
    buffer << file.rdbuf();  // Read entire file at once
    auto out = lidar_parser::process_single_frame(buffer.str(), 121000);
    std::println("size: {}", out.positions.size());
    EXPECT_EQ(out.positions.size(), 120574);
}

TEST(OXTS_parser, BasicAssertions) {
    std::filesystem::path path = "../bin_datas/oxts/data/0000000000.txt";
    auto out = oxts_parser::parse_oxts_file(path);
    std::println("lat: {}, lon: {}, alt: {}", out.lat, out.lon, out.alt);
    EXPECT_FLOAT_EQ(out.lat, 49.015003823272);
    EXPECT_FLOAT_EQ(out.lon, 8.4342971002335);
}


TEST(lidar_parse_test, BasicAssertions) {
    auto cloud = lidar_parser::parse_frame_bin("../bin_datas/velodyne_points/data/0000000000.bin");
    std::println("size: {}", cloud.positions.size());
    //    EXPECT_EQ(cloud->points.size(), 121000);
}

TEST(pcl_parse_test, BasicAssertions) {
    auto cloud = pcl::PointCloud<pcl::PointXYZI>();
    lidar_parser::pcl_parse_frame_bin("../bin_datas/velodyne_points/data/0000000000.bin", cloud);
    std::println("size: {}", cloud.points.size());
    EXPECT_EQ(cloud.points.size(), 121015);
}

TEST(pcl_group_parse_test, BasicAssertions) {
    auto clouds = lidar_parser::pcl_process_all_frames_from_bin_multithreading(3);
    for (const auto& cloud : clouds) {
        std::println("size: {}", cloud.points.size());
    }
}

TEST(Cv_imshow, BasicAssertions) {
    auto img = camera_fuser::load_image_png("../bin_datas/image_02/data/0000000000.png");
    cv::imshow("image", img);
    cv::waitKey(5000);
    cv::destroyAllWindows();
}


TEST(Cv_imshow_multiple, BasicAssertions) {
    auto imgs = camera_fuser::load_images_from_dir_name("image_02", 3);
    int i = 0;
    for(const auto& img : imgs) {
        cv::imshow(std::format("image{}", i++), img);
    }
    cv::waitKey(5000);
    cv::destroyAllWindows();
}

TEST(Cv_get_size, BasicAssertions) {
    auto img = camera_fuser::load_image_png("../bin_datas/image_02/data/0000000000.png");
    std::println("w: {}, h: {}", img.cols, img.rows);
}