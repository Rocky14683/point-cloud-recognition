#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <armadillo>

#include "lidar_parser.hpp"
#include "oxts_parser.hpp"
#include "camera_fuser.hpp"
#include "DBSCAN.hpp"
#include "nn_helper.hpp"
#include "helper.hpp"

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

TEST(mlpack_dbscan, BasicAssertions) {
    auto cloud = pcl::PointCloud<pcl::PointXYZI>();
    lidar_parser::pcl_parse_frame_bin("../bin_datas/velodyne_points/data/0000000000.bin", cloud);
    std::println("cloud size: {}", cloud.size());
    arma::Row<size_t> assignments;
    arma::mat data(3, cloud.points.size());
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        data(0, i) = cloud.points[i].x;
        data(1, i) = cloud.points[i].y;
        data(2, i) = cloud.points[i].z;
    }
    double epsilon = 1;
    int minPts = 4;
    mlpack::DBSCAN dbscan(epsilon, minPts);
    auto size = dbscan.Cluster(data, assignments);
    std::println("mesh size: {}", size);
}



TEST(mlpack_dbscan_multiple_frame, BasicAssertions) {
    auto clouds = lidar_parser::pcl_process_all_frames_from_bin_multithreading(3);
    std::vector<arma::Row<size_t>> assignments_per_frame(clouds.size());
    std::vector<size_t> cluster_counts_each_frame(clouds.size());
    double epsilon = 1;
    int minPts = 4;
    dbscan::apply_DBSCAN_multithreading(clouds, epsilon, minPts, assignments_per_frame, cluster_counts_each_frame);
    for (size_t i = 0; i < cluster_counts_each_frame.size(); ++i) {
        std::println("frame {}: cluster size: {}", i, assignments_per_frame.at(i).n_elem);
    }
}


std::vector<float> generate_test_array(size_t N) {
    std::vector<float> arr(N);
    for (size_t i = 0; i < N; ++i)
        arr[i] = static_cast<float>(i);
    return arr;
}

TEST(npz, saving) {
    constexpr size_t C = 3, D = 32, H = 32, W = 32;
    constexpr size_t N = C * D * H * W;  // = 98304

    auto arr = generate_test_array(N);
    std::mdspan<const float, std::extents<size_t, 3, 32, 32, 32>> grid(arr.data());
    features::InputFeatures input_features {
        .voxel_grid = grid,
        .view_angle = {0.5f, 0.0003f},
        .centroid = {50.f, 20.f, 3.23f},
        .label = features::Label_classes::CAR,
        .bbox = {1.502f, 2.222f, 7.0f, 0.1234f, 1.9999f},
    };
    store_input_features(0, input_features);
}

TEST(npz, parsing) {
    auto feature = parse_npz_features(0);
    auto vec = helper::flatten_mdspan(feature.voxel_grid);
    for(const auto& i : vec) {
        std::print("{}", i);
    }
    std::println("");
    std::println("view_angle: {}, {}", feature.view_angle[0], feature.view_angle[1]);
    std::println("centroid: {}, {}, {}", feature.centroid.x, feature.centroid.y, feature.centroid.z);
    std::println("label: {}", static_cast<uint8_t>(feature.label));
    std::println("bbox: {}, {}, {}, {}, {}", feature.bbox.width, feature.bbox.height, feature.bbox.length,
                 feature.bbox.sin_yaw, feature.bbox.cos_yaw);
}