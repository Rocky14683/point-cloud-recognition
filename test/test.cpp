#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>

#include "parser.hpp"

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
    auto out = process_single_frame(DATA_110, 121000);

    for(auto &pos: out.positions) {
        std::println("{} {} {}", pos.x(), pos.y(), pos.z());
    }

    std::println("size: {}", out.positions.size());
}

TEST(EmbedSizeCheck1, BasicAssertions) {
    constexpr char DATA_000[] = {
#embed "../datas/velodyne_points/data/0000000000.txt"
    };
    auto out = process_single_frame(DATA_000, 121000);
    std::println("size: {}", out.positions.size());
    EXPECT_EQ(out.positions.size(), 120574);
}

TEST(EmbedSizeCheck2, BasicAssertions) {
    constexpr char DATA_000[] = {
#embed "../datas/velodyne_points/data/0000000000.txt"
    };
    auto out = process_single_frame(DATA_000, 121000);

    std::println("size: {}", out.positions.size());
    EXPECT_EQ(out.positions.size(), 120574);

    constexpr char DATA_001[] = {
#embed "../datas/velodyne_points/data/0000000001.txt"
    };
    auto out1 = process_single_frame(DATA_001, 121000);

    std::println("size: {}", out1.positions.size());
    EXPECT_EQ(out1.positions.size(), 120831);

    constexpr char DATA_002[] = {
#embed "../datas/velodyne_points/data/0000000002.txt"
    };
    auto out2 = process_single_frame(DATA_002, 121000);

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
    auto out = process_single_frame(buffer.str(), 121000);
    std::println("size: {}", out.positions.size());
    EXPECT_EQ(out.positions.size(), 120574);
}