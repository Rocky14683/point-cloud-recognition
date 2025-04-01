#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "camera_fuser.hpp"

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