#include "sphere_vio/ros/d2slam_stitched.hpp"

#include <array>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "gtest/gtest.h"

namespace {

using sphere_vio::splitD2SlamStitchedImage;

TEST(D2SlamStitchedTest, SplitsFourSubImages) {
  cv::Mat stitched(100, 256, CV_8UC1);
  for (int y = 0; y < stitched.rows; ++y) {
    for (int x = 0; x < stitched.cols; ++x) {
      stitched.at<unsigned char>(y, x) =
          static_cast<unsigned char>((x / 64) * 40 + y);
    }
  }

  std::vector<unsigned char> encoded;
  ASSERT_TRUE(cv::imencode(".jpg", stitched, encoded));

  sensor_msgs::CompressedImage compressed;
  compressed.header.stamp = ros::Time(1.25);
  compressed.format = "jpeg";
  compressed.data = encoded;

  std::array<sensor_msgs::ImagePtr, 4> images;
  ASSERT_TRUE(splitD2SlamStitchedImage(compressed, &images));
  for (const sensor_msgs::ImagePtr& image : images) {
    ASSERT_NE(image, nullptr);
    EXPECT_EQ(image->width, 64);
    EXPECT_EQ(image->height, 100);
    EXPECT_EQ(image->encoding, "mono8");
    EXPECT_DOUBLE_EQ(image->header.stamp.toSec(), 1.25);
    EXPECT_EQ(image->data.size(),
              static_cast<std::size_t>(image->width * image->height));
  }
}

TEST(D2SlamStitchedTest, RejectsInvalidWidth) {
  sensor_msgs::CompressedImage compressed;
  compressed.format = "jpeg";
  compressed.data = {1U, 2U, 3U};
  std::array<sensor_msgs::ImagePtr, 4> images;
  EXPECT_FALSE(splitD2SlamStitchedImage(compressed, &images));
}

}  // namespace
