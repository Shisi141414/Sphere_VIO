#include <memory>

#include <gtest/gtest.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "sphere_vio/ros/frame_assembler.hpp"

namespace sphere_vio {
namespace {

sensor_msgs::ImagePtr makeImage(double timestamp, std::uint8_t value) {
  sensor_msgs::ImagePtr image(new sensor_msgs::Image);
  image->header.stamp.fromSec(timestamp);
  image->height = 2;
  image->width = 3;
  image->encoding = sensor_msgs::image_encodings::MONO8;
  image->step = 3;
  image->data.assign(6, value);
  return image;
}

bool addSet(FrameAssembler* assembler, double timestamp,
            MultiCameraFrame* frame) {
  bool completed = false;
  for (CameraId camera = 0; camera < 4; ++camera) {
    completed = assembler->addImage(
        camera, makeImage(timestamp, static_cast<std::uint8_t>(camera + 1)),
        frame);
  }
  return completed;
}

TEST(FrameAssemblerTest, EqualTimestampsCompleteOneFrame) {
  FrameAssembler assembler(0.001, true);
  MultiCameraFrame frame;
  EXPECT_TRUE(addSet(&assembler, 10.0, &frame));
  ASSERT_EQ(4U, frame.images.size());
  EXPECT_DOUBLE_EQ(10.0, frame.timestamp);
  EXPECT_EQ(1U, assembler.statistics().completed_frames);
}

TEST(FrameAssemblerTest, MissingCameraDoesNotCompleteFrame) {
  FrameAssembler assembler(0.001, true);
  MultiCameraFrame frame;
  for (CameraId camera = 0; camera < 3; ++camera) {
    EXPECT_FALSE(assembler.addImage(camera, makeImage(10.0, 1), &frame));
  }
  EXPECT_EQ(0U, assembler.statistics().completed_frames);
  assembler.discardPendingImages();
  EXPECT_EQ(3U, assembler.statistics().dropped_images);
}

TEST(FrameAssemblerTest, ExcessiveDifferenceDropsOldImages) {
  FrameAssembler assembler(0.001, false);
  MultiCameraFrame frame;
  EXPECT_FALSE(assembler.addImage(0, makeImage(10.0, 1), &frame));
  EXPECT_FALSE(assembler.addImage(1, makeImage(10.0, 1), &frame));
  EXPECT_FALSE(assembler.addImage(2, makeImage(10.0, 1), &frame));
  EXPECT_FALSE(assembler.addImage(3, makeImage(10.01, 1), &frame));
  EXPECT_EQ(3U, assembler.statistics().dropped_images);
  EXPECT_EQ(1U, assembler.statistics().timestamp_mismatches);
}

TEST(FrameAssemblerTest, InvalidCameraIdFailsSafely) {
  FrameAssembler assembler(0.001, true);
  MultiCameraFrame frame;
  EXPECT_FALSE(assembler.addImage(4, makeImage(10.0, 1), &frame));
  EXPECT_EQ(0U, assembler.statistics().received_images);
}

TEST(FrameAssemblerTest, ConsecutiveSetsCompleteTwoFrames) {
  FrameAssembler assembler(0.001, true);
  MultiCameraFrame frame;
  EXPECT_TRUE(addSet(&assembler, 10.0, &frame));
  EXPECT_TRUE(addSet(&assembler, 10.05, &frame));
  EXPECT_EQ(2U, assembler.statistics().completed_frames);
}

TEST(FrameAssemblerTest, OutputOwnsImageStorage) {
  FrameAssembler assembler(0.001, true);
  MultiCameraFrame frame;
  for (CameraId camera = 0; camera < 4; ++camera) {
    sensor_msgs::ImagePtr input = makeImage(10.0, 42);
    assembler.addImage(camera, input, &frame);
    input.reset();
  }
  ASSERT_EQ(4U, frame.images.size());
  ASSERT_FALSE(frame.images[0].image.empty());
  EXPECT_EQ(42, frame.images[0].image.at<std::uint8_t>(0, 0));
}

}  // namespace
}  // namespace sphere_vio
