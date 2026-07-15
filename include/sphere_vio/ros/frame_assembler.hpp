#pragma once

#include <array>
#include <cstdint>
#include <deque>

#include <sensor_msgs/Image.h>

#include "sphere_vio/common/types.hpp"

namespace sphere_vio {

class FrameAssembler {
 public:
  static constexpr std::size_t kCameraCount = 4;

  struct Statistics {
    std::uint64_t received_images = 0;
    std::uint64_t completed_frames = 0;
    std::uint64_t dropped_images = 0;
    std::uint64_t timestamp_mismatches = 0;
  };

  FrameAssembler(double maximum_image_time_difference,
                 bool require_exact_image_timestamps);

  bool addImage(CameraId camera_id,
                const sensor_msgs::ImageConstPtr& message,
                MultiCameraFrame* completed_frame);
  const Statistics& statistics() const;
  std::size_t pendingImageCount() const;
  void discardPendingImages();

 private:
  struct QueuedImage {
    Timestamp timestamp = 0.0;
    cv::Mat image;
  };

  bool tryAssemble(MultiCameraFrame* completed_frame);

  double maximum_image_time_difference_;
  bool require_exact_image_timestamps_;
  std::array<std::deque<QueuedImage>, kCameraCount> queues_;
  Statistics statistics_;
};

}  // namespace sphere_vio
