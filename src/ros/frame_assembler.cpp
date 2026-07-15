#include "sphere_vio/ros/frame_assembler.hpp"

#include <algorithm>
#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace sphere_vio {

FrameAssembler::FrameAssembler(double maximum_image_time_difference,
                               bool require_exact_image_timestamps)
    : maximum_image_time_difference_(
          std::max(0.0, maximum_image_time_difference)),
      require_exact_image_timestamps_(require_exact_image_timestamps) {}

bool FrameAssembler::addImage(CameraId camera_id,
                              const sensor_msgs::ImageConstPtr& message,
                              MultiCameraFrame* completed_frame) {
  if (camera_id >= kCameraCount || !message || !completed_frame) return false;
  ++statistics_.received_images;
  try {
    const cv_bridge::CvImageConstPtr converted = cv_bridge::toCvShare(message);
    queues_[camera_id].push_back(
        {message->header.stamp.toSec(), converted->image.clone()});
  } catch (const cv_bridge::Exception&) {
    ++statistics_.dropped_images;
    return false;
  }
  return tryAssemble(completed_frame);
}

bool FrameAssembler::tryAssemble(MultiCameraFrame* completed_frame) {
  while (std::all_of(queues_.begin(), queues_.end(),
                     [](const std::deque<QueuedImage>& queue) {
                       return !queue.empty();
                     })) {
    Timestamp minimum = queues_[0].front().timestamp;
    Timestamp maximum = minimum;
    for (const auto& queue : queues_) {
      minimum = std::min(minimum, queue.front().timestamp);
      maximum = std::max(maximum, queue.front().timestamp);
    }
    const double difference = maximum - minimum;
    const bool exact = difference == 0.0;
    if ((!require_exact_image_timestamps_ || exact) &&
        difference <= maximum_image_time_difference_) {
      completed_frame->timestamp = maximum;
      completed_frame->images.clear();
      completed_frame->images.reserve(kCameraCount);
      for (CameraId camera_id = 0; camera_id < kCameraCount; ++camera_id) {
        QueuedImage queued = std::move(queues_[camera_id].front());
        queues_[camera_id].pop_front();
        completed_frame->images.push_back(
            {queued.timestamp, camera_id, std::move(queued.image)});
      }
      ++statistics_.completed_frames;
      return true;
    }

    ++statistics_.timestamp_mismatches;
    for (auto& queue : queues_) {
      if (queue.front().timestamp == minimum) {
        queue.pop_front();
        ++statistics_.dropped_images;
      }
    }
  }
  return false;
}

const FrameAssembler::Statistics& FrameAssembler::statistics() const {
  return statistics_;
}

std::size_t FrameAssembler::pendingImageCount() const {
  std::size_t count = 0;
  for (const auto& queue : queues_) count += queue.size();
  return count;
}

void FrameAssembler::discardPendingImages() {
  for (auto& queue : queues_) {
    statistics_.dropped_images += queue.size();
    queue.clear();
  }
}

}  // namespace sphere_vio
