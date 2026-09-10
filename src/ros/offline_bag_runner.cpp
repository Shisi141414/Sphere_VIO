#include "sphere_vio/ros/offline_bag_runner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <yaml-cpp/yaml.h>

#include "sphere_vio/imu_interval_buffer.hpp"
#include "sphere_vio/ros/frame_assembler.hpp"
#include "sphere_vio/ros/ros_conversions.hpp"

namespace sphere_vio {
namespace {

struct RunStatistics {
  std::array<std::uint64_t, 4> camera_images{};
  std::uint64_t imu_messages = 0;
  bool has_first_frame = false;
  Timestamp first_frame_time = 0.0;
  Timestamp last_frame_time = 0.0;
  std::uint64_t imu_frame_intervals = 0;
  std::uint64_t imu_per_frame_sum = 0;
  std::size_t minimum_imu_per_frame = 0;
  std::size_t maximum_imu_per_frame = 0;
  std::uint64_t frames_without_sufficient_imu = 0;
  std::vector<Timestamp> completed_frame_times;
};

template <typename T>
void readIfPresent(const YAML::Node& node, const char* key, T* value) {
  if (node && node[key]) *value = node[key].as<T>();
}

std::string formatValue(double value) {
  if (!std::isfinite(value)) return "n/a";
  std::ostringstream output;
  output << std::fixed << std::setprecision(9) << value;
  return output.str();
}

}  // namespace

bool loadOfflineBagConfiguration(const std::string& config_file,
                                 const std::string& bag_path_override,
                                 OfflineBagConfiguration* configuration,
                                 std::string* error) {
  if (!configuration) return false;
  try {
    const YAML::Node root = YAML::LoadFile(config_file);
    readIfPresent(root["bag"], "path", &configuration->bag_path);
    readIfPresent(root["topics"], "camera0", &configuration->camera_topics[0]);
    readIfPresent(root["topics"], "camera1", &configuration->camera_topics[1]);
    readIfPresent(root["topics"], "camera2", &configuration->camera_topics[2]);
    readIfPresent(root["topics"], "camera3", &configuration->camera_topics[3]);
    readIfPresent(root["topics"], "imu", &configuration->imu_topic);
    readIfPresent(root["topics"], "d2slam_stitched_image",
                  &configuration->d2slam_stitched_image_topic);
    readIfPresent(root["topics"], "d2slam_imu",
                  &configuration->d2slam_imu_topic);
    readIfPresent(root["synchronization"], "maximum_image_time_difference",
                  &configuration->maximum_image_time_difference);
    readIfPresent(root["synchronization"], "require_exact_image_timestamps",
                  &configuration->require_exact_image_timestamps);
    readIfPresent(root["offline"], "start_time_offset",
                  &configuration->start_time_offset);
    readIfPresent(root["offline"], "duration", &configuration->duration);
    readIfPresent(root["offline"], "progress_interval_frames",
                  &configuration->progress_interval_frames);
    readIfPresent(root["offline"], "maximum_imu_time_difference",
                  &configuration->maximum_imu_time_difference);
  } catch (const YAML::Exception& exception) {
    if (error) *error = exception.what();
    return false;
  }
  if (!bag_path_override.empty()) configuration->bag_path = bag_path_override;
  if (configuration->bag_path.empty()) {
    if (error) *error = "bag.path is empty (or no bag path override was given)";
    return false;
  }
  for (const std::string& topic : configuration->camera_topics) {
    if (topic.empty()) {
      if (error) *error = "all four camera topics must be configured";
      return false;
    }
  }
  if (configuration->imu_topic.empty()) {
    if (error) *error = "the IMU topic must be configured";
    return false;
  }
  configuration->start_time_offset =
      std::max(0.0, configuration->start_time_offset);
  configuration->progress_interval_frames =
      std::max(0, configuration->progress_interval_frames);
  return true;
}

OfflineBagRunner::OfflineBagRunner(OfflineBagConfiguration configuration)
    : configuration_(std::move(configuration)) {}

int OfflineBagRunner::run() {
  const auto wall_start = std::chrono::steady_clock::now();
  rosbag::Bag bag;
  try {
    bag.open(configuration_.bag_path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& exception) {
    std::cerr << "Cannot open bag: " << exception.what() << std::endl;
    return 2;
  }

  const std::vector<std::string> topics{
      configuration_.camera_topics[0], configuration_.camera_topics[1],
      configuration_.camera_topics[2], configuration_.camera_topics[3],
      configuration_.imu_topic};
  rosbag::View complete_view(bag, rosbag::TopicQuery(topics));
  if (complete_view.size() == 0) {
    std::cerr << "The bag contains no messages on the configured topics."
              << std::endl;
    bag.close();
    return 3;
  }
  const ros::Time bag_start = complete_view.getBeginTime();
  const ros::Time bag_end = complete_view.getEndTime();
  ros::Time processing_start =
      bag_start + ros::Duration(configuration_.start_time_offset);
  ros::Time processing_end = bag_end;
  if (configuration_.duration >= 0.0) {
    processing_end = std::min(
        bag_end, processing_start + ros::Duration(configuration_.duration));
  }
  if (processing_start > bag_end || processing_end < processing_start) {
    std::cerr << "The configured time range does not overlap the bag."
              << std::endl;
    bag.close();
    return 4;
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics), processing_start,
                    processing_end);
  FrameAssembler assembler(configuration_.maximum_image_time_difference,
                           configuration_.require_exact_image_timestamps);
  ImuIntervalBuffer imu_buffer(configuration_.maximum_imu_time_difference);
  RunStatistics run_statistics;
  for (const rosbag::MessageInstance& instance : view) {
    if (instance.getTopic() == configuration_.imu_topic ||
        ("/" + instance.getTopic()) == configuration_.imu_topic) {
      const sensor_msgs::ImuConstPtr message =
          instance.instantiate<sensor_msgs::Imu>();
      if (!message) continue;
      ++run_statistics.imu_messages;
      ImuMeasurement measurement;
      if (!convertImuMessage(*message, &measurement)) continue;
      imu_buffer.add(measurement);
      continue;
    }

    for (CameraId camera_id = 0; camera_id < FrameAssembler::kCameraCount;
         ++camera_id) {
      const std::string& configured_topic =
          configuration_.camera_topics[camera_id];
      if (instance.getTopic() != configured_topic &&
          ("/" + instance.getTopic()) != configured_topic) {
        continue;
      }
      const sensor_msgs::ImageConstPtr message =
          instance.instantiate<sensor_msgs::Image>();
      if (!message) break;
      ++run_statistics.camera_images[camera_id];
      MultiCameraFrame frame;
      if (!assembler.addImage(camera_id, message, &frame)) break;

      if (!run_statistics.has_first_frame) {
        run_statistics.has_first_frame = true;
        run_statistics.first_frame_time = frame.timestamp;
      }
      run_statistics.last_frame_time = frame.timestamp;
      run_statistics.completed_frame_times.push_back(frame.timestamp);
      if (configuration_.progress_interval_frames > 0 &&
          assembler.statistics().completed_frames %
                  static_cast<std::uint64_t>(
                      configuration_.progress_interval_frames) ==
              0) {
        std::cout << "Progress: completed frames="
                  << assembler.statistics().completed_frames
                  << ", stamp=" << std::fixed << std::setprecision(9)
                  << frame.timestamp << std::endl;
      }
      break;
    }
  }

  // rosbag iteration is ordered by bag record time, while interval membership
  // is defined by message header stamps. Defer extraction until every IMU
  // header stamp has been seen so that a late-recorded measurement is not
  // omitted from an otherwise valid image interval.
  if (!run_statistics.completed_frame_times.empty()) {
    imu_buffer.discardThrough(run_statistics.completed_frame_times.front());
    for (std::size_t index = 1;
         index < run_statistics.completed_frame_times.size(); ++index) {
      const std::vector<ImuMeasurement> interval = imu_buffer.extract(
          run_statistics.completed_frame_times[index - 1],
          run_statistics.completed_frame_times[index]);
      const std::size_t count = interval.size();
      if (run_statistics.imu_frame_intervals == 0) {
        run_statistics.minimum_imu_per_frame = count;
        run_statistics.maximum_imu_per_frame = count;
      } else {
        run_statistics.minimum_imu_per_frame =
            std::min(run_statistics.minimum_imu_per_frame, count);
        run_statistics.maximum_imu_per_frame =
            std::max(run_statistics.maximum_imu_per_frame, count);
      }
      run_statistics.imu_per_frame_sum += count;
      ++run_statistics.imu_frame_intervals;
      if (count == 0) ++run_statistics.frames_without_sufficient_imu;
    }
  }

  bag.close();
  const std::size_t residual_imu = imu_buffer.size();
  const std::size_t pending_images = assembler.pendingImageCount();
  assembler.discardPendingImages();
  const FrameAssembler::Statistics& frame_statistics = assembler.statistics();
  const ImuIntervalBuffer::Statistics& imu_statistics =
      imu_buffer.statistics();
  const double average_imu_interval =
      imu_statistics.interval_count == 0
          ? std::numeric_limits<double>::quiet_NaN()
          : imu_statistics.interval_sum / imu_statistics.interval_count;
  const double average_imu_per_frame =
      run_statistics.imu_frame_intervals == 0
          ? std::numeric_limits<double>::quiet_NaN()
          : static_cast<double>(run_statistics.imu_per_frame_sum) /
                run_statistics.imu_frame_intervals;
  const double wall_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start)
          .count();
  const double average_frame_seconds =
      frame_statistics.completed_frames == 0
          ? std::numeric_limits<double>::quiet_NaN()
          : wall_seconds / frame_statistics.completed_frames;

  std::cout << std::fixed << std::setprecision(9)
            << "\nOffline rosbag summary"
            << "\n  bag path: " << configuration_.bag_path
            << "\n  bag start timestamp: " << bag_start.toSec()
            << "\n  bag end timestamp: " << bag_end.toSec()
            << "\n  bag duration: " << (bag_end - bag_start).toSec() << " s";
  for (std::size_t index = 0; index < run_statistics.camera_images.size();
       ++index) {
    std::cout << "\n  camera" << index
              << " image count: " << run_statistics.camera_images[index];
  }
  std::cout << "\n  IMU message count: " << run_statistics.imu_messages
            << "\n  completed four-camera frame count: "
            << frame_statistics.completed_frames
            << "\n  dropped image count: " << frame_statistics.dropped_images
            << "\n  pending image count at bag end: " << pending_images
            << "\n  timestamp mismatch count: "
            << frame_statistics.timestamp_mismatches
            << "\n  first completed frame timestamp: "
            << (run_statistics.has_first_frame
                    ? formatValue(run_statistics.first_frame_time)
                    : "n/a")
            << "\n  last completed frame timestamp: "
            << (run_statistics.has_first_frame
                    ? formatValue(run_statistics.last_frame_time)
                    : "n/a")
            << "\n  minimum IMU interval: "
            << (imu_statistics.interval_count
                    ? formatValue(imu_statistics.minimum_interval)
                    : "n/a")
            << " s\n  maximum IMU interval: "
            << (imu_statistics.interval_count
                    ? formatValue(imu_statistics.maximum_interval)
                    : "n/a")
            << " s\n  average IMU interval: "
            << formatValue(average_imu_interval)
            << " s\n  non-monotonic IMU count: "
            << imu_statistics.non_monotonic_timestamps
            << "\n  duplicate IMU timestamp count: "
            << imu_statistics.duplicate_timestamps
            << "\n  abnormally large IMU interval count: "
            << imu_statistics.large_intervals
            << "\n  minimum IMU measurements per image frame: "
            << (run_statistics.imu_frame_intervals
                    ? std::to_string(run_statistics.minimum_imu_per_frame)
                    : "n/a")
            << "\n  maximum IMU measurements per image frame: "
            << (run_statistics.imu_frame_intervals
                    ? std::to_string(run_statistics.maximum_imu_per_frame)
                    : "n/a")
            << "\n  average IMU measurements per image frame: "
            << formatValue(average_imu_per_frame)
            << "\n  frames without sufficient IMU: "
            << run_statistics.frames_without_sufficient_imu
            << "\n  residual IMU measurements at bag end: " << residual_imu
            << "\n  total processing wall time: " << wall_seconds
            << " s\n  average processing time per completed frame: "
            << formatValue(average_frame_seconds) << " s" << std::endl;
  return 0;
}

}  // namespace sphere_vio
