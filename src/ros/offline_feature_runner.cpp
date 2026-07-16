#include "sphere_vio/ros/offline_feature_runner.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/ros/frame_assembler.hpp"

namespace sphere_vio {
namespace {

struct CameraRunStatistics {
  std::uint64_t frames = 0U;
  std::size_t first_frame_detections = 0U;
  std::uint64_t active_sum = 0U;
  std::size_t minimum_active = std::numeric_limits<std::size_t>::max();
  std::size_t maximum_active = 0U;
  std::uint64_t total_new = 0U;
  std::uint64_t total_tracked = 0U;
  std::uint64_t total_rejected = 0U;
  std::uint64_t track_age_sum = 0U;
  std::uint64_t track_age_samples = 0U;
  std::uint32_t maximum_track_age = 0U;
  double forward_backward_sum = 0.0;
  std::uint64_t forward_backward_samples = 0U;
  double maximum_forward_backward_error = 0.0;
  std::uint64_t model_domain_rejections = 0U;
  double processing_time_sum = 0.0;
  double maximum_processing_time = 0.0;
};

bool topicMatches(const std::string& recorded_topic,
                  const std::string& configured_topic) {
  return recorded_topic == configured_topic ||
         ("/" + recorded_topic) == configured_topic;
}

void addResult(const CameraTrackingResult& result,
               CameraRunStatistics* statistics) {
  if (!statistics) return;
  if (statistics->frames == 0U)
    statistics->first_frame_detections = result.newly_detected;
  ++statistics->frames;
  statistics->active_sum += result.tracks.size();
  statistics->minimum_active =
      std::min(statistics->minimum_active, result.tracks.size());
  statistics->maximum_active =
      std::max(statistics->maximum_active, result.tracks.size());
  statistics->total_new += result.newly_detected;
  statistics->total_tracked += result.successfully_tracked;
  statistics->total_rejected += result.rejected_tracks;
  statistics->maximum_track_age =
      std::max(statistics->maximum_track_age, result.maximum_track_age);
  for (const FeatureTrack& track : result.tracks) {
    statistics->track_age_sum += track.age;
    ++statistics->track_age_samples;
  }
  statistics->forward_backward_sum +=
      result.average_forward_backward_error * result.successfully_tracked;
  statistics->forward_backward_samples += result.successfully_tracked;
  statistics->maximum_forward_backward_error =
      std::max(statistics->maximum_forward_backward_error,
               result.maximum_forward_backward_error);
  statistics->model_domain_rejections += result.model_domain_rejections;
  statistics->processing_time_sum += result.processing_time_seconds;
  statistics->maximum_processing_time =
      std::max(statistics->maximum_processing_time,
               result.processing_time_seconds);
}

}  // namespace

OfflineFeatureRunner::OfflineFeatureRunner(OfflineFeatureRunnerOptions options)
    : options_(std::move(options)) {}

int OfflineFeatureRunner::run() {
  CameraRig camera_rig;
  std::string error;
  if (!loadCameraRigFromYaml(options_.camera_config_file, &camera_rig,
                             &error)) {
    std::cerr << "Cannot load camera rig: " << error << std::endl;
    return 2;
  }

  rosbag::Bag bag;
  try {
    bag.open(options_.bag.bag_path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& exception) {
    std::cerr << "Cannot open bag: " << exception.what() << std::endl;
    return 3;
  }

  std::vector<std::string> topics(options_.bag.camera_topics.begin(),
                                  options_.bag.camera_topics.end());
  // Include IMU only to preserve the offline runner's bag-relative time range.
  // The Phase 4A frontend neither consumes nor integrates these measurements.
  topics.push_back(options_.bag.imu_topic);
  rosbag::View complete_view(bag, rosbag::TopicQuery(topics));
  if (complete_view.size() == 0U) {
    std::cerr << "The bag contains no configured camera messages."
              << std::endl;
    bag.close();
    return 4;
  }
  const ros::Time bag_start = complete_view.getBeginTime();
  const ros::Time bag_end = complete_view.getEndTime();
  const ros::Time processing_start =
      bag_start + ros::Duration(options_.bag.start_time_offset);
  ros::Time processing_end = bag_end;
  if (options_.bag.duration >= 0.0) {
    processing_end = std::min(
        bag_end, processing_start + ros::Duration(options_.bag.duration));
  }
  if (processing_start > bag_end || processing_end < processing_start) {
    std::cerr << "The configured time range does not overlap the bag."
              << std::endl;
    bag.close();
    return 5;
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics), processing_start,
                    processing_end);
  FrameAssembler assembler(options_.bag.maximum_image_time_difference,
                           options_.bag.require_exact_image_timestamps);
  TemporalFrontend frontend(options_.frontend);
  std::array<CameraRunStatistics, 4> run_statistics;
  std::uint64_t completed_frames = 0U;

  for (const rosbag::MessageInstance& instance : view) {
    for (CameraId camera_id = 0U; camera_id < FrameAssembler::kCameraCount;
         ++camera_id) {
      if (!topicMatches(instance.getTopic(),
                        options_.bag.camera_topics[camera_id])) {
        continue;
      }
      const sensor_msgs::ImageConstPtr message =
          instance.instantiate<sensor_msgs::Image>();
      if (!message) break;
      MultiCameraFrame frame;
      if (!assembler.addImage(camera_id, message, &frame)) break;

      MultiCameraTrackingResult tracking_result;
      if (!frontend.processFrame(frame, camera_rig, &tracking_result)) {
        std::cerr << "Frontend rejected completed frame "
                  << (completed_frames + 1U) << " at timestamp "
                  << std::fixed << std::setprecision(9) << frame.timestamp
                  << std::endl;
        bag.close();
        return 6;
      }
      ++completed_frames;
      for (CameraId id = 0U; id < 4U; ++id)
        addResult(tracking_result.cameras[id], &run_statistics[id]);

      if (options_.bag.progress_interval_frames > 0 &&
          completed_frames % static_cast<std::uint64_t>(
                                 options_.bag.progress_interval_frames) ==
              0U) {
        std::cout << "Processed " << completed_frames
                  << " synchronized frames; active tracks C0..C3 = "
                  << tracking_result.cameras[0].tracks.size() << "/"
                  << tracking_result.cameras[1].tracks.size() << "/"
                  << tracking_result.cameras[2].tracks.size() << "/"
                  << tracking_result.cameras[3].tracks.size() << std::endl;
      }
      break;
    }
  }

  assembler.discardPendingImages();
  bag.close();
  std::cout << "\nSphere-VIO temporal frontend summary\n"
            << "  bag: " << options_.bag.bag_path << "\n"
            << "  cameras: " << options_.camera_config_file << "\n"
            << "  completed four-camera frames: " << completed_frames << "\n"
            << "  assembler dropped images: "
            << assembler.statistics().dropped_images << "\n"
            << "  assembler timestamp mismatches: "
            << assembler.statistics().timestamp_mismatches << std::endl;
  std::cout << std::fixed << std::setprecision(6);
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    const CameraRunStatistics& stats = run_statistics[camera_id];
    const double average_active =
        stats.frames == 0U
            ? 0.0
            : static_cast<double>(stats.active_sum) / stats.frames;
    const double average_age =
        stats.track_age_samples == 0U
            ? 0.0
            : static_cast<double>(stats.track_age_sum) /
                  stats.track_age_samples;
    const double average_fb =
        stats.forward_backward_samples == 0U
            ? 0.0
            : stats.forward_backward_sum / stats.forward_backward_samples;
    const double average_time_ms =
        stats.frames == 0U ? 0.0
                           : 1000.0 * stats.processing_time_sum / stats.frames;
    std::cout
        << "  C" << camera_id << ": frames=" << stats.frames
        << ", first detected=" << stats.first_frame_detections
        << ", active avg/min/max=" << average_active << "/"
        << (stats.frames == 0U ? 0U : stats.minimum_active) << "/"
        << stats.maximum_active << ", new=" << stats.total_new
        << ", tracked=" << stats.total_tracked
        << ", rejected=" << stats.total_rejected
        << ", age avg/max=" << average_age << "/"
        << stats.maximum_track_age << ", FB avg/max=" << average_fb << "/"
        << stats.maximum_forward_backward_error
        << ", model-domain rejected=" << stats.model_domain_rejections
        << ", processing ms avg/max=" << average_time_ms << "/"
        << 1000.0 * stats.maximum_processing_time << std::endl;
  }
  return completed_frames == 0U ? 7 : 0;
}

}  // namespace sphere_vio
