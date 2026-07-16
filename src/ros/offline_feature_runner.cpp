#include "sphere_vio/ros/offline_feature_runner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/frontend/orb_descriptor_extractor.hpp"
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

struct DescriptorRunStatistics {
  std::uint64_t frames = 0U;
  std::uint64_t input_tracks = 0U;
  std::uint64_t descriptors = 0U;
  std::uint64_t outside_image_rejections = 0U;
  std::uint64_t patch_boundary_rejections = 0U;
  std::uint64_t orb_discarded = 0U;
};

struct PairRunStatistics {
  CameraId camera_id_1 = 0U;
  CameraId camera_id_2 = 0U;
  std::uint64_t frames = 0U;
  std::uint64_t descriptors_1 = 0U;
  std::uint64_t descriptors_2 = 0U;
  std::uint64_t raw_candidates = 0U;
  std::uint64_t absolute_distance_accepted = 0U;
  std::uint64_t ratio_accepted = 0U;
  std::uint64_t mutual_accepted = 0U;
  std::uint64_t epipolar_accepted = 0U;
  std::uint64_t final_matches = 0U;
  std::size_t minimum_final_matches = std::numeric_limits<std::size_t>::max();
  std::size_t maximum_final_matches = 0U;
  double descriptor_distance_sum = 0.0;
  double maximum_descriptor_distance = 0.0;
  double epipolar_error_sum = 0.0;
  double maximum_epipolar_error = 0.0;
  std::uint64_t rejected_absolute_distance = 0U;
  std::uint64_t rejected_ratio = 0U;
  std::uint64_t rejected_non_mutual = 0U;
  std::uint64_t rejected_epipolar = 0U;
  std::uint64_t rejected_degenerate_geometry = 0U;
  std::uint64_t rejected_duplicate = 0U;
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

void addDescriptorResult(const DescriptorExtractionStatistics& result,
                         DescriptorRunStatistics* statistics) {
  if (!statistics) return;
  ++statistics->frames;
  statistics->input_tracks += result.input_tracks;
  statistics->descriptors += result.accepted;
  statistics->outside_image_rejections += result.outside_image_rejections;
  statistics->patch_boundary_rejections +=
      result.patch_boundary_rejections;
  statistics->orb_discarded += result.orb_discarded;
}

void addPairResult(const CrossCameraPairResult& result,
                   PairRunStatistics* statistics) {
  if (!statistics) return;
  statistics->camera_id_1 = result.camera_id_1;
  statistics->camera_id_2 = result.camera_id_2;
  ++statistics->frames;
  statistics->descriptors_1 += result.descriptors_1;
  statistics->descriptors_2 += result.descriptors_2;
  statistics->raw_candidates += result.raw_candidates;
  statistics->absolute_distance_accepted +=
      result.absolute_distance_accepted;
  statistics->ratio_accepted += result.ratio_accepted;
  statistics->mutual_accepted += result.mutual_accepted;
  statistics->epipolar_accepted += result.epipolar_accepted;
  statistics->final_matches += result.matches.size();
  statistics->minimum_final_matches =
      std::min(statistics->minimum_final_matches, result.matches.size());
  statistics->maximum_final_matches =
      std::max(statistics->maximum_final_matches, result.matches.size());
  for (const CrossCameraMatch& match : result.matches) {
    statistics->descriptor_distance_sum += match.descriptor_distance;
    statistics->maximum_descriptor_distance =
        std::max(statistics->maximum_descriptor_distance,
                 match.descriptor_distance);
    statistics->epipolar_error_sum += match.epipolar_error_maximum;
    statistics->maximum_epipolar_error =
        std::max(statistics->maximum_epipolar_error,
                 match.epipolar_error_maximum);
  }
  statistics->rejected_absolute_distance +=
      result.rejected_absolute_distance;
  statistics->rejected_ratio += result.rejected_ratio;
  statistics->rejected_non_mutual += result.rejected_non_mutual;
  statistics->rejected_epipolar += result.rejected_epipolar;
  statistics->rejected_degenerate_geometry +=
      result.rejected_degenerate_geometry;
  statistics->rejected_duplicate += result.rejected_duplicate;
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
  std::unique_ptr<OrbDescriptorExtractor> descriptor_extractor;
  std::unique_ptr<CrossCameraMatcher> cross_camera_matcher;
  if (options_.cross_camera_matching) {
    descriptor_extractor.reset(new OrbDescriptorExtractor(options_.descriptor));
    cross_camera_matcher.reset(new CrossCameraMatcher(options_.matcher));
  }
  std::array<CameraRunStatistics, 4> run_statistics;
  std::array<DescriptorRunStatistics, 4> descriptor_statistics;
  std::vector<PairRunStatistics> pair_statistics(
      options_.matcher.camera_pairs.size());
  std::uint64_t completed_frames = 0U;
  double cross_camera_processing_time_sum = 0.0;
  double maximum_cross_camera_processing_time = 0.0;

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

      std::vector<CrossCameraPairResult> cross_camera_results;
      if (cross_camera_matcher) {
        const auto cross_camera_start = std::chrono::steady_clock::now();
        std::array<const ImageFrame*, 4> images{{nullptr, nullptr, nullptr,
                                                 nullptr}};
        for (const ImageFrame& image : frame.images) {
          if (image.camera_id >= 4U || images[image.camera_id] != nullptr) {
            std::cerr << "Completed frame has invalid camera ids." << std::endl;
            bag.close();
            return 7;
          }
          images[image.camera_id] = &image;
        }
        std::vector<CameraDescriptorSet> descriptor_sets;
        descriptor_sets.reserve(4U);
        for (CameraId id = 0U; id < 4U; ++id) {
          if (!images[id]) {
            std::cerr << "Completed frame is missing camera " << id
                      << std::endl;
            bag.close();
            return 7;
          }
          CameraDescriptorSet descriptor_set;
          DescriptorExtractionStatistics extraction_statistics;
          if (!descriptor_extractor->extract(
                  images[id]->image, tracking_result.cameras[id],
                  &descriptor_set, &extraction_statistics)) {
            std::cerr << "ORB descriptor extraction failed for camera " << id
                      << " at frame " << completed_frames << std::endl;
            bag.close();
            return 7;
          }
          addDescriptorResult(extraction_statistics,
                              &descriptor_statistics[id]);
          descriptor_sets.push_back(std::move(descriptor_set));
        }
        if (!cross_camera_matcher->matchConfiguredPairs(
                descriptor_sets, camera_rig, &cross_camera_results)) {
          std::cerr << "Cross-camera matching failed at frame "
                    << completed_frames << std::endl;
          bag.close();
          return 7;
        }
        if (cross_camera_results.size() != pair_statistics.size()) {
          std::cerr << "Cross-camera pair result count changed." << std::endl;
          bag.close();
          return 7;
        }
        for (std::size_t index = 0U; index < cross_camera_results.size();
             ++index) {
          addPairResult(cross_camera_results[index], &pair_statistics[index]);
        }
        const double cross_camera_processing_time =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          cross_camera_start)
                .count();
        cross_camera_processing_time_sum += cross_camera_processing_time;
        maximum_cross_camera_processing_time =
            std::max(maximum_cross_camera_processing_time,
                     cross_camera_processing_time);
      }

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
        if (!cross_camera_results.empty()) {
          std::cout << "  final cross-camera candidates";
          for (const CrossCameraPairResult& pair : cross_camera_results) {
            std::cout << " C" << pair.camera_id_1 << "-C" << pair.camera_id_2
                      << "=" << pair.matches.size();
          }
          std::cout << std::endl;
        }
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
  if (options_.cross_camera_matching) {
    std::cout << "\nCross-camera descriptor extraction summary" << std::endl;
    for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
      const DescriptorRunStatistics& stats =
          descriptor_statistics[camera_id];
      const double average_input =
          stats.frames == 0U ? 0.0 :
          static_cast<double>(stats.input_tracks) / stats.frames;
      const double average_descriptors =
          stats.frames == 0U ? 0.0 :
          static_cast<double>(stats.descriptors) / stats.frames;
      std::cout << "  C" << camera_id << ": frames=" << stats.frames
                << ", tracks avg=" << average_input
                << ", descriptors avg=" << average_descriptors
                << ", outside=" << stats.outside_image_rejections
                << ", patch-boundary="
                << stats.patch_boundary_rejections
                << ", ORB-discarded=" << stats.orb_discarded << std::endl;
    }
    std::cout << "\nCross-camera candidate matching summary" << std::endl;
    for (const PairRunStatistics& stats : pair_statistics) {
      const double frames = static_cast<double>(stats.frames);
      const double average_matches =
          stats.frames == 0U ? 0.0 : stats.final_matches / frames;
      const double average_distance =
          stats.final_matches == 0U
              ? 0.0
              : stats.descriptor_distance_sum / stats.final_matches;
      const double average_epipolar =
          stats.final_matches == 0U
              ? 0.0
              : stats.epipolar_error_sum / stats.final_matches;
      std::cout
          << "  C" << stats.camera_id_1 << "-C" << stats.camera_id_2
          << ": frames=" << stats.frames
          << ", descriptors avg="
          << (stats.frames == 0U ? 0.0 : stats.descriptors_1 / frames) << "/"
          << (stats.frames == 0U ? 0.0 : stats.descriptors_2 / frames)
          << ", raw=" << stats.raw_candidates
          << ", absolute=" << stats.absolute_distance_accepted
          << ", ratio=" << stats.ratio_accepted
          << ", mutual=" << stats.mutual_accepted
          << ", epipolar=" << stats.epipolar_accepted
          << ", final avg/min/max=" << average_matches << "/"
          << (stats.frames == 0U ? 0U : stats.minimum_final_matches) << "/"
          << stats.maximum_final_matches
          << ", descriptor distance avg/max=" << average_distance << "/"
          << stats.maximum_descriptor_distance
          << ", epipolar error avg/max rad=" << average_epipolar << "/"
          << stats.maximum_epipolar_error
          << ", rejected absolute/ratio/non-mutual/epipolar/degenerate/duplicate="
          << stats.rejected_absolute_distance << "/"
          << stats.rejected_ratio << "/" << stats.rejected_non_mutual << "/"
          << stats.rejected_epipolar << "/"
          << stats.rejected_degenerate_geometry << "/"
          << stats.rejected_duplicate
          << ", processing ms avg/max="
          << (stats.frames == 0U
                  ? 0.0
                  : 1000.0 * stats.processing_time_sum / frames)
          << "/" << 1000.0 * stats.maximum_processing_time << std::endl;
    }
    std::cout << "  complete descriptor + configured-pair processing ms avg/max="
              << (completed_frames == 0U
                      ? 0.0
                      : 1000.0 * cross_camera_processing_time_sum /
                            completed_frames)
              << "/" << 1000.0 * maximum_cross_camera_processing_time
              << std::endl;
  }
  return completed_frames == 0U ? 7 : 0;
}

}  // namespace sphere_vio
