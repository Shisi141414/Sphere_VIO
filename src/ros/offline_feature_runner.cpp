#include "sphere_vio/ros/offline_feature_runner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <string>
#include <utility>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "sphere_vio/backend/eskf.hpp"
#include "sphere_vio/backend/landmark_map.hpp"
#include "sphere_vio/backend/msckf.hpp"
#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/imu_interval_buffer.hpp"
#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/frontend/orb_descriptor_extractor.hpp"
#include "sphere_vio/frontend/landmark_track_manager.hpp"
#include "sphere_vio/frontend/triangulation_candidate_evaluator.hpp"
#include "sphere_vio/ros/frame_assembler.hpp"
#include "sphere_vio/ros/output_recorder.hpp"
#include "sphere_vio/ros/ros_conversions.hpp"
#include "sphere_vio/ros/ros_output.hpp"

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

constexpr std::size_t kCandidateStatusCount = static_cast<std::size_t>(
    TriangulationCandidateStatus::kCount);

struct CandidateMetricSamples {
  std::vector<double> ray_angle;
  std::vector<double> minimum_depth;
  std::vector<double> maximum_depth;
  std::vector<double> closest_distance;
  std::vector<double> closest_to_baseline_ratio;
  std::vector<double> angular_reprojection_error;
};

struct CandidateRunStatistics {
  CameraId camera_id_1 = 0U;
  CameraId camera_id_2 = 0U;
  std::uint64_t frames = 0U;
  std::uint64_t input_matches = 0U;
  std::uint64_t triangulation_successes = 0U;
  std::uint64_t admitted = 0U;
  std::size_t minimum_admitted = std::numeric_limits<std::size_t>::max();
  std::size_t maximum_admitted = 0U;
  std::array<std::uint64_t, kCandidateStatusCount> status_counts{{}};
  std::uint64_t frames_with_candidates = 0U;
  std::uint64_t consecutive_frames_without_candidates = 0U;
  std::uint64_t longest_frames_without_candidates = 0U;
  double processing_time_sum = 0.0;
  double maximum_processing_time = 0.0;
  CandidateMetricSamples metrics;
};

struct ThresholdSweepRun {
  std::string parameter;
  double value = 0.0;
  TriangulationCandidateOptions options;
  std::vector<CandidateRunStatistics> pairs;
  CandidateRunStatistics global;
};

constexpr std::size_t kAssociationStatusCount = static_cast<std::size_t>(
    LandmarkAssociationStatus::kCount);

struct LandmarkRunStatistics {
  std::uint64_t frames = 0U;
  std::uint64_t admitted_candidate_inputs = 0U;
  std::uint64_t created = 0U;
  std::uint64_t attached_members = 0U;
  std::uint64_t updated_existing = 0U;
  std::uint64_t association_conflicts = 0U;
  std::array<std::uint64_t, kAssociationStatusCount> status_counts{{}};
  std::uint64_t activation_events = 0U;
  std::uint64_t stale_events = 0U;
  std::uint64_t retirement_events = 0U;
  std::size_t maximum_simultaneous_live_tracks = 0U;
  std::uint64_t frames_with_new_tracks = 0U;
  std::uint64_t consecutive_frames_without_new_tracks = 0U;
  std::uint64_t longest_frames_without_new_tracks = 0U;
  std::uint64_t current_c2_c3_candidate_streak = 0U;
  std::uint64_t longest_c2_c3_candidate_streak = 0U;
  std::map<std::pair<CameraId, CameraId>, std::uint64_t> admitted_by_pair;
  std::map<std::tuple<CameraId, FeatureId, CameraId, FeatureId>,
           std::uint64_t> feature_pair_confirmations;
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

void addGeometryMetrics(const TriangulationDiagnostic& diagnostic,
                        CandidateMetricSamples* samples) {
  if (!samples || !diagnostic.triangulation_succeeded) return;
  samples->ray_angle.push_back(diagnostic.ray_angle);
  samples->minimum_depth.push_back(diagnostic.minimum_depth);
  samples->maximum_depth.push_back(diagnostic.maximum_depth);
  samples->closest_distance.push_back(diagnostic.closest_ray_distance);
  samples->closest_to_baseline_ratio.push_back(
      diagnostic.closest_distance_to_baseline_ratio);
  samples->angular_reprojection_error.push_back(
      diagnostic.maximum_angular_reprojection_error);
}

void addCandidateResult(const TriangulationCandidatePairResult& result,
                        bool count_frame,
                        CandidateRunStatistics* statistics) {
  if (!statistics) return;
  if (count_frame) {
    statistics->camera_id_1 = result.camera_id_1;
    statistics->camera_id_2 = result.camera_id_2;
    ++statistics->frames;
  }
  statistics->input_matches += result.input_matches;
  statistics->triangulation_successes += result.triangulation_successes;
  statistics->admitted += result.candidates.size();
  for (const TriangulationDiagnostic& diagnostic : result.diagnostics) {
    const std::size_t status = static_cast<std::size_t>(diagnostic.status);
    if (status < statistics->status_counts.size())
      ++statistics->status_counts[status];
    addGeometryMetrics(diagnostic, &statistics->metrics);
  }
  statistics->processing_time_sum += result.processing_time_seconds;
  statistics->maximum_processing_time =
      std::max(statistics->maximum_processing_time,
               result.processing_time_seconds);
  if (count_frame) {
    statistics->minimum_admitted =
        std::min(statistics->minimum_admitted, result.candidates.size());
    statistics->maximum_admitted =
        std::max(statistics->maximum_admitted, result.candidates.size());
    if (result.candidates.empty()) {
      ++statistics->consecutive_frames_without_candidates;
      statistics->longest_frames_without_candidates = std::max(
          statistics->longest_frames_without_candidates,
          statistics->consecutive_frames_without_candidates);
    } else {
      ++statistics->frames_with_candidates;
      statistics->consecutive_frames_without_candidates = 0U;
    }
  }
}

void finishGlobalCandidateFrame(std::size_t admitted_in_frame,
                                CandidateRunStatistics* statistics) {
  if (!statistics) return;
  ++statistics->frames;
  statistics->minimum_admitted =
      std::min(statistics->minimum_admitted, admitted_in_frame);
  statistics->maximum_admitted =
      std::max(statistics->maximum_admitted, admitted_in_frame);
  if (admitted_in_frame == 0U) {
    ++statistics->consecutive_frames_without_candidates;
    statistics->longest_frames_without_candidates = std::max(
        statistics->longest_frames_without_candidates,
        statistics->consecutive_frames_without_candidates);
  } else {
    ++statistics->frames_with_candidates;
    statistics->consecutive_frames_without_candidates = 0U;
  }
}

std::vector<ThresholdSweepRun> makeThresholdSweeps(
    const TriangulationCandidateOptions& base,
    std::size_t pair_count) {
  std::vector<ThresholdSweepRun> sweeps;
  const auto append = [&](const std::string& parameter, double value,
                          const TriangulationCandidateOptions& options) {
    ThresholdSweepRun sweep;
    sweep.parameter = parameter;
    sweep.value = value;
    sweep.options = options;
    sweep.pairs.resize(pair_count);
    sweeps.push_back(std::move(sweep));
  };
  for (const double value :
       std::array<double, 5>{{0.001, 0.002, 0.003, 0.005, 0.010}}) {
    TriangulationCandidateOptions options = base;
    options.minimum_ray_angle = value;
    append("minimum_ray_angle", value, options);
  }
  for (const double value :
       std::array<double, 4>{{0.005, 0.010, 0.020, 0.050}}) {
    TriangulationCandidateOptions options = base;
    options.maximum_closest_ray_distance = value;
    append("maximum_closest_ray_distance", value, options);
  }
  for (const double value :
       std::array<double, 4>{{0.001, 0.002, 0.003, 0.005}}) {
    TriangulationCandidateOptions options = base;
    options.maximum_angular_reprojection_error = value;
    append("maximum_angular_reprojection_error", value, options);
  }
  return sweeps;
}

TriangulationCandidatePairResult readmitPair(
    const TriangulationCandidatePairResult& geometry,
    const TriangulationCandidateOptions& options) {
  TriangulationCandidatePairResult result;
  result.camera_id_1 = geometry.camera_id_1;
  result.camera_id_2 = geometry.camera_id_2;
  result.input_matches = geometry.input_matches;
  result.triangulation_successes = geometry.triangulation_successes;
  result.diagnostics = geometry.diagnostics;
  for (TriangulationDiagnostic& diagnostic : result.diagnostics) {
    if (!applyTriangulationCandidateAdmission(options, &diagnostic)) continue;
    if (diagnostic.admitted) {
      TriangulationCandidate candidate;
      candidate.match = diagnostic.match;
      candidate.triangulation = diagnostic.triangulation;
      candidate.point_b = diagnostic.point_b;
      candidate.depth_1 = diagnostic.depth_1;
      candidate.depth_2 = diagnostic.depth_2;
      candidate.minimum_depth = diagnostic.minimum_depth;
      candidate.maximum_depth = diagnostic.maximum_depth;
      candidate.relative_depth_difference =
          diagnostic.relative_depth_difference;
      candidate.closest_distance_to_baseline_ratio =
          diagnostic.closest_distance_to_baseline_ratio;
      result.candidates.push_back(std::move(candidate));
    }
  }
  return result;
}

void printQuantiles(const char* name, const std::vector<double>& samples,
                    const std::string& indent) {
  DeterministicQuantiles quantiles;
  if (!computeDeterministicQuantiles(samples, &quantiles) ||
      !quantiles.valid) {
    std::cout << indent << name << " quantiles: count=0" << std::endl;
    return;
  }
  std::cout << indent << name
            << " quantiles count/min/p10/median/p90/p95/max="
            << quantiles.count << "/" << quantiles.minimum << "/"
            << quantiles.p10 << "/" << quantiles.median << "/"
            << quantiles.p90 << "/" << quantiles.p95 << "/"
            << quantiles.maximum << std::endl;
}

void printCandidateStatistics(const CandidateRunStatistics& statistics,
                              const std::string& label,
                              const std::string& indent,
                              bool include_timing) {
  const double frames = static_cast<double>(statistics.frames);
  std::cout << indent << label << ": frames=" << statistics.frames
            << ", input=" << statistics.input_matches
            << ", triangulation successes="
            << statistics.triangulation_successes
            << ", admitted=" << statistics.admitted
            << ", admitted avg/min/max="
            << (statistics.frames == 0U ? 0.0
                                        : statistics.admitted / frames)
            << "/"
            << (statistics.frames == 0U ? 0U
                                        : statistics.minimum_admitted)
            << "/" << statistics.maximum_admitted
            << ", frames with candidates="
            << statistics.frames_with_candidates
            << ", longest consecutive frames without candidates="
            << statistics.longest_frames_without_candidates;
  if (include_timing) {
    std::cout << ", evaluator ms avg/max="
              << (statistics.frames == 0U
                      ? 0.0
                      : 1000.0 * statistics.processing_time_sum / frames)
              << "/" << 1000.0 * statistics.maximum_processing_time;
  }
  std::cout << std::endl << indent << "  status counts";
  for (std::size_t index = 0U; index < statistics.status_counts.size();
       ++index) {
    std::cout << " "
              << triangulationCandidateStatusName(
                     static_cast<TriangulationCandidateStatus>(index))
              << "=" << statistics.status_counts[index];
  }
  std::cout << std::endl;
  const std::string metric_indent = indent + "  ";
  printQuantiles("ray angle rad", statistics.metrics.ray_angle,
                 metric_indent);
  printQuantiles("minimum depth m", statistics.metrics.minimum_depth,
                 metric_indent);
  printQuantiles("maximum depth m", statistics.metrics.maximum_depth,
                 metric_indent);
  printQuantiles("closest distance m",
                 statistics.metrics.closest_distance, metric_indent);
  printQuantiles("closest/baseline ratio",
                 statistics.metrics.closest_to_baseline_ratio,
                 metric_indent);
  printQuantiles("maximum angular reprojection rad",
                 statistics.metrics.angular_reprojection_error,
                 metric_indent);
}

void addLandmarkFrameResult(
    const std::vector<TriangulationCandidatePairResult>& candidates,
    const LandmarkTrackFrameResult& result,
    const LandmarkTrackManager& manager,
    LandmarkRunStatistics* statistics) {
  if (!statistics) return;
  ++statistics->frames;
  bool has_c2_c3_candidate = false;
  for (const TriangulationCandidatePairResult& pair : candidates) {
    for (const TriangulationDiagnostic& diagnostic : pair.diagnostics) {
      if (!diagnostic.admitted) continue;
      ++statistics->admitted_candidate_inputs;
      const auto camera_pair = std::make_pair(
          diagnostic.match.camera_id_1, diagnostic.match.camera_id_2);
      ++statistics->admitted_by_pair[camera_pair];
      if (camera_pair == std::make_pair(2U, 3U))
        has_c2_c3_candidate = true;
      ++statistics->feature_pair_confirmations[std::make_tuple(
          diagnostic.match.camera_id_1, diagnostic.match.feature_id_1,
          diagnostic.match.camera_id_2, diagnostic.match.feature_id_2)];
    }
  }
  if (has_c2_c3_candidate) {
    ++statistics->current_c2_c3_candidate_streak;
    statistics->longest_c2_c3_candidate_streak = std::max(
        statistics->longest_c2_c3_candidate_streak,
        statistics->current_c2_c3_candidate_streak);
  } else {
    statistics->current_c2_c3_candidate_streak = 0U;
  }

  statistics->created += result.created.size();
  statistics->activation_events += result.activated.size();
  statistics->stale_events += result.marked_stale.size();
  statistics->retirement_events += result.retired.size();
  statistics->association_conflicts += result.association_conflicts;
  for (const LandmarkAssociationResult& association : result.associations) {
    const std::size_t status = static_cast<std::size_t>(association.status);
    if (status < statistics->status_counts.size())
      ++statistics->status_counts[status];
    if (association.status ==
            LandmarkAssociationStatus::kAttachedFirstFeature ||
        association.status ==
            LandmarkAssociationStatus::kAttachedSecondFeature) {
      ++statistics->attached_members;
    } else if (association.status ==
               LandmarkAssociationStatus::kUpdatedExisting) {
      ++statistics->updated_existing;
    }
  }
  if (result.created.empty()) {
    ++statistics->consecutive_frames_without_new_tracks;
    statistics->longest_frames_without_new_tracks = std::max(
        statistics->longest_frames_without_new_tracks,
        statistics->consecutive_frames_without_new_tracks);
  } else {
    ++statistics->frames_with_new_tracks;
    statistics->consecutive_frames_without_new_tracks = 0U;
  }
  statistics->maximum_simultaneous_live_tracks = std::max(
      statistics->maximum_simultaneous_live_tracks,
      manager.liveTrackCount());
  statistics->processing_time_sum += result.processing_time_seconds;
  statistics->maximum_processing_time = std::max(
      statistics->maximum_processing_time, result.processing_time_seconds);
}

void printLandmarkSummary(const LandmarkRunStatistics& statistics,
                          const LandmarkTrackManager& manager,
                          std::uint64_t final_frame_index) {
  const std::vector<LandmarkTrack> tracks = manager.allTracks();
  std::array<std::uint64_t,
             static_cast<std::size_t>(LandmarkTrackState::kCount)>
      state_counts{{}};
  std::array<std::uint64_t, 5> member_counts{{}};
  std::map<unsigned int, std::uint64_t> camera_combinations;
  std::vector<double> lifetimes;
  std::vector<double> observation_counts;
  std::vector<double> confirmation_counts;
  std::vector<double> confirmation_frame_counts;
  std::uint64_t single_confirmation_tracks = 0U;
  std::uint64_t multi_confirmation_tracks = 0U;
  std::uint64_t ever_active_tracks = 0U;
  std::uint64_t c2_c3_only_tracks = 0U;
  std::uint64_t c0_or_c1_tracks = 0U;
  for (const LandmarkTrack& track : tracks) {
    const std::size_t state = static_cast<std::size_t>(track.state);
    if (state < state_counts.size()) ++state_counts[state];
    const std::size_t members = track.member_features.size();
    if (members < member_counts.size()) ++member_counts[members];
    unsigned int mask = 0U;
    for (const auto& member : track.member_features)
      mask |= (1U << member.first);
    ++camera_combinations[mask];
    if (mask == ((1U << 2U) | (1U << 3U))) ++c2_c3_only_tracks;
    if ((mask & ((1U << 0U) | (1U << 1U))) != 0U) ++c0_or_c1_tracks;
    const std::uint64_t end_frame =
        track.has_retirement_frame_index ? track.retirement_frame_index
                                         : final_frame_index;
    lifetimes.push_back(static_cast<double>(
        end_frame >= track.creation_frame_index
            ? end_frame - track.creation_frame_index + 1U
            : 0U));
    observation_counts.push_back(
        static_cast<double>(track.total_observation_count));
    confirmation_counts.push_back(
        static_cast<double>(track.cross_camera_confirmation_count));
    confirmation_frame_counts.push_back(
        static_cast<double>(track.distinct_confirmation_frame_count));
    if (track.distinct_confirmation_frame_count == 1U)
      ++single_confirmation_tracks;
    if (track.distinct_confirmation_frame_count >= 2U)
      ++multi_confirmation_tracks;
    if (track.ever_active) ++ever_active_tracks;
  }
  std::uint64_t repeated_feature_pairs = 0U;
  std::uint64_t maximum_feature_pair_confirmations = 0U;
  for (const auto& entry : statistics.feature_pair_confirmations) {
    if (entry.second >= 2U) ++repeated_feature_pairs;
    maximum_feature_pair_confirmations =
        std::max(maximum_feature_pair_confirmations, entry.second);
  }

  std::cout << "\nLandmark track hypothesis summary\n"
            << "  semantics: observation association only; not confirmed "
               "landmarks or map points\n"
            << "  admitted candidate inputs: "
            << statistics.admitted_candidate_inputs << "\n"
            << "  created tracks: " << statistics.created << "\n"
            << "  attached new camera members: "
            << statistics.attached_members << "\n"
            << "  updated existing tracks: "
            << statistics.updated_existing << "\n"
            << "  association conflicts: "
            << statistics.association_conflicts << "\n"
            << "  lifecycle events activated/stale/retired: "
            << statistics.activation_events << "/"
            << statistics.stale_events << "/"
            << statistics.retirement_events << "\n"
            << "  final tentative/active/stale/retired: "
            << state_counts[static_cast<std::size_t>(
                   LandmarkTrackState::kTentative)] << "/"
            << state_counts[static_cast<std::size_t>(
                   LandmarkTrackState::kActive)] << "/"
            << state_counts[static_cast<std::size_t>(
                   LandmarkTrackState::kStale)] << "/"
            << state_counts[static_cast<std::size_t>(
                   LandmarkTrackState::kRetired)] << "\n"
            << "  maximum simultaneous live tracks: "
            << statistics.maximum_simultaneous_live_tracks << "\n"
            << "  frames with new tracks: "
            << statistics.frames_with_new_tracks << "\n"
            << "  longest consecutive frames without new tracks: "
            << statistics.longest_frames_without_new_tracks << "\n"
            << "  member camera counts 2/3/4: " << member_counts[2] << "/"
            << member_counts[3] << "/" << member_counts[4] << "\n"
            << "  C2-C3-only tracks: " << c2_c3_only_tracks << "\n"
            << "  tracks involving C0 or C1: " << c0_or_c1_tracks << "\n"
            << "  single/multi confirmation-frame tracks: "
            << single_confirmation_tracks << "/"
            << multi_confirmation_tracks << "\n"
            << "  tracks ever reaching active: " << ever_active_tracks
            << "\n"
            << "  longest consecutive frames with C2-C3 candidates: "
            << statistics.longest_c2_c3_candidate_streak << "\n"
            << "  repeated exact feature pairs/max confirmations: "
            << repeated_feature_pairs << "/"
            << maximum_feature_pair_confirmations << "\n"
            << "  association status counts";
  for (std::size_t index = 0U; index < statistics.status_counts.size();
       ++index) {
    std::cout << " "
              << landmarkAssociationStatusName(
                     static_cast<LandmarkAssociationStatus>(index))
              << "=" << statistics.status_counts[index];
  }
  std::cout << "\n  admitted candidates by configured pair";
  for (const auto& pair : statistics.admitted_by_pair) {
    std::cout << " C" << pair.first.first << "-C" << pair.first.second
              << "=" << pair.second;
  }
  std::cout << "\n  final member camera combinations";
  for (const auto& combination : camera_combinations) {
    std::cout << " mask" << combination.first << "=" << combination.second;
  }
  std::cout << std::endl;
  printQuantiles("track lifetime frames", lifetimes, "  ");
  printQuantiles("total observation count", observation_counts, "  ");
  printQuantiles("cross-camera confirmation count", confirmation_counts,
                 "  ");
  printQuantiles("distinct confirmation frame count",
                 confirmation_frame_counts, "  ");
  const double frames = static_cast<double>(statistics.frames);
  std::cout << "  manager processing ms avg/max="
            << (statistics.frames == 0U
                    ? 0.0
                    : 1000.0 * statistics.processing_time_sum / frames)
            << "/" << 1000.0 * statistics.maximum_processing_time
            << std::endl;
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
  std::unique_ptr<TriangulationCandidateEvaluator> candidate_evaluator;
  std::unique_ptr<LandmarkTrackManager> landmark_track_manager;
  if (options_.cross_camera_matching) {
    descriptor_extractor.reset(new OrbDescriptorExtractor(options_.descriptor));
    cross_camera_matcher.reset(new CrossCameraMatcher(options_.matcher));
  }
  if (options_.triangulation_candidates) {
    candidate_evaluator.reset(new TriangulationCandidateEvaluator(
        options_.triangulation_candidate));
  }
  if (options_.landmark_tracks) {
    landmark_track_manager.reset(
        new LandmarkTrackManager(options_.landmark_track));
  }
  std::unique_ptr<Eskf> backend;
  std::unique_ptr<Msckf> msckf_backend;
  std::unique_ptr<BackendLandmarkMap> backend_landmarks;
  std::unique_ptr<RosOutput> ros_output;
  OutputRecorder output_recorder;
  if (options_.enable_msckf) {
    msckf_backend.reset(new Msckf(options_.msckf));
    backend_landmarks.reset(new BackendLandmarkMap());
  } else if (options_.enable_backend) {
    backend.reset(new Eskf(options_.backend));
    backend_landmarks.reset(new BackendLandmarkMap());
  }
  if (options_.publish_ros) {
    ros_output.reset(new RosOutput());
  }
  if (!options_.output_directory.empty() &&
      !output_recorder.open(options_.output_directory)) {
    std::cerr << "Cannot open output directory: "
              << options_.output_directory << std::endl;
    bag.close();
    return 8;
  }
  ImuIntervalBuffer backend_imu_buffer(
      options_.bag.maximum_imu_time_difference);
  bool backend_imu_preloaded = false;
  if (backend || msckf_backend) {
    rosbag::View imu_view(
        bag, rosbag::TopicQuery(std::vector<std::string>{
                 options_.bag.imu_topic}),
        processing_start, processing_end);
    for (const rosbag::MessageInstance& instance : imu_view) {
      const sensor_msgs::ImuConstPtr message =
          instance.instantiate<sensor_msgs::Imu>();
      if (!message) continue;
      ImuMeasurement measurement;
      if (convertImuMessage(*message, &measurement)) {
        backend_imu_buffer.add(measurement);
      }
    }
    backend_imu_preloaded = true;
  }
  bool backend_has_frame = false;
  Timestamp backend_previous_frame_time = 0.0;
  std::array<CameraRunStatistics, 4> run_statistics;
  std::array<DescriptorRunStatistics, 4> descriptor_statistics;
  std::vector<PairRunStatistics> pair_statistics(
      options_.matcher.camera_pairs.size());
  std::vector<CandidateRunStatistics> candidate_pair_statistics(
      options_.matcher.camera_pairs.size());
  CandidateRunStatistics global_candidate_statistics;
  LandmarkRunStatistics landmark_statistics;
  std::vector<ThresholdSweepRun> threshold_sweeps;
  if (options_.triangulation_threshold_sweep) {
    threshold_sweeps = makeThresholdSweeps(
        options_.triangulation_candidate,
        options_.matcher.camera_pairs.size());
  }
  std::uint64_t completed_frames = 0U;
  double cross_camera_processing_time_sum = 0.0;
  double maximum_cross_camera_processing_time = 0.0;

  for (const rosbag::MessageInstance& instance : view) {
    if (!backend_imu_preloaded &&
        topicMatches(instance.getTopic(), options_.bag.imu_topic)) {
      const sensor_msgs::ImuConstPtr message =
          instance.instantiate<sensor_msgs::Imu>();
      if (message) {
        ImuMeasurement measurement;
        if (convertImuMessage(*message, &measurement)) {
          backend_imu_buffer.add(measurement);
        }
      }
      continue;
    }
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

      if (backend || msckf_backend) {
        const std::vector<ImuMeasurement> interval =
            backend_imu_buffer.extract(
                backend_has_frame ? backend_previous_frame_time : 0.0,
                frame.timestamp);
        if (msckf_backend) {
          if (!msckf_backend->initialized()) {
            if (!interval.empty()) {
              msckf_backend->initialize(interval.front());
              msckf_backend->propagate(interval, frame.timestamp);
            }
          } else {
            msckf_backend->propagate(interval, frame.timestamp);
          }
          if (msckf_backend->initialized()) {
            msckf_backend->augmentClone(frame.timestamp);
          }
        } else if (backend) {
          if (!backend->initialized()) {
            if (!interval.empty()) {
              backend->initialize(interval.front());
              backend->propagate(interval, frame.timestamp);
            }
          } else {
            backend->propagate(interval, frame.timestamp);
          }
        }
        backend_has_frame = true;
        backend_previous_frame_time = frame.timestamp;
      }

      for (CameraId id = 0U; id < 4U; ++id)
        addResult(tracking_result.cameras[id], &run_statistics[id]);

      std::vector<CrossCameraPairResult> cross_camera_results;
      std::vector<TriangulationCandidatePairResult> candidate_results;
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
        if (candidate_evaluator) {
          candidate_results.resize(cross_camera_results.size());
          std::size_t admitted_in_frame = 0U;
          double candidate_processing_in_frame = 0.0;
          for (std::size_t index = 0U; index < cross_camera_results.size();
               ++index) {
            const CrossCameraPairResult& matching =
                cross_camera_results[index];
            if (matching.camera_id_1 >= descriptor_sets.size() ||
                matching.camera_id_2 >= descriptor_sets.size() ||
                !candidate_evaluator->evaluatePair(
                    matching, descriptor_sets[matching.camera_id_1],
                    descriptor_sets[matching.camera_id_2], camera_rig,
                    &candidate_results[index])) {
              std::cerr << "Triangulation candidate evaluation failed at "
                        << "frame " << completed_frames << std::endl;
              bag.close();
              return 7;
            }
            addCandidateResult(candidate_results[index], true,
                               &candidate_pair_statistics[index]);
            addCandidateResult(candidate_results[index], false,
                               &global_candidate_statistics);
            admitted_in_frame += candidate_results[index].candidates.size();
            candidate_processing_in_frame +=
                candidate_results[index].processing_time_seconds;
          }
          finishGlobalCandidateFrame(admitted_in_frame,
                                     &global_candidate_statistics);
          global_candidate_statistics.maximum_processing_time = std::max(
              global_candidate_statistics.maximum_processing_time,
              candidate_processing_in_frame);

          for (ThresholdSweepRun& sweep : threshold_sweeps) {
            std::size_t sweep_admitted_in_frame = 0U;
            for (std::size_t index = 0U; index < candidate_results.size();
                 ++index) {
              const TriangulationCandidatePairResult readmitted = readmitPair(
                  candidate_results[index], sweep.options);
              addCandidateResult(readmitted, true, &sweep.pairs[index]);
              addCandidateResult(readmitted, false, &sweep.global);
              sweep_admitted_in_frame += readmitted.candidates.size();
            }
            finishGlobalCandidateFrame(sweep_admitted_in_frame,
                                       &sweep.global);
          }
        }
        if (landmark_track_manager) {
          LandmarkTrackFrameInput landmark_input;
          landmark_input.timestamp = frame.timestamp;
          landmark_input.frame_index = completed_frames;
          landmark_input.camera_tracking = tracking_result.cameras;
          for (const TriangulationCandidatePairResult& pair :
               candidate_results) {
            landmark_input.triangulation_diagnostics.insert(
                landmark_input.triangulation_diagnostics.end(),
                pair.diagnostics.begin(), pair.diagnostics.end());
          }
          LandmarkTrackFrameResult landmark_result;
          if (!landmark_track_manager->processFrame(landmark_input,
                                                    &landmark_result)) {
            std::cerr << "Landmark track management failed at frame "
                      << completed_frames << std::endl;
            bag.close();
            return 7;
          }
          addLandmarkFrameResult(candidate_results, landmark_result,
                                 *landmark_track_manager,
                                 &landmark_statistics);

          if (backend && backend_landmarks && backend->initialized()) {
            for (const LandmarkTrack& track :
                 landmark_track_manager->activeTracks()) {
              Eigen::Vector3d body_point;
              Eigen::Vector3d world_measurement;
              bool is_new = false;
              if (backend_landmarks->observe(track, backend->state(),
                                             &body_point,
                                             &world_measurement, &is_new) &&
                  !is_new) {
                backend->updatePosition(world_measurement, body_point,
                                        options_.backend_position_noise);
              }
            }
          } else if (msckf_backend && msckf_backend->initialized()) {
            msckf_backend->update(landmark_track_manager->activeTracks(),
                                  camera_rig);
            msckf_backend->marginalizeOldestClone();
          }
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

      EskfState output_state;
      Eigen::Matrix<double, 15, 15> output_covariance;
      if (backend && backend->initialized()) {
        output_state = backend->state();
        output_covariance = backend->covariance();
      } else if (msckf_backend && msckf_backend->initialized()) {
        output_state.timestamp = msckf_backend->state().timestamp;
        output_state.q_wb = msckf_backend->state().q_wb;
        output_state.p_wb = msckf_backend->state().p_wb;
        output_state.v_wb = msckf_backend->state().v_wb;
        output_state.bias_gyro = msckf_backend->state().bias_gyro;
        output_state.bias_accel = msckf_backend->state().bias_accel;
        output_covariance =
            msckf_backend->covariance().topLeftCorner<15, 15>();

        if (backend_landmarks && landmark_track_manager) {
          for (const LandmarkTrack& track :
               landmark_track_manager->activeTracks()) {
            Eigen::Vector3d body_point;
            Eigen::Vector3d world_measurement;
            bool is_new = false;
            backend_landmarks->observe(track, output_state, &body_point,
                                       &world_measurement, &is_new);
          }
        }
      }

      if (backend_landmarks) {
        output_recorder.record(output_state, backend_landmarks->landmarks());
        if (ros_output) {
          ros_output->publish(output_state, output_covariance,
                              backend_landmarks->landmarks());
        }
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
        if (!candidate_results.empty()) {
          std::cout << "  admitted triangulation candidates";
          for (const TriangulationCandidatePairResult& pair :
               candidate_results) {
            std::cout << " C" << pair.camera_id_1 << "-C"
                      << pair.camera_id_2 << "=" << pair.candidates.size();
          }
          std::cout << std::endl;
        }
        if (landmark_track_manager) {
          std::cout << "  live landmark hypotheses="
                    << landmark_track_manager->liveTrackCount()
                    << " (observation association only)" << std::endl;
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
    if (options_.triangulation_candidates) {
      std::cout << "\nTriangulation candidate diagnostics summary"
                << std::endl
                << "  policy: current-frame geometry admission only; no "
                   "landmark association, no depth filtering"
                << std::endl;
      for (const CandidateRunStatistics& statistics :
           candidate_pair_statistics) {
        const std::string label =
            "C" + std::to_string(statistics.camera_id_1) + "-C" +
            std::to_string(statistics.camera_id_2);
        printCandidateStatistics(statistics, label, "  ", true);
      }
      printCandidateStatistics(global_candidate_statistics, "GLOBAL", "  ",
                               true);
    }
    if (options_.triangulation_threshold_sweep) {
      std::cout << "\nTriangulation candidate single-variable threshold "
                   "sweep"
                << std::endl
                << "  all non-scanned gates remain at configured defaults; "
                   "no Cartesian product"
                << std::endl;
      for (const ThresholdSweepRun& sweep : threshold_sweeps) {
        std::cout << "  scan " << sweep.parameter << "=" << sweep.value
                  << std::endl;
        for (const CandidateRunStatistics& statistics : sweep.pairs) {
          const std::string label =
              "C" + std::to_string(statistics.camera_id_1) + "-C" +
              std::to_string(statistics.camera_id_2);
          printCandidateStatistics(statistics, label, "    ", false);
        }
        printCandidateStatistics(sweep.global, "GLOBAL", "    ", false);
      }
    }
  if (options_.landmark_tracks && landmark_track_manager) {
      printLandmarkSummary(landmark_statistics, *landmark_track_manager,
                           completed_frames);
    }
  }
  output_recorder.close();
  return completed_frames == 0U ? 7 : 0;
}

}  // namespace sphere_vio
