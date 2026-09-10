#include "sphere_vio/ros/frontend_options_loader.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace sphere_vio {
namespace {

template <typename T>
void readIfPresent(const YAML::Node& node, const char* key, T* value) {
  if (node && node[key]) *value = node[key].as<T>();
}

}  // namespace

bool loadTemporalFrontendOptions(const std::string& config_file,
                                 TemporalFrontendOptions* options,
                                 std::string* error) {
  if (!options) return false;
  try {
    const YAML::Node root = YAML::LoadFile(config_file);
    const YAML::Node frontend = root["frontend"];
    if (!frontend) {
      if (error) *error = "missing frontend section";
      return false;
    }
    readIfPresent(frontend, "maximum_features_per_camera",
                  &options->detector.maximum_features);
    readIfPresent(frontend, "grid_rows", &options->detector.grid_rows);
    readIfPresent(frontend, "grid_columns", &options->detector.grid_columns);
    readIfPresent(frontend, "fast_threshold",
                  &options->detector.fast_threshold);
    readIfPresent(frontend, "fast_nonmax_suppression",
                  &options->detector.fast_nonmax_suppression);
    readIfPresent(frontend, "minimum_feature_distance",
                  &options->detector.minimum_feature_distance);
    readIfPresent(frontend, "border_margin",
                  &options->detector.border_margin);
    options->tracker.border_margin = options->detector.border_margin;

    int window_size = options->tracker.window_size.width;
    readIfPresent(frontend, "lk_window_size", &window_size);
    options->tracker.window_size = cv::Size(window_size, window_size);
    readIfPresent(frontend, "pyramid_levels",
                  &options->tracker.pyramid_levels);
    readIfPresent(frontend, "lk_max_iterations",
                  &options->tracker.maximum_iterations);
    readIfPresent(frontend, "lk_termination_epsilon",
                  &options->tracker.termination_epsilon);
    readIfPresent(frontend, "maximum_lk_error",
                  &options->tracker.maximum_lk_error);
    readIfPresent(frontend, "maximum_forward_backward_error",
                  &options->tracker.maximum_forward_backward_error);
    readIfPresent(frontend, "redetection_ratio",
                  &options->redetection_ratio);
  } catch (const YAML::Exception& exception) {
    if (error) *error = exception.what();
    return false;
  }

  const FeatureDetectorOptions& detector = options->detector;
  const FeatureTrackerOptions& tracker = options->tracker;
  if (detector.maximum_features == 0U || detector.grid_rows <= 0 ||
      detector.grid_columns <= 0 || detector.fast_threshold < 0 ||
      !std::isfinite(detector.minimum_feature_distance) ||
      detector.minimum_feature_distance < 0.0 || detector.border_margin < 0 ||
      tracker.window_size.width <= 0 || tracker.pyramid_levels < 0 ||
      tracker.maximum_iterations <= 0 ||
      !std::isfinite(tracker.termination_epsilon) ||
      tracker.termination_epsilon <= 0.0 ||
      !std::isfinite(tracker.maximum_lk_error) ||
      tracker.maximum_lk_error < 0.0 ||
      !std::isfinite(tracker.maximum_forward_backward_error) ||
      tracker.maximum_forward_backward_error < 0.0 ||
      !std::isfinite(options->redetection_ratio) ||
      options->redetection_ratio < 0.0 || options->redetection_ratio > 1.0) {
    if (error) *error = "frontend parameters are outside their valid range";
    return false;
  }
  return true;
}

bool loadCrossCameraOptions(
    const std::string& config_file,
    OrbDescriptorExtractorOptions* descriptor_options,
    CrossCameraMatcherOptions* matcher_options, std::string* error) {
  if (!descriptor_options || !matcher_options) return false;
  try {
    const YAML::Node root = YAML::LoadFile(config_file);
    const YAML::Node cross_camera = root["frontend"]["cross_camera"];
    if (!cross_camera) {
      if (error) *error = "missing frontend.cross_camera section";
      return false;
    }
    readIfPresent(cross_camera, "orb_patch_size",
                  &descriptor_options->patch_size);
    readIfPresent(cross_camera, "orb_edge_threshold",
                  &descriptor_options->edge_threshold);
    readIfPresent(cross_camera, "orb_levels", &descriptor_options->levels);
    readIfPresent(cross_camera, "orb_scale_factor",
                  &descriptor_options->scale_factor);
    readIfPresent(cross_camera, "orb_fast_threshold",
                  &descriptor_options->fast_threshold);
    readIfPresent(cross_camera, "maximum_descriptor_distance",
                  &matcher_options->maximum_descriptor_distance);
    readIfPresent(cross_camera, "ratio_test", &matcher_options->ratio_test);
    readIfPresent(cross_camera, "require_mutual_best",
                  &matcher_options->require_mutual_best);
    readIfPresent(cross_camera, "maximum_epipolar_angle",
                  &matcher_options->maximum_epipolar_angle);

    const YAML::Node pairs = cross_camera["camera_pairs"];
    if (!pairs || !pairs.IsSequence()) {
      if (error) *error = "cross-camera camera_pairs must be a sequence";
      return false;
    }
    matcher_options->camera_pairs.clear();
    for (const YAML::Node& pair : pairs) {
      if (!pair.IsSequence() || pair.size() != 2U) {
        if (error) *error = "each camera pair must contain exactly two ids";
        return false;
      }
      CameraId first = pair[0].as<CameraId>();
      CameraId second = pair[1].as<CameraId>();
      if (first > second) std::swap(first, second);
      matcher_options->camera_pairs.emplace_back(first, second);
    }
  } catch (const YAML::Exception& exception) {
    if (error) *error = exception.what();
    return false;
  }

  std::sort(matcher_options->camera_pairs.begin(),
            matcher_options->camera_pairs.end());
  std::set<std::pair<CameraId, CameraId>> unique_pairs;
  for (const auto& pair : matcher_options->camera_pairs) {
    if (pair.first >= 4U || pair.second >= 4U || pair.first >= pair.second ||
        !unique_pairs.insert(pair).second) {
      if (error) *error = "cross-camera pairs are invalid or duplicated";
      return false;
    }
  }
  if (matcher_options->camera_pairs.empty() ||
      descriptor_options->patch_size <= 0 ||
      descriptor_options->patch_size % 2 != 1 ||
      descriptor_options->edge_threshold < 0 ||
      descriptor_options->levels <= 0 ||
      !std::isfinite(descriptor_options->scale_factor) ||
      descriptor_options->scale_factor <= 1.0 ||
      descriptor_options->fast_threshold < 0 ||
      !std::isfinite(matcher_options->maximum_descriptor_distance) ||
      matcher_options->maximum_descriptor_distance < 0.0 ||
      !std::isfinite(matcher_options->ratio_test) ||
      matcher_options->ratio_test <= 0.0 || matcher_options->ratio_test >= 1.0 ||
      !std::isfinite(matcher_options->maximum_epipolar_angle) ||
      matcher_options->maximum_epipolar_angle < 0.0) {
    if (error) *error = "cross-camera parameters are outside valid ranges";
    return false;
  }
  return true;
}

bool loadTriangulationCandidateOptions(
    const std::string& config_file,
    TriangulationCandidateOptions* candidate_options,
    std::string* error) {
  if (!candidate_options) return false;
  try {
    const YAML::Node root = YAML::LoadFile(config_file);
    const YAML::Node candidates =
        root["frontend"]["triangulation_candidates"];
    if (!candidates) {
      if (error) {
        *error = "missing frontend.triangulation_candidates section";
      }
      return false;
    }
    readIfPresent(candidates, "minimum_ray_angle",
                  &candidate_options->minimum_ray_angle);
    readIfPresent(candidates, "minimum_depth",
                  &candidate_options->minimum_depth);
    readIfPresent(candidates, "maximum_depth",
                  &candidate_options->maximum_depth);
    readIfPresent(candidates, "maximum_closest_ray_distance",
                  &candidate_options->maximum_closest_ray_distance);
    readIfPresent(candidates, "maximum_angular_reprojection_error",
                  &candidate_options->maximum_angular_reprojection_error);
    readIfPresent(candidates, "maximum_epipolar_error",
                  &candidate_options->maximum_epipolar_error);
  } catch (const YAML::Exception& exception) {
    if (error) *error = exception.what();
    return false;
  }
  if (!std::isfinite(candidate_options->minimum_ray_angle) ||
      candidate_options->minimum_ray_angle < 0.0 ||
      !std::isfinite(candidate_options->minimum_depth) ||
      candidate_options->minimum_depth < 0.0 ||
      !std::isfinite(candidate_options->maximum_depth) ||
      candidate_options->maximum_depth < candidate_options->minimum_depth ||
      !std::isfinite(candidate_options->maximum_closest_ray_distance) ||
      candidate_options->maximum_closest_ray_distance < 0.0 ||
      !std::isfinite(
          candidate_options->maximum_angular_reprojection_error) ||
      candidate_options->maximum_angular_reprojection_error < 0.0 ||
      !std::isfinite(candidate_options->maximum_epipolar_error) ||
      candidate_options->maximum_epipolar_error < 0.0) {
    if (error) {
      *error = "triangulation candidate parameters are outside valid ranges";
    }
    return false;
  }
  return true;
}

bool loadLandmarkTrackManagerOptions(
    const std::string& config_file,
    LandmarkTrackManagerOptions* landmark_options,
    std::string* error) {
  if (!landmark_options) return false;
  try {
    const YAML::Node root = YAML::LoadFile(config_file);
    const YAML::Node landmark_tracks =
        root["frontend"]["landmark_tracks"];
    if (!landmark_tracks) {
      if (error) *error = "missing frontend.landmark_tracks section";
      return false;
    }
    readIfPresent(landmark_tracks, "minimum_confirmations_for_active",
                  &landmark_options->minimum_confirmations_for_active);
    readIfPresent(landmark_tracks, "maximum_frames_without_observation",
                  &landmark_options->maximum_frames_without_observation);
    readIfPresent(landmark_tracks, "maximum_frames_without_confirmation",
                  &landmark_options->maximum_frames_without_confirmation);
    readIfPresent(landmark_tracks,
                  "retire_after_frames_without_observation",
                  &landmark_options->retire_after_frames_without_observation);
    readIfPresent(landmark_tracks, "maximum_observation_history",
                  &landmark_options->maximum_observation_history);
  } catch (const YAML::Exception& exception) {
    if (error) *error = exception.what();
    return false;
  }
  if (landmark_options->minimum_confirmations_for_active < 1U ||
      landmark_options->maximum_observation_history < 1U ||
      landmark_options->retire_after_frames_without_observation <
          landmark_options->maximum_frames_without_observation) {
    if (error) {
      *error = "landmark track parameters are outside valid ranges";
    }
    return false;
  }
  return true;
}

bool loadMsckfOptions(const std::string& config_file,
                      MsckfOptions* msckf_options, std::string* error) {
  if (!msckf_options) return false;
  try {
    const YAML::Node root = YAML::LoadFile(config_file);
    const YAML::Node backend = root["backend"];
    const YAML::Node msckf = backend ? backend["msckf"]
                                      : root["msckf"];
    if (!msckf) {
      if (error) *error = "missing backend.msckf section";
      return false;
    }
    readIfPresent(msckf, "gravity_magnitude",
                  &msckf_options->gravity_magnitude);
    readIfPresent(msckf, "gyroscope_noise",
                  &msckf_options->gyroscope_noise);
    readIfPresent(msckf, "accelerometer_noise",
                  &msckf_options->accelerometer_noise);
    readIfPresent(msckf, "gyroscope_bias_noise",
                  &msckf_options->gyroscope_bias_noise);
    readIfPresent(msckf, "accelerometer_bias_noise",
                  &msckf_options->accelerometer_bias_noise);
    readIfPresent(msckf, "pixel_noise", &msckf_options->pixel_noise);
    readIfPresent(msckf, "feature_chi_square_probability",
                  &msckf_options->feature_chi_square_probability);
    readIfPresent(msckf, "maximum_clones",
                  &msckf_options->maximum_clones);
    readIfPresent(msckf, "maximum_feature_observations",
                  &msckf_options->maximum_feature_observations);
    readIfPresent(msckf, "maximum_frames_without_observation",
                  &msckf_options->maximum_frames_without_observation);

    const YAML::Node covariance =
        msckf["initial_covariance_diagonal"];
    if (covariance && covariance.IsSequence()) {
      Eigen::Matrix<double, 15, 15> diagonal =
          Eigen::Matrix<double, 15, 15>::Identity();
      if (covariance.size() == 15U) {
        for (std::size_t index = 0U; index < 15U; ++index) {
          diagonal(index, index) = covariance[index].as<double>();
        }
        msckf_options->initial_covariance = diagonal;
      }
    }

    msckf_options->gravity =
        Eigen::Vector3d(0.0, 0.0, -msckf_options->gravity_magnitude);
  } catch (const YAML::Exception& exception) {
    if (error) *error = exception.what();
    return false;
  }

  if (!std::isfinite(msckf_options->gravity_magnitude) ||
      msckf_options->gravity_magnitude <= 0.0 ||
      !std::isfinite(msckf_options->gyroscope_noise) ||
      msckf_options->gyroscope_noise < 0.0 ||
      !std::isfinite(msckf_options->accelerometer_noise) ||
      msckf_options->accelerometer_noise < 0.0 ||
      !std::isfinite(msckf_options->gyroscope_bias_noise) ||
      msckf_options->gyroscope_bias_noise < 0.0 ||
      !std::isfinite(msckf_options->accelerometer_bias_noise) ||
      msckf_options->accelerometer_bias_noise < 0.0 ||
      !std::isfinite(msckf_options->pixel_noise) ||
      msckf_options->pixel_noise <= 0.0 ||
      !std::isfinite(msckf_options->feature_chi_square_probability) ||
      msckf_options->feature_chi_square_probability <= 0.0 ||
      msckf_options->feature_chi_square_probability >= 1.0 ||
      msckf_options->maximum_clones < 2 ||
      msckf_options->maximum_feature_observations < 2U ||
      msckf_options->maximum_frames_without_observation == 0U) {
    if (error) *error = "backend.msckf parameters are outside valid ranges";
    return false;
  }
  return true;
}

}  // namespace sphere_vio
