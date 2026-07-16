#include "sphere_vio/ros/frontend_options_loader.hpp"

#include <cmath>

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

}  // namespace sphere_vio
