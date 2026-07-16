#pragma once

#include <string>

#include "sphere_vio/frontend/temporal_frontend.hpp"
#include "sphere_vio/ros/offline_bag_runner.hpp"

namespace sphere_vio {

struct OfflineBagVisualizerOptions {
  OfflineBagConfiguration bag;
  double playback_rate = 1.0;
  double imu_gap_warning = 0.008;
  bool show_spherical_coverage = false;
  bool show_epipolar_curve = false;
  bool show_temporal_features = false;
  std::string camera_config_file;
  TemporalFrontendOptions frontend;
  int epipolar_source_camera = 0;
  int epipolar_target_camera = 1;
  double epipolar_source_u = -1.0;
  double epipolar_source_v = -1.0;
};

class OfflineBagVisualizer {
 public:
  explicit OfflineBagVisualizer(OfflineBagVisualizerOptions options);
  int run();

 private:
  OfflineBagVisualizerOptions options_;
};

}  // namespace sphere_vio
