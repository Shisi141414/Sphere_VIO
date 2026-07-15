#pragma once

#include "sphere_vio/ros/offline_bag_runner.hpp"

namespace sphere_vio {

struct OfflineBagVisualizerOptions {
  OfflineBagConfiguration bag;
  double playback_rate = 1.0;
  double imu_gap_warning = 0.008;
};

class OfflineBagVisualizer {
 public:
  explicit OfflineBagVisualizer(OfflineBagVisualizerOptions options);
  int run();

 private:
  OfflineBagVisualizerOptions options_;
};

}  // namespace sphere_vio
