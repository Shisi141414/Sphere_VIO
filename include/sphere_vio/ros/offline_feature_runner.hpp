#pragma once

#include <string>

#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/frontend/temporal_frontend.hpp"
#include "sphere_vio/ros/offline_bag_runner.hpp"

namespace sphere_vio {

struct OfflineFeatureRunnerOptions {
  OfflineBagConfiguration bag;
  TemporalFrontendOptions frontend;
  bool cross_camera_matching = false;
  OrbDescriptorExtractorOptions descriptor;
  CrossCameraMatcherOptions matcher;
  std::string camera_config_file;
};

class OfflineFeatureRunner {
 public:
  explicit OfflineFeatureRunner(OfflineFeatureRunnerOptions options);
  int run();

 private:
  OfflineFeatureRunnerOptions options_;
};

}  // namespace sphere_vio
