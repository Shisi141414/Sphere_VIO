#pragma once

#include <string>

#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/frontend/landmark_track_manager.hpp"
#include "sphere_vio/frontend/temporal_frontend.hpp"
#include "sphere_vio/frontend/triangulation_candidate_evaluator.hpp"
#include "sphere_vio/ros/offline_bag_runner.hpp"

namespace sphere_vio {

struct OfflineFeatureRunnerOptions {
  OfflineBagConfiguration bag;
  TemporalFrontendOptions frontend;
  bool cross_camera_matching = false;
  bool triangulation_candidates = false;
  bool triangulation_threshold_sweep = false;
  bool landmark_tracks = false;
  OrbDescriptorExtractorOptions descriptor;
  CrossCameraMatcherOptions matcher;
  TriangulationCandidateOptions triangulation_candidate;
  LandmarkTrackManagerOptions landmark_track;
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
