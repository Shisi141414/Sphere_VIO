#pragma once

#include <string>

#include "sphere_vio/backend/eskf.hpp"
#include "sphere_vio/backend/msckf.hpp"
#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/frontend/landmark_track_manager.hpp"
#include "sphere_vio/frontend/superpoint_extractor.hpp"
#include "sphere_vio/frontend/temporal_frontend.hpp"
#include "sphere_vio/frontend/triangulation_candidate_evaluator.hpp"
#include "sphere_vio/ros/offline_bag_runner.hpp"

namespace sphere_vio {

struct RuntimeGovernorOptions {
  double budget_ms = 62.5;
  std::size_t minimum_features_per_camera = 100U;
  double feature_decay_ratio = 0.90;
  double feature_recovery_ratio = 1.02;
  int minimum_pyramid_levels = 1;
};

struct OfflineFeatureRunnerOptions {
  OfflineBagConfiguration bag;
  TemporalFrontendOptions frontend;
  // "legacy" runs detection/LK on the raw fisheye images (default).
  // "rectified" runs the same FAST/ORB path on the 200x100 degree local
  // spherical view while keeping geometry on raw fisheye pixels.
  // "superpoint_cuda" additionally replaces ORB with the SuperPoint detector
  // and requires the optional ONNX Runtime CUDA provider.
  std::string frontend_mode = "legacy";
  bool cross_camera_matching = false;
  bool triangulation_candidates = false;
  bool triangulation_threshold_sweep = false;
  bool landmark_tracks = false;
  OrbDescriptorExtractorOptions descriptor;
  SuperPointExtractorOptions superpoint;
  CrossCameraMatcherOptions matcher;
  TriangulationCandidateOptions triangulation_candidate;
  LandmarkTrackManagerOptions landmark_track;
  std::string camera_config_file;
  bool enable_backend = false;
  EskfOptions backend;
  bool enable_msckf = false;
  MsckfOptions msckf;
  RuntimeGovernorOptions runtime_governor;
  double backend_position_noise = 0.20;
  // Fixed camera-to-IMU clock offset: t_imu = t_camera + offset. The D2SLAM
  // reference configuration records -0.186 s as the authoritative calibration
  // for this dataset family; it is used to stamp IMU intervals, MSCKF
  // propagation, clones, and observations while trajectory output keeps the
  // original camera timestamps for groundtruth evaluation.
  double camera_to_imu_offset_s = -0.186;
  // Regular output sampling period in camera-clock seconds. A value of zero
  // disables interpolation and writes one pose per completed image frame.
  double output_period = 0.05;
  std::string output_directory;
  bool publish_ros = false;
};

class OfflineFeatureRunner {
 public:
  explicit OfflineFeatureRunner(OfflineFeatureRunnerOptions options);
  int run();

 private:
  OfflineFeatureRunnerOptions options_;
};

}  // namespace sphere_vio
