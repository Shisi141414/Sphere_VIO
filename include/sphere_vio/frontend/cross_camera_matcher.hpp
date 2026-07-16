#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/frontend/orb_descriptor_extractor.hpp"

namespace sphere_vio {

struct CrossCameraMatcherOptions {
  std::vector<std::pair<CameraId, CameraId>> camera_pairs;
  double maximum_descriptor_distance = 64.0;
  double ratio_test = 0.80;
  bool require_mutual_best = true;
  double maximum_epipolar_angle = 0.003;
};

struct CrossCameraMatch {
  CameraId camera_id_1 = 0U;
  CameraId camera_id_2 = 0U;
  FeatureId feature_id_1 = 0U;
  FeatureId feature_id_2 = 0U;
  Eigen::Vector2d pixel_1 = Eigen::Vector2d::Zero();
  Eigen::Vector2d pixel_2 = Eigen::Vector2d::Zero();
  double descriptor_distance = 0.0;
  double ratio = 0.0;
  double epipolar_error_forward = 0.0;
  double epipolar_error_backward = 0.0;
  double epipolar_error_maximum = 0.0;
};

struct CrossCameraPairResult {
  CameraId camera_id_1 = 0U;
  CameraId camera_id_2 = 0U;
  std::vector<CrossCameraMatch> matches;
  std::size_t descriptors_1 = 0U;
  std::size_t descriptors_2 = 0U;
  std::size_t raw_candidates = 0U;
  std::size_t absolute_distance_accepted = 0U;
  std::size_t ratio_accepted = 0U;
  std::size_t mutual_accepted = 0U;
  std::size_t epipolar_accepted = 0U;
  std::size_t rejected_absolute_distance = 0U;
  std::size_t rejected_ratio = 0U;
  std::size_t rejected_non_mutual = 0U;
  std::size_t rejected_epipolar = 0U;
  std::size_t rejected_degenerate_geometry = 0U;
  std::size_t rejected_duplicate = 0U;
  double processing_time_seconds = 0.0;
};

class CrossCameraMatcher {
 public:
  explicit CrossCameraMatcher(CrossCameraMatcherOptions options = {});

  const CrossCameraMatcherOptions& options() const { return options_; }
  bool isConfiguredPair(CameraId camera_id_1, CameraId camera_id_2) const;

  bool matchPair(const CameraDescriptorSet& descriptors_1,
                 const CameraDescriptorSet& descriptors_2,
                 const CameraRig& camera_rig,
                 CrossCameraPairResult* result) const;

  bool matchConfiguredPairs(
      const std::vector<CameraDescriptorSet>& descriptor_sets,
      const CameraRig& camera_rig,
      std::vector<CrossCameraPairResult>* results) const;

 private:
  CrossCameraMatcherOptions options_;
};

}  // namespace sphere_vio
