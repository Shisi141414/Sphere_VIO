#pragma once

#include <cstddef>
#include <cstdint>

#include <Eigen/Core>

#include "sphere_vio/common/types.hpp"

namespace sphere_vio {

struct FeatureObservation {
  Timestamp timestamp = 0.0;
  CameraId camera_id = 0U;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
  Eigen::Vector3d bearing_c = Eigen::Vector3d::Zero();
  Eigen::Vector3d bearing_b = Eigen::Vector3d::Zero();
  double tracking_error = 0.0;
};

struct FeatureTrack {
  FeatureId id = 0U;
  CameraId camera_id = 0U;
  FeatureObservation current;
  FeatureObservation previous;
  bool has_previous_observation = false;
  std::uint32_t age = 0U;
  std::uint64_t total_observation_count = 0U;
  bool newly_detected = false;
  double forward_backward_error = 0.0;
  double lk_error = 0.0;
};

enum class TrackRejectionReason {
  kTracked,
  kForwardFlowFailed,
  kBackwardFlowFailed,
  kForwardBackwardError,
  kLkErrorTooLarge,
  kOutsideImage,
  kCameraModelInvalid,
  kInvalidInput
};

struct FeatureDetectionStatistics {
  std::size_t candidate_count = 0U;
  std::size_t border_rejections = 0U;
  std::size_t distance_rejections = 0U;
  std::size_t model_domain_rejections = 0U;
  std::size_t accepted_count = 0U;
};

struct FeatureTrackingStatistics {
  std::size_t input_tracks = 0U;
  std::size_t successfully_tracked = 0U;
  std::size_t forward_flow_failures = 0U;
  std::size_t backward_flow_failures = 0U;
  std::size_t forward_backward_rejections = 0U;
  std::size_t lk_error_rejections = 0U;
  std::size_t outside_image_rejections = 0U;
  std::size_t model_domain_rejections = 0U;
  double average_forward_backward_error = 0.0;
  double maximum_forward_backward_error = 0.0;

  std::size_t rejectedCount() const {
    return forward_flow_failures + backward_flow_failures +
           forward_backward_rejections + lk_error_rejections +
           outside_image_rejections + model_domain_rejections;
  }
};

}  // namespace sphere_vio
