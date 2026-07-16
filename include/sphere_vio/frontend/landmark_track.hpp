#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>

#include <Eigen/Core>

#include "sphere_vio/frontend/triangulation_candidate_evaluator.hpp"

namespace sphere_vio {

// Strongly typed so a temporal FeatureId cannot be passed accidentally where
// a cross-camera landmark-hypothesis id is required.
struct LandmarkTrackId {
  std::uint64_t value = 0U;

  bool operator==(const LandmarkTrackId& other) const {
    return value == other.value;
  }
  bool operator!=(const LandmarkTrackId& other) const {
    return !(*this == other);
  }
  bool operator<(const LandmarkTrackId& other) const {
    return value < other.value;
  }
};

struct TemporalFeatureKey {
  CameraId camera_id = 0U;
  FeatureId feature_id = 0U;

  bool operator==(const TemporalFeatureKey& other) const {
    return camera_id == other.camera_id && feature_id == other.feature_id;
  }
  bool operator!=(const TemporalFeatureKey& other) const {
    return !(*this == other);
  }
  bool operator<(const TemporalFeatureKey& other) const {
    if (camera_id != other.camera_id) return camera_id < other.camera_id;
    return feature_id < other.feature_id;
  }
};

struct LandmarkObservation {
  Timestamp timestamp = 0.0;
  std::uint64_t frame_index = 0U;
  TemporalFeatureKey feature;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
  Eigen::Vector3d bearing_c = Eigen::Vector3d::Zero();
  Eigen::Vector3d bearing_b = Eigen::Vector3d::Zero();
  std::uint32_t feature_age = 0U;
  bool from_cross_camera_confirmation = false;
  double descriptor_distance = 0.0;
  double epipolar_error = 0.0;
};

enum class LandmarkTrackState {
  kTentative = 0,
  kActive,
  kStale,
  kRetired,
  kCount
};

const char* landmarkTrackStateName(LandmarkTrackState state);

// Observation-association and lifecycle container only. It is not a confirmed
// landmark, map point, filtered depth, inverse-depth state, or world point.
struct LandmarkTrack {
  LandmarkTrackId id;
  LandmarkTrackState state = LandmarkTrackState::kTentative;

  Timestamp creation_timestamp = 0.0;
  Timestamp last_observation_timestamp = 0.0;
  Timestamp last_cross_camera_confirmation_timestamp = 0.0;
  std::uint64_t creation_frame_index = 0U;
  std::uint64_t last_observation_frame_index = 0U;
  std::uint64_t last_confirmation_frame_index = 0U;

  std::map<CameraId, TemporalFeatureKey> member_features;
  std::deque<LandmarkObservation> observations;

  std::uint64_t total_observation_count = 0U;
  std::uint64_t cross_camera_confirmation_count = 0U;
  std::uint64_t distinct_confirmation_frame_count = 0U;
  std::size_t maximum_simultaneous_camera_count = 0U;

  bool confirmation_stale = false;
  bool ever_active = false;
  bool has_activation_frame_index = false;
  std::uint64_t activation_frame_index = 0U;
  bool has_retirement_frame_index = false;
  std::uint64_t retirement_frame_index = 0U;

  bool has_latest_triangulation_diagnostic = false;
  // A single latest diagnostic snapshot is allowed for inspection. Its point_b
  // and depths are not fused or treated as persistent landmark state.
  TriangulationDiagnostic latest_triangulation_diagnostic;
};

}  // namespace sphere_vio
