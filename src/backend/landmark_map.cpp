#include "sphere_vio/backend/landmark_map.hpp"

#include <algorithm>

namespace sphere_vio {

bool BackendLandmarkMap::observe(const LandmarkTrack& track,
                                 const EskfState& state,
                                 Eigen::Vector3d* body_point,
                                 Eigen::Vector3d* world_measurement,
                                 bool* is_new) {
  if (!body_point || !world_measurement || !is_new) return false;
  *is_new = false;

  if (!track.has_latest_triangulation_diagnostic ||
      !track.latest_triangulation_diagnostic.admitted ||
      !track.latest_triangulation_diagnostic.triangulation.valid) {
    return false;
  }

  const Eigen::Vector3d current_body =
      track.latest_triangulation_diagnostic.point_b;
  if (!current_body.allFinite()) return false;
  const Eigen::Vector3d current_world =
      state.q_wb * current_body + state.p_wb;

  const auto existing = landmarks_.find(track.id);
  if (existing == landmarks_.end()) {
    BackendLandmark landmark;
    landmark.track_id = track.id;
    landmark.point_w = current_world;
    landmark.last_update_timestamp = track.last_observation_timestamp;
    landmark.observation_count = 1U;
    landmarks_.emplace(track.id, landmark);
    *body_point = current_body;
    *world_measurement = current_world;
    *is_new = true;
    return true;
  }

  BackendLandmark& landmark = existing->second;
  *body_point = current_body;
  *world_measurement = landmark.point_w;
  // Slightly smooth the anchor as new observations arrive while keeping the
  // previous estimate as the measurement so the update constrains drift.
  landmark.point_w =
      0.9 * landmark.point_w + 0.1 * current_world;
  landmark.last_update_timestamp = track.last_observation_timestamp;
  ++landmark.observation_count;
  return true;
}

std::vector<BackendLandmark> BackendLandmarkMap::landmarks() const {
  std::vector<BackendLandmark> result;
  result.reserve(landmarks_.size());
  for (const auto& entry : landmarks_) result.push_back(entry.second);
  return result;
}

}  // namespace sphere_vio
