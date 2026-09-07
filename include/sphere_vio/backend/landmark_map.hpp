#pragma once

#include <map>
#include <vector>

#include <Eigen/Core>

#include "sphere_vio/backend/eskf.hpp"
#include "sphere_vio/frontend/landmark_track.hpp"

namespace sphere_vio {

struct BackendLandmark {
  LandmarkTrackId track_id;
  Eigen::Vector3d point_w = Eigen::Vector3d::Zero();
  Timestamp last_update_timestamp = 0.0;
  std::uint32_t observation_count = 0U;
};

// Maintains a small world-frame landmark map from LandmarkTrack triangulation
// diagnostics. This is intentionally a lightweight EKF-support map, not a full
// bundle-adjustment landmark state: landmark points are initialized from the
// current filter pose and then used as position measurements.
class BackendLandmarkMap {
 public:
  // Returns true when the supplied track has a usable body-frame triangulation.
  // On a new landmark, body_point and world_measurement are the same point. On
  // an existing landmark, world_measurement is the previous smoothed estimate.
  bool observe(const LandmarkTrack& track, const EskfState& state,
               Eigen::Vector3d* body_point,
               Eigen::Vector3d* world_measurement, bool* is_new);

  std::vector<BackendLandmark> landmarks() const;

 private:
  std::map<LandmarkTrackId, BackendLandmark> landmarks_;
};

}  // namespace sphere_vio
