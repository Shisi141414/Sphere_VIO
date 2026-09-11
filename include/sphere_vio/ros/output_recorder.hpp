#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "sphere_vio/backend/eskf.hpp"
#include "sphere_vio/backend/landmark_map.hpp"

namespace sphere_vio {

// Persists backend state and landmark map to CSV files. This is independent of
// ROS so the same output can be produced when no roscore is running.
class OutputRecorder {
 public:
  bool open(const std::string& directory, double output_period = 0.05);
  // camera_timestamp stays in the camera/ground-truth clock while
  // state.timestamp remains on the IMU clock.
  void record(Timestamp camera_timestamp, const EskfState& state,
              const std::vector<BackendLandmark>& landmarks);
  void close();

 private:
  std::ofstream odometry_file_;
  std::ofstream trajectory_file_;
  std::ofstream landmarks_file_;
  double output_period_ = 0.0;
  bool has_previous_state_ = false;
  Timestamp previous_camera_timestamp_ = 0.0;
  EskfState previous_state_;
  Timestamp next_regular_timestamp_ = 0.0;
};

}  // namespace sphere_vio
