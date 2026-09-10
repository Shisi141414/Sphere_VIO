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
  bool open(const std::string& directory);
  void record(const EskfState& state,
              const std::vector<BackendLandmark>& landmarks);
  void close();

 private:
  std::ofstream odometry_file_;
  std::ofstream trajectory_file_;
  std::ofstream landmarks_file_;
};

}  // namespace sphere_vio
