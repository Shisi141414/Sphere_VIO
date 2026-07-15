#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace sphere_vio {

struct OfflineBagConfiguration {
  std::string bag_path;
  std::array<std::string, 4> camera_topics;
  std::string imu_topic;
  double maximum_image_time_difference = 0.001;
  bool require_exact_image_timestamps = true;
  double start_time_offset = 0.0;
  double duration = -1.0;
  int progress_interval_frames = 100;
  double maximum_imu_time_difference = 0.05;
};

class OfflineBagRunner {
 public:
  explicit OfflineBagRunner(OfflineBagConfiguration configuration);
  int run();

 private:
  OfflineBagConfiguration configuration_;
};

bool loadOfflineBagConfiguration(const std::string& config_file,
                                 const std::string& bag_path_override,
                                 OfflineBagConfiguration* configuration,
                                 std::string* error);

}  // namespace sphere_vio
