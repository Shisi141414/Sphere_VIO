#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace sphere_vio {

struct OfflineBagConfiguration {
  std::string bag_path;
  std::array<std::string, 4> camera_topics;
  std::string imu_topic;
  // Raw D2SLAM full bags use the DJI SDK IMU topic. Converted Sphere-VIO bags
  // use /imu_data_raw, so the feature runner selects the matching one when the
  // stitched compressed-image topic is present.
  std::string d2slam_imu_topic = "/dji_sdk_1/dji_sdk/imu";
  // D2SLAM records all four fisheye cameras side by side in one compressed
  // image. The feature runner can decode that topic directly, avoiding the
  // large intermediate Sphere-VIO bags required by the old converter path.
  std::string d2slam_stitched_image_topic = "/arducam/image/compressed";
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
