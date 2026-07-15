#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace sphere_vio {

using Timestamp = double;
using CameraId = std::uint32_t;
using FeatureId = std::uint64_t;

struct ImageFrame {
  Timestamp timestamp = 0.0;
  CameraId camera_id = 0;
  cv::Mat image;
};

struct MultiCameraFrame {
  Timestamp timestamp = 0.0;
  std::vector<ImageFrame> images;
};

struct ImuMeasurement {
  Timestamp timestamp = 0.0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

}  // namespace sphere_vio
