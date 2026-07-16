#pragma once

#include <Eigen/Core>

namespace sphere_vio {

// Project transform convention: p_P = R_p_b * p_B + t_p_b.
struct PanoramaFrame {
  Eigen::Matrix3d R_p_b = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_p_b = Eigen::Vector3d::Zero();
};

struct PanoramaSpec {
  int width = 2048;
  int height = 1024;
  double horizontal_fov = 6.28318530717958647692;
  double vertical_fov = 3.14159265358979323846;
  double sphere_radius = 1.0;
  PanoramaFrame frame;
};

}  // namespace sphere_vio
