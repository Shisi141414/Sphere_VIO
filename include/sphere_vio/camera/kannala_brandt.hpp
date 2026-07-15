#pragma once

#include "sphere_vio/camera/camera_model.hpp"

namespace sphere_vio {

class KannalaBrandt final : public CameraModel {
 public:
  struct Parameters {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
  };

  explicit KannalaBrandt(const Parameters& parameters);

  bool project(const Eigen::Vector3d& point_c,
               Eigen::Vector2d* pixel) const override;
  bool unproject(const Eigen::Vector2d& pixel,
                 Eigen::Vector3d* bearing_c) const override;
  bool isPixelValid(const Eigen::Vector2d& pixel) const override;
  int width() const override;
  int height() const override;
  std::string modelName() const override;

 private:
  double distortAngle(double theta) const;
  double distortionDerivative(double theta) const;
  bool undistortAngle(double theta_d, double* theta) const;

  Parameters parameters_;
};

}  // namespace sphere_vio
