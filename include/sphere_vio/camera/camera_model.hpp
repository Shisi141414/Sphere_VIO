#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

namespace sphere_vio {

class CameraModel {
 public:
  using Ptr = std::shared_ptr<CameraModel>;

  virtual ~CameraModel() = default;

  // Returns true only when the projection is finite and inside the image.
  virtual bool project(const Eigen::Vector3d& point_c,
                       Eigen::Vector2d* pixel) const = 0;

  // Converts an in-image pixel to a unit-length camera-frame bearing.
  virtual bool unproject(const Eigen::Vector2d& pixel,
                         Eigen::Vector3d* bearing_c) const = 0;

  virtual bool isPixelValid(const Eigen::Vector2d& pixel) const = 0;
  virtual int width() const = 0;
  virtual int height() const = 0;
  virtual std::string modelName() const = 0;
};

}  // namespace sphere_vio
