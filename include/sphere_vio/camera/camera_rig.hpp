#pragma once

#include <cstddef>
#include <limits>
#include <map>
#include <string>

#include <Eigen/Core>

#include "sphere_vio/camera/camera_model.hpp"
#include "sphere_vio/common/types.hpp"

namespace sphere_vio {

struct RigCamera {
  CameraId camera_id = 0;
  std::string name;
  CameraModel::Ptr model;
  Eigen::Matrix3d R_b_c = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_b_c = Eigen::Vector3d::Zero();
  int kalibr_camera_id = -1;
  bool timeshift_available = false;
  double timeshift_cam_imu = std::numeric_limits<double>::quiet_NaN();
};

class CameraRig {
 public:
  bool addCamera(const RigCamera& camera);

  std::size_t size() const;
  bool hasCamera(CameraId camera_id) const;
  const RigCamera* camera(CameraId camera_id) const;

  bool pixelToCameraBearing(CameraId camera_id,
                            const Eigen::Vector2d& pixel,
                            Eigen::Vector3d* bearing_c) const;
  bool cameraBearingToBody(CameraId camera_id,
                           const Eigen::Vector3d& bearing_c,
                           Eigen::Vector3d* bearing_b) const;
  bool bodyBearingToCamera(CameraId camera_id,
                           const Eigen::Vector3d& bearing_b,
                           Eigen::Vector3d* bearing_c) const;
  bool pixelToBodyBearing(CameraId camera_id, const Eigen::Vector2d& pixel,
                          Eigen::Vector3d* bearing_b) const;

  bool cameraPointToBody(CameraId camera_id,
                         const Eigen::Vector3d& point_c,
                         Eigen::Vector3d* point_b) const;
  bool bodyPointToCamera(CameraId camera_id,
                         const Eigen::Vector3d& point_b,
                         Eigen::Vector3d* point_c) const;
  bool projectBodyPointToCamera(CameraId camera_id,
                                const Eigen::Vector3d& point_b,
                                Eigen::Vector2d* pixel) const;

 private:
  std::map<CameraId, RigCamera> cameras_;
};

}  // namespace sphere_vio
