#pragma once

#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/common/types.hpp"
#include "sphere_vio/frontend/landmark_track.hpp"

namespace sphere_vio {

struct MsckfOptions {
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  double gyroscope_noise = 1.0e-2;
  double accelerometer_noise = 1.0e-1;
  double gyroscope_bias_noise = 1.0e-4;
  double accelerometer_bias_noise = 1.0e-3;
  double pixel_noise = 1.5;
  int maximum_clones = 20;
  Eigen::Matrix<double, 15, 15> initial_covariance =
      Eigen::Matrix<double, 15, 15>::Identity();
};

struct MsckfCurrentState {
  Timestamp timestamp = 0.0;
  Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_wb = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_wb = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
};

struct MsckfClone {
  Timestamp timestamp = 0.0;
  Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_wb = Eigen::Vector3d::Zero();
  int index = 0;
};

// Sliding-window MSCKF backend. The current IMU state occupies the first 15
// covariance entries and each clone adds six error-state entries
// [theta, position].
class Msckf {
 public:
  explicit Msckf(MsckfOptions options = {});

  bool initialized() const { return initialized_; }
  const MsckfCurrentState& state() const { return state_; }
  const Eigen::MatrixXd& covariance() const { return covariance_; }
  std::size_t cloneCount() const { return clones_.size(); }

  bool initialize(const ImuMeasurement& measurement);
  bool propagate(const std::vector<ImuMeasurement>& measurements,
                 Timestamp end_time);
  bool augmentClone(Timestamp timestamp);
  bool update(const std::vector<LandmarkTrack>& tracks,
              const CameraRig& camera_rig);
  void marginalizeOldestClone();

 private:
  struct FeatureMeasurement {
    Timestamp timestamp = 0.0;
    CameraId camera_id = 0U;
    Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
    Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
    int clone_index = 0;
  };

  bool buildFeature(const LandmarkTrack& track,
                    std::vector<FeatureMeasurement>* measurements) const;
  void propagateSegment(const ImuMeasurement& minus,
                        const ImuMeasurement& plus);
  bool applyErrorState(const Eigen::VectorXd& correction);
  bool computeProjectionAndJacobians(
      const CameraRig& rig, const MsckfClone& clone,
      const FeatureMeasurement& measurement, Eigen::Vector2d* predicted,
      Eigen::Matrix<double, 2, 3>* jacobian_feature,
      Eigen::Matrix<double, 2, 6>* jacobian_clone) const;

  MsckfOptions options_;
  MsckfCurrentState state_;
  Eigen::MatrixXd covariance_;
  std::map<Timestamp, MsckfClone> clones_;
  bool initialized_ = false;
};

}  // namespace sphere_vio
