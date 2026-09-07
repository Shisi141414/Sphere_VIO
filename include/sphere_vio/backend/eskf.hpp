#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sphere_vio/common/types.hpp"

namespace sphere_vio {

struct EskfOptions {
  // World-frame gravity. The z axis points down in this initial convention.
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  // Continuous-time IMU noise densities.
  double gyroscope_noise = 1.0e-2;
  double accelerometer_noise = 1.0e-1;
  double gyroscope_bias_noise = 1.0e-4;
  double accelerometer_bias_noise = 1.0e-3;

  Eigen::Matrix<double, 15, 15> initial_covariance =
      Eigen::Matrix<double, 15, 15>::Identity();
};

struct EskfState {
  Timestamp timestamp = 0.0;
  Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_wb = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_wb = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
};

// Small 15-state error-state Kalman filter for visual-inertial experiments.
//
// Error-state ordering is [theta, p, v, bg, ba], where theta is the local
// orientation error in the body frame. The nominal quaternion q_wb maps a body
// point to world: p_w = q_wb * p_b + p_wb.
class Eskf {
 public:
  explicit Eskf(EskfOptions options = {});

  bool initialized() const { return initialized_; }
  const EskfState& state() const { return state_; }
  const Eigen::Matrix<double, 15, 15>& covariance() const {
    return covariance_;
  }

  bool initialize(const ImuMeasurement& measurement);

  // Integrates the supplied measurements up to end_time. The vector should be
  // timestamp ordered and may include the first measurement needed to start the
  // interval. Missing beginning/end samples are extrapolated with the nearest
  // sample so the filter still advances during short image-frame gaps.
  bool propagate(const std::vector<ImuMeasurement>& measurements,
                 Timestamp end_time);

  // Position update: measured_world is an externally maintained landmark
  // estimate; body_point is the current triangulated body-frame point.
  bool updatePosition(const Eigen::Vector3d& measured_world,
                      const Eigen::Vector3d& body_point,
                      double measurement_noise);

  // Optional zero-velocity / external velocity update.
  bool updateVelocity(const Eigen::Vector3d& measured_velocity,
                      double measurement_noise);

 private:
  void propagateSegment(const ImuMeasurement& minus,
                        const ImuMeasurement& plus);
  bool applyErrorState(const Eigen::Matrix<double, 15, 1>& correction);

  EskfOptions options_;
  EskfState state_;
  Eigen::Matrix<double, 15, 15> covariance_;
  bool initialized_ = false;
};

}  // namespace sphere_vio
