#include "sphere_vio/backend/eskf.hpp"

#include <algorithm>
#include <cmath>

namespace sphere_vio {
namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vector.z(), vector.y(),
            vector.z(), 0.0, -vector.x(),
            -vector.y(), vector.x(), 0.0;
  return matrix;
}

Eigen::Quaterniond exponentialMap(const Eigen::Vector3d& axis_angle) {
  const double angle = axis_angle.norm();
  if (angle < 1e-12) {
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
    quaternion.coeffs().head<3>() += 0.5 * axis_angle;
    return quaternion.normalized();
  }
  const double half_angle = 0.5 * angle;
  Eigen::Quaterniond quaternion(std::cos(half_angle),
                                std::sin(half_angle) * axis_angle.x() / angle,
                                std::sin(half_angle) * axis_angle.y() / angle,
                                std::sin(half_angle) * axis_angle.z() / angle);
  return quaternion.normalized();
}

bool isFinite(const Eigen::Vector3d& vector) {
  return vector.allFinite();
}

bool isFinite(const Eigen::Quaterniond& quaternion) {
  return quaternion.coeffs().allFinite();
}

}  // namespace

Eskf::Eskf(EskfOptions options) : options_(std::move(options)) {
  covariance_.setIdentity();
}

bool Eskf::initialize(const ImuMeasurement& measurement) {
  if (!measurement.acceleration.allFinite() ||
      !measurement.angular_velocity.allFinite()) {
    return false;
  }

  state_ = EskfState();
  state_.timestamp = measurement.timestamp;
  covariance_ = options_.initial_covariance;
  initialized_ = true;
  return true;
}

bool Eskf::propagate(const std::vector<ImuMeasurement>& measurements,
                     Timestamp end_time) {
  if (!initialized_ || measurements.empty() || end_time <= state_.timestamp) {
    return initialized_;
  }

  for (std::size_t index = 0; index + 1U < measurements.size(); ++index) {
    const ImuMeasurement& minus = measurements[index];
    const ImuMeasurement& plus = measurements[index + 1U];
    if (plus.timestamp <= state_.timestamp) continue;
    if (minus.timestamp >= end_time) break;
    propagateSegment(minus, plus);
    if (state_.timestamp >= end_time) break;
  }

  // Finish any remaining partial interval up to the requested image
  // timestamp with the last known IMU values.
  if (state_.timestamp < end_time) {
    const ImuMeasurement& minus = measurements.back();
    ImuMeasurement plus = minus;
    plus.timestamp = end_time;
    propagateSegment(minus, plus);
  }

  return isFinite(state_.q_wb) && isFinite(state_.p_wb) &&
         isFinite(state_.v_wb) && covariance_.allFinite();
}

bool Eskf::updatePosition(const Eigen::Vector3d& measured_world,
                          const Eigen::Vector3d& body_point,
                          double measurement_noise) {
  if (!initialized_ || !measured_world.allFinite() ||
      !body_point.allFinite() || measurement_noise <= 0.0) {
    return false;
  }

  const Eigen::Matrix3d rotation = state_.q_wb.toRotationMatrix();
  const Eigen::Vector3d predicted = rotation * body_point + state_.p_wb;
  const Eigen::Vector3d residual = measured_world - predicted;

  Eigen::Matrix<double, 3, 15> jacobian =
      Eigen::Matrix<double, 3, 15>::Zero();
  jacobian.block<3, 3>(0, 0) = -rotation * skew(body_point);
  jacobian.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d innovation =
      jacobian * covariance_ * jacobian.transpose();
  innovation += measurement_noise * measurement_noise *
                Eigen::Matrix3d::Identity();
  if (!innovation.allFinite()) return false;

  const Eigen::Matrix<double, 15, 3> gain =
      covariance_ * jacobian.transpose() * innovation.inverse();
  const Eigen::Matrix<double, 15, 1> correction = gain * residual;
  if (!applyErrorState(correction)) return false;

  const Eigen::Matrix<double, 15, 15> identity =
      Eigen::Matrix<double, 15, 15>::Identity();
  // Joseph-form update keeps the result symmetric positive semi-definite
  // even for compressed or partially corrected linearizations.
  const Eigen::Matrix<double, 15, 15> closed_loop =
      identity - gain * jacobian;
  covariance_ =
      closed_loop * covariance_ * closed_loop.transpose() +
      gain *
          (measurement_noise * measurement_noise *
           Eigen::Matrix3d::Identity()) *
          gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  return covariance_.allFinite();
}

bool Eskf::updateVelocity(const Eigen::Vector3d& measured_velocity,
                          double measurement_noise) {
  if (!initialized_ || !measured_velocity.allFinite() ||
      measurement_noise <= 0.0) {
    return false;
  }

  const Eigen::Vector3d residual = measured_velocity - state_.v_wb;
  Eigen::Matrix<double, 3, 15> jacobian =
      Eigen::Matrix<double, 3, 15>::Zero();
  jacobian.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d innovation =
      jacobian * covariance_ * jacobian.transpose();
  innovation += measurement_noise * measurement_noise *
                Eigen::Matrix3d::Identity();
  if (!innovation.allFinite()) return false;

  const Eigen::Matrix<double, 15, 3> gain =
      covariance_ * jacobian.transpose() * innovation.inverse();
  const Eigen::Matrix<double, 15, 1> correction = gain * residual;
  if (!applyErrorState(correction)) return false;

  const Eigen::Matrix<double, 15, 15> identity =
      Eigen::Matrix<double, 15, 15>::Identity();
  // Joseph-form update keeps the result symmetric positive semi-definite
  // even for compressed or partially corrected linearizations.
  const Eigen::Matrix<double, 15, 15> closed_loop =
      identity - gain * jacobian;
  covariance_ =
      closed_loop * covariance_ * closed_loop.transpose() +
      gain *
          (measurement_noise * measurement_noise *
           Eigen::Matrix3d::Identity()) *
          gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  return covariance_.allFinite();
}

void Eskf::propagateSegment(const ImuMeasurement& minus,
                            const ImuMeasurement& plus) {
  const double start_time =
      std::max(state_.timestamp, minus.timestamp);
  const double end_time = plus.timestamp;
  if (end_time <= start_time) return;
  const double dt = end_time - start_time;

  // Midpoint integration with the current bias estimates. This mirrors the
  // discrete propagator used by OpenVINS' fast-state path.
  const Eigen::Vector3d angular_rate =
      0.5 * (minus.angular_velocity + plus.angular_velocity) -
      state_.bias_gyro;
  const Eigen::Vector3d acceleration =
      0.5 * (minus.acceleration + plus.acceleration) - state_.bias_accel;

  const Eigen::Matrix3d rotation = state_.q_wb.toRotationMatrix();
  const Eigen::Quaterniond rotation_delta =
      exponentialMap(angular_rate * dt);
  const Eigen::Vector3d old_position = state_.p_wb;
  const Eigen::Vector3d old_velocity = state_.v_wb;

  state_.q_wb = (state_.q_wb * rotation_delta).normalized();
  state_.v_wb = old_velocity +
                (rotation * acceleration + options_.gravity) * dt;
  state_.p_wb = old_position + old_velocity * dt +
                0.5 * (rotation * acceleration + options_.gravity) * dt * dt;

  Eigen::Matrix<double, 15, 15> transition =
      Eigen::Matrix<double, 15, 15>::Identity();
  transition.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() - skew(angular_rate) * dt;
  transition.block<3, 3>(0, 9) = -Eigen::Matrix3d::Identity() * dt;
  transition.block<3, 3>(3, 0) =
      -0.5 * rotation * skew(acceleration) * dt * dt;
  transition.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  transition.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
  transition.block<3, 3>(3, 12) =
      -0.5 * rotation * dt * dt;
  transition.block<3, 3>(6, 0) =
      -rotation * skew(acceleration) * dt;
  transition.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  transition.block<3, 3>(6, 12) = -rotation * dt;

  Eigen::Matrix<double, 15, 12> noise_jacobian =
      Eigen::Matrix<double, 15, 12>::Zero();
  noise_jacobian.block<3, 3>(0, 0) =
      -Eigen::Matrix3d::Identity() * dt;
  noise_jacobian.block<3, 3>(3, 3) =
      -0.5 * rotation * dt * dt;
  noise_jacobian.block<3, 3>(6, 3) =
      -rotation * dt;
  noise_jacobian.block<3, 3>(9, 6) =
      Eigen::Matrix3d::Identity() * dt;
  noise_jacobian.block<3, 3>(12, 9) =
      Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double, 12, 12> continuous_noise =
      Eigen::Matrix<double, 12, 12>::Zero();
  continuous_noise.block<3, 3>(0, 0) =
      options_.gyroscope_noise * options_.gyroscope_noise *
      Eigen::Matrix3d::Identity();
  continuous_noise.block<3, 3>(3, 3) =
      options_.accelerometer_noise * options_.accelerometer_noise *
      Eigen::Matrix3d::Identity();
  continuous_noise.block<3, 3>(6, 6) =
      options_.gyroscope_bias_noise * options_.gyroscope_bias_noise *
      Eigen::Matrix3d::Identity();
  continuous_noise.block<3, 3>(9, 9) =
      options_.accelerometer_bias_noise * options_.accelerometer_bias_noise *
      Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 15, 15> discrete_noise =
      noise_jacobian * continuous_noise * noise_jacobian.transpose() * dt;
  discrete_noise =
      0.5 * (discrete_noise + discrete_noise.transpose());

  covariance_ = transition * covariance_ * transition.transpose() +
                discrete_noise;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  state_.timestamp = end_time;
}

bool Eskf::applyErrorState(
    const Eigen::Matrix<double, 15, 1>& correction) {
  if (!correction.allFinite()) return false;

  const Eigen::Vector3d theta = correction.segment<3>(0);
  const Eigen::Quaterniond orientation_correction = exponentialMap(theta);
  state_.q_wb = (state_.q_wb * orientation_correction).normalized();
  state_.p_wb += correction.segment<3>(3);
  state_.v_wb += correction.segment<3>(6);
  state_.bias_gyro += correction.segment<3>(9);
  state_.bias_accel += correction.segment<3>(12);
  return isFinite(state_.q_wb) && isFinite(state_.p_wb) &&
         isFinite(state_.v_wb) && isFinite(state_.bias_gyro) &&
         isFinite(state_.bias_accel);
}

}  // namespace sphere_vio
