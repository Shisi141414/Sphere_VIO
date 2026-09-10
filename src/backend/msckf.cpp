#include "sphere_vio/backend/msckf.hpp"
#include "sphere_vio/geometry/triangulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <Eigen/LU>
#include <Eigen/QR>

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
  return Eigen::Quaterniond(std::cos(half_angle),
                            std::sin(half_angle) * axis_angle.x() / angle,
                            std::sin(half_angle) * axis_angle.y() / angle,
                            std::sin(half_angle) * axis_angle.z() / angle)
      .normalized();
}

bool finiteVector(const Eigen::Vector3d& vector) {
  return vector.allFinite();
}

bool finiteQuaternion(const Eigen::Quaterniond& quaternion) {
  return quaternion.coeffs().allFinite();
}

// Wilson-Hilferty approximation for the one-sided chi-square quantile. This
// avoids a Boost link dependency and is accurate enough for feature gating.
double chiSquareQuantile(double degrees_of_freedom, double probability) {
  if (!std::isfinite(degrees_of_freedom) || degrees_of_freedom <= 0.0 ||
      probability <= 0.0 || probability >= 1.0) {
    return std::numeric_limits<double>::infinity();
  }
  // Normal quantile table is unnecessary for a fixed feature gate: 0.95 and
  // 0.99 are the only supported probabilities in configuration.
  const double z =
      probability >= 0.99 ? 2.3263478740408408 : 1.6448536269514722;
  const double correction = 2.0 / (9.0 * degrees_of_freedom);
  const double value =
      degrees_of_freedom *
      std::pow(1.0 - correction + z * std::sqrt(correction), 3.0);
  return std::isfinite(value) ? value
                              : std::numeric_limits<double>::infinity();
}

const MsckfClone* findClone(const std::map<Timestamp, MsckfClone>& clones,
                            Timestamp timestamp) {
  const auto exact = clones.find(timestamp);
  if (exact != clones.end()) return &exact->second;

  const MsckfClone* best = nullptr;
  double best_delta = 1e-3;
  for (const auto& entry : clones) {
    const double delta = std::abs(entry.first - timestamp);
    if (delta <= best_delta) {
      best_delta = delta;
      best = &entry.second;
    }
  }
  return best;
}

}  // namespace

MsckfFeatureAccumulator::MsckfFeatureAccumulator(
    std::size_t maximum_observations)
    : maximum_observations_(maximum_observations) {
  if (maximum_observations_ == 0U) maximum_observations_ = 40U;
}

void MsckfFeatureAccumulator::add(CameraId camera_id, FeatureId feature_id,
                                  Timestamp timestamp,
                                  const Eigen::Vector2d& pixel,
                                  std::uint64_t frame_index) {
  if (feature_id == 0U || !std::isfinite(timestamp) ||
      !pixel.allFinite()) {
    return;
  }
  const TemporalFeatureKey key{camera_id, feature_id};
  TrackState& state = tracks_[key];
  state.last_frame_index = frame_index;

  if (!state.observations.empty() &&
      state.observations.back().timestamp == timestamp) {
    state.observations.back().pixel = pixel;
    return;
  }
  MsckfObservation observation;
  observation.timestamp = timestamp;
  observation.camera_id = camera_id;
  observation.pixel = pixel;
  state.observations.push_back(observation);
  if (state.observations.size() > maximum_observations_) {
    state.observations.erase(
        state.observations.begin(),
        state.observations.begin() +
            (state.observations.size() - maximum_observations_));
  }
}

void MsckfFeatureAccumulator::prune(
    std::uint64_t current_frame_index,
    std::uint64_t maximum_frames_without_observation) {
  for (auto iterator = tracks_.begin(); iterator != tracks_.end();) {
    if (current_frame_index > iterator->second.last_frame_index &&
        current_frame_index - iterator->second.last_frame_index >
            maximum_frames_without_observation) {
      iterator = tracks_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

const std::vector<MsckfObservation>*
MsckfFeatureAccumulator::observations(CameraId camera_id,
                                      FeatureId feature_id) const {
  const auto found = tracks_.find(TemporalFeatureKey{camera_id, feature_id});
  return found == tracks_.end() ? nullptr : &found->second.observations;
}

std::vector<TemporalFeatureKey> MsckfFeatureAccumulator::keys() const {
  std::vector<TemporalFeatureKey> result;
  result.reserve(tracks_.size());
  for (const auto& entry : tracks_) result.push_back(entry.first);
  return result;
}

Msckf::Msckf(MsckfOptions options) : options_(std::move(options)) {
  options_.gravity =
      Eigen::Vector3d(0.0, 0.0, -options_.gravity_magnitude);
}

bool Msckf::initialize(const ImuMeasurement& measurement) {
  if (!measurement.acceleration.allFinite() ||
      !measurement.angular_velocity.allFinite()) {
    return false;
  }
  state_ = MsckfCurrentState();
  state_.timestamp = measurement.timestamp;
  covariance_ = options_.initial_covariance;
  clones_.clear();
  initialized_ = true;
  return true;
}

bool Msckf::propagate(const std::vector<ImuMeasurement>& measurements,
                      Timestamp end_time) {
  if (!initialized_ || measurements.empty() || end_time <= state_.timestamp) {
    return initialized_;
  }

  if (measurements.front().timestamp > state_.timestamp) {
    propagateSegment(measurements.front(), measurements.front());
  }
  for (std::size_t index = 0; index + 1U < measurements.size(); ++index) {
    if (measurements[index + 1U].timestamp <= state_.timestamp) continue;
    if (measurements[index].timestamp >= end_time) break;
    propagateSegment(measurements[index], measurements[index + 1U]);
  }
  if (state_.timestamp < end_time) {
    propagateSegment(measurements.back(), measurements.back());
  }

  state_.timestamp = end_time;
  return finiteQuaternion(state_.q_wb) && finiteVector(state_.p_wb) &&
         finiteVector(state_.v_wb) && covariance_.allFinite();
}

bool Msckf::augmentClone(Timestamp timestamp) {
  if (!initialized_ || clones_.find(timestamp) != clones_.end()) return false;

  const int old_size = static_cast<int>(covariance_.rows());
  const int new_size = old_size + 6;
  Eigen::MatrixXd new_covariance = Eigen::MatrixXd::Zero(new_size, new_size);
  new_covariance.topLeftCorner(old_size, old_size) = covariance_;
  new_covariance.block(old_size, old_size, 6, 6) =
      covariance_.block(0, 0, 6, 6);
  new_covariance.block(0, old_size, old_size, 6) =
      covariance_.block(0, 0, old_size, 6);
  new_covariance.block(old_size, 0, 6, old_size) =
      covariance_.block(0, 0, 6, old_size);

  MsckfClone clone;
  clone.timestamp = timestamp;
  clone.q_wb = state_.q_wb;
  clone.p_wb = state_.p_wb;
  clone.index = old_size;
  clones_.emplace(timestamp, clone);
  covariance_ = std::move(new_covariance);
  return true;
}

bool Msckf::update(const std::vector<MsckfFeature>& features,
                   const CameraRig& camera_rig) {
  if (!initialized_ || clones_.size() < 2U) return false;

  std::vector<Eigen::MatrixXd> projected_H;
  std::vector<Eigen::VectorXd> projected_residual;
  const int state_size = static_cast<int>(covariance_.rows());

  for (const MsckfFeature& feature : features) {
    Eigen::Vector3d point_w;
    std::vector<FeatureMeasurement> measurements;
    if (!buildFeatureMeasurements(feature, camera_rig, &point_w,
                                  &measurements) ||
        measurements.size() < 2U) {
      continue;
    }

    const int measurement_count = static_cast<int>(measurements.size());
    Eigen::MatrixXd H_f = Eigen::MatrixXd::Zero(2 * measurement_count, 3);
    Eigen::MatrixXd H_x =
        Eigen::MatrixXd::Zero(2 * measurement_count, state_size);
    Eigen::VectorXd residual = Eigen::VectorXd::Zero(2 * measurement_count);

    int row = 0;
    for (const FeatureMeasurement& measurement : measurements) {
      const auto clone_it = clones_.find(measurement.timestamp);
      if (clone_it == clones_.end()) continue;
      const MsckfClone& clone = clone_it->second;

      Eigen::Vector2d predicted;
      Eigen::Matrix<double, 2, 3> jacobian_feature;
      Eigen::Matrix<double, 2, 6> jacobian_clone;
      if (!computeProjectionAndJacobians(
              camera_rig, clone, measurement, &predicted, &jacobian_feature,
              &jacobian_clone)) {
        continue;
      }

      residual.segment<2>(row) = measurement.pixel - predicted;
      H_f.block<2, 3>(row, 0) = jacobian_feature;
      H_x.block<2, 6>(row, clone.index) = jacobian_clone;
      row += 2;
    }

    // Two observations are enough for a monocular feature: after removing the
    // three feature-position degrees of freedom there is one useful residual
    // row. Requiring three observations would discard most temporal tracks.
    if (row < 4) continue;
    H_f.conservativeResize(row, 3);
    H_x.conservativeResize(row, state_size);
    residual.conservativeResize(row);

    // Project onto the left nullspace of the feature Jacobian so the update
    // does not depend on the triangulated feature position.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H_f, Eigen::ComputeFullU);
    int rank = 0;
    const double tolerance = 1e-10 * svd.singularValues().maxCoeff();
    for (int i = 0; i < svd.singularValues().size(); ++i) {
      if (svd.singularValues()(i) > tolerance) ++rank;
    }
    if (rank >= H_f.cols()) continue;

    Eigen::MatrixXd left_nullspace =
        svd.matrixU().block(0, rank, H_f.rows(), H_f.rows() - rank).transpose();
    const Eigen::MatrixXd feature_H = left_nullspace * H_x;
    const Eigen::VectorXd feature_residual = left_nullspace * residual;
    if (!feature_H.allFinite() || !feature_residual.allFinite()) continue;

    // Per-feature innovation gate rejects stale or outlier temporal tracks
    // before they can poison the joint update.
    const int degrees_of_freedom = feature_residual.rows();
    if (degrees_of_freedom > 0) {
      const Eigen::MatrixXd feature_innovation =
          feature_H * covariance_ * feature_H.transpose() +
          options_.pixel_noise * options_.pixel_noise *
              Eigen::MatrixXd::Identity(degrees_of_freedom,
                                        degrees_of_freedom);
      if (!feature_innovation.allFinite()) continue;
      const Eigen::VectorXd weighted =
          feature_innovation.ldlt().solve(feature_residual);
      const double mahalanobis_squared =
          feature_residual.dot(weighted);
      const double threshold = chiSquareQuantile(
          degrees_of_freedom, options_.feature_chi_square_probability);
      if (!std::isfinite(mahalanobis_squared) ||
          mahalanobis_squared > threshold) {
        continue;
      }
    }

    projected_H.push_back(std::move(feature_H));
    projected_residual.push_back(std::move(feature_residual));
  }

  if (projected_H.empty()) return false;

  int total_rows = 0;
  for (const auto& matrix : projected_H) total_rows += matrix.rows();
  Eigen::MatrixXd H_all = Eigen::MatrixXd::Zero(total_rows, state_size);
  Eigen::VectorXd residual_all = Eigen::VectorXd::Zero(total_rows);
  int offset = 0;
  for (std::size_t i = 0; i < projected_H.size(); ++i) {
    H_all.block(offset, 0, projected_H[i].rows(), state_size) = projected_H[i];
    residual_all.segment(offset, projected_residual[i].rows()) =
        projected_residual[i];
    offset += projected_H[i].rows();
  }

  // Measurement compression via QR, keeping only the information-bearing rows.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(H_all);
  const int qr_rank = qr.rank();
  Eigen::MatrixXd H_compressed = H_all;
  Eigen::VectorXd residual_compressed = residual_all;
  if (qr_rank < total_rows) {
    const Eigen::MatrixXd q = qr.matrixQ();
    H_compressed = q.leftCols(qr_rank).transpose() * H_all;
    residual_compressed = q.leftCols(qr_rank).transpose() * residual_all;
  }

  const Eigen::MatrixXd measurement_noise =
      options_.pixel_noise * options_.pixel_noise *
      Eigen::MatrixXd::Identity(residual_compressed.rows(),
                                residual_compressed.rows());
  const Eigen::MatrixXd innovation =
      H_compressed * covariance_ * H_compressed.transpose() +
      measurement_noise;
  if (!innovation.allFinite()) return false;

  const Eigen::MatrixXd gain =
      covariance_ * H_compressed.transpose() * innovation.inverse();
  const Eigen::VectorXd correction = gain * residual_compressed;
  if (!applyErrorState(correction)) return false;

  const Eigen::MatrixXd identity =
      Eigen::MatrixXd::Identity(state_size, state_size);
  covariance_ = (identity - gain * H_compressed) * covariance_;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  return covariance_.allFinite();
}

void Msckf::marginalizeOldestClone() {
  if (static_cast<int>(clones_.size()) <= options_.maximum_clones) return;

  auto oldest = clones_.begin();
  const int index = oldest->second.index;
  const int before = index;
  const int after = static_cast<int>(covariance_.rows()) - index - 6;
  Eigen::MatrixXd new_covariance =
      Eigen::MatrixXd::Zero(covariance_.rows() - 6, covariance_.rows() - 6);

  new_covariance.topLeftCorner(before, before) =
      covariance_.topLeftCorner(before, before);
  new_covariance.topRightCorner(before, after) =
      covariance_.block(0, index + 6, before, after);
  new_covariance.bottomLeftCorner(after, before) =
      new_covariance.topRightCorner(before, after).transpose();
  new_covariance.bottomRightCorner(after, after) =
      covariance_.block(index + 6, index + 6, after, after);

  covariance_ = std::move(new_covariance);
  clones_.erase(oldest);
  for (auto& entry : clones_) {
    if (entry.second.index > index) entry.second.index -= 6;
  }
}

bool Msckf::buildFeatureMeasurements(
    const MsckfFeature& feature, const CameraRig& camera_rig,
    Eigen::Vector3d* point_w,
    std::vector<FeatureMeasurement>* measurements) const {
  if (!point_w || !measurements) return false;
  measurements->clear();
  if (feature.observations.size() < 2U) return false;

  for (const MsckfObservation& observation : feature.observations) {
    const MsckfClone* clone = findClone(clones_, observation.timestamp);
    if (!clone) continue;
    FeatureMeasurement measurement;
    measurement.timestamp = clone->timestamp;
    measurement.camera_id = observation.camera_id;
    measurement.pixel = observation.pixel;
    measurement.p_w = Eigen::Vector3d::Zero();
    measurement.clone_index = clone->index;
    measurements->push_back(measurement);
  }
  if (measurements->size() < 2U) return false;

  if (!estimateFeaturePosition(*measurements, camera_rig, point_w)) {
    return false;
  }
  for (FeatureMeasurement& measurement : *measurements) {
    measurement.p_w = *point_w;
  }
  return true;
}

bool Msckf::estimateFeaturePosition(
    const std::vector<FeatureMeasurement>& measurements,
    const CameraRig& camera_rig, Eigen::Vector3d* point_w) const {
  if (!point_w || measurements.size() < 2U) return false;

  const MsckfClone* anchor = findClone(clones_, measurements.front().timestamp);
  const MsckfClone* far =
      findClone(clones_, measurements.back().timestamp);
  if (!anchor || !far || anchor->timestamp == far->timestamp) return false;

  Eigen::Vector3d anchor_bearing_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d anchor_origin_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d far_bearing_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d far_origin_w = Eigen::Vector3d::Zero();

  const auto bearingInWorld =
      [&camera_rig](const FeatureMeasurement& measurement,
                    const MsckfClone& clone, Eigen::Vector3d* bearing,
                    Eigen::Vector3d* origin) {
        const RigCamera* camera = camera_rig.camera(measurement.camera_id);
        if (!camera || !camera->model) return false;
        Eigen::Vector3d bearing_c;
        if (!camera->model->unproject(measurement.pixel, &bearing_c)) {
          return false;
        }
        const Eigen::Vector3d bearing_b = camera->R_b_c * bearing_c;
        *bearing = clone.q_wb * bearing_b;
        *origin = clone.p_wb + clone.q_wb * camera->t_b_c;
        return bearing->allFinite() && origin->allFinite();
      };

  if (!bearingInWorld(measurements.front(), *anchor, &anchor_bearing_w,
                      &anchor_origin_w) ||
      !bearingInWorld(measurements.back(), *far, &far_bearing_w,
                      &far_origin_w)) {
    return false;
  }

  TriangulationOptions options;
  options.minimum_baseline = 1e-4;
  options.minimum_ray_angle = 5e-4;
  options.minimum_depth = 0.1;
  options.maximum_closest_ray_distance =
      std::numeric_limits<double>::infinity();
  options.maximum_angular_reprojection_error =
      std::numeric_limits<double>::infinity();

  TriangulationResult triangulation;
  Eigen::Vector3d estimate;
  if (triangulateRays(anchor_origin_w, anchor_bearing_w, far_origin_w,
                      far_bearing_w, options, &triangulation)) {
    estimate = triangulation.point_common;
  } else {
    // Fallback for nearly parallel rays or degenerate motion: initialize on
    // the anchor ray and let Gauss-Newton correct the depth.
    estimate = anchor_origin_w + 8.0 * anchor_bearing_w;
  }
  if (!estimate.allFinite() || estimate.norm() > 500.0) return false;

  for (int iteration = 0; iteration < 5; ++iteration) {
    const int rows = 2 * static_cast<int>(measurements.size());
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(rows, 3);
    Eigen::VectorXd residual = Eigen::VectorXd::Zero(rows);
    int row = 0;
    bool projected_all = true;

    for (const FeatureMeasurement& measurement : measurements) {
      const auto clone_it = clones_.find(measurement.timestamp);
      if (clone_it == clones_.end()) {
        projected_all = false;
        break;
      }
      FeatureMeasurement working = measurement;
      working.p_w = estimate;
      Eigen::Vector2d predicted;
      Eigen::Matrix<double, 2, 3> jacobian_feature;
      Eigen::Matrix<double, 2, 6> jacobian_clone;
      if (!computeProjectionAndJacobians(
              camera_rig, clone_it->second, working, &predicted,
              &jacobian_feature, &jacobian_clone)) {
        projected_all = false;
        break;
      }
      residual.segment<2>(row) = measurement.pixel - predicted;
      jacobian.block<2, 3>(row, 0) = jacobian_feature;
      row += 2;
    }
    if (!projected_all || row == 0) break;

    jacobian.conservativeResize(row, 3);
    residual.conservativeResize(row);

    const Eigen::Matrix3d normal = jacobian.transpose() * jacobian;
    double damping = 1e-3 * std::max(1.0, normal.diagonal().maxCoeff());
    const Eigen::Matrix3d system =
        normal + damping * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d gradient = jacobian.transpose() * residual;
    const Eigen::Vector3d step = system.ldlt().solve(gradient);
    if (!step.allFinite()) break;
    estimate -= step;
    if (!estimate.allFinite() || estimate.norm() > 500.0) break;
  }

  *point_w = estimate;
  return estimate.allFinite() && estimate.norm() <= 500.0;
}

void Msckf::propagateSegment(const ImuMeasurement& minus,
                             const ImuMeasurement& plus) {
  const double start_time = std::max(state_.timestamp, minus.timestamp);
  const double end_time = plus.timestamp;
  if (end_time <= start_time) return;
  const double dt = end_time - start_time;

  const Eigen::Vector3d angular_rate =
      0.5 * (minus.angular_velocity + plus.angular_velocity) -
      state_.bias_gyro;
  const Eigen::Vector3d acceleration =
      0.5 * (minus.acceleration + plus.acceleration) - state_.bias_accel;
  const Eigen::Matrix3d rotation = state_.q_wb.toRotationMatrix();

  const Eigen::Vector3d old_position = state_.p_wb;
  const Eigen::Vector3d old_velocity = state_.v_wb;
  state_.q_wb =
      (state_.q_wb * exponentialMap(angular_rate * dt)).normalized();
  state_.v_wb =
      old_velocity + (rotation * acceleration + options_.gravity) * dt;
  state_.p_wb =
      old_position + old_velocity * dt +
      0.5 * (rotation * acceleration + options_.gravity) * dt * dt;

  Eigen::Matrix<double, 15, 15> transition =
      Eigen::Matrix<double, 15, 15>::Identity();
  transition.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() - skew(angular_rate) * dt;
  transition.block<3, 3>(0, 9) = -Eigen::Matrix3d::Identity() * dt;
  transition.block<3, 3>(3, 0) =
      -0.5 * rotation * skew(acceleration) * dt * dt;
  transition.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
  transition.block<3, 3>(3, 12) = -0.5 * rotation * dt * dt;
  transition.block<3, 3>(6, 0) = -rotation * skew(acceleration) * dt;
  transition.block<3, 3>(6, 12) = -rotation * dt;

  Eigen::Matrix<double, 15, 12> noise_jacobian =
      Eigen::Matrix<double, 15, 12>::Zero();
  noise_jacobian.block<3, 3>(0, 0) =
      -Eigen::Matrix3d::Identity() * dt;
  noise_jacobian.block<3, 3>(3, 3) =
      -0.5 * rotation * dt * dt;
  noise_jacobian.block<3, 3>(6, 3) = -rotation * dt;
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
  const Eigen::Matrix<double, 15, 15> discrete_noise =
      noise_jacobian * continuous_noise * noise_jacobian.transpose() * dt;

  const int state_size = static_cast<int>(covariance_.rows());
  Eigen::MatrixXd propagated = covariance_;
  propagated.topLeftCorner(15, 15) =
      transition * covariance_.topLeftCorner(15, 15) *
          transition.transpose() +
      discrete_noise;
  if (state_size > 15) {
    propagated.topRightCorner(15, state_size - 15) =
        transition * covariance_.topRightCorner(15, state_size - 15);
    propagated.bottomLeftCorner(state_size - 15, 15) =
        propagated.topRightCorner(15, state_size - 15).transpose();
  }
  propagated = 0.5 * (propagated + propagated.transpose());
  covariance_ = std::move(propagated);
  state_.timestamp = end_time;
}

bool Msckf::applyErrorState(const Eigen::VectorXd& correction) {
  if (!correction.allFinite() || correction.size() != covariance_.rows()) {
    return false;
  }

  const Eigen::Vector3d theta = correction.segment<3>(0);
  state_.q_wb =
      (state_.q_wb * exponentialMap(theta)).normalized();
  state_.p_wb += correction.segment<3>(3);
  state_.v_wb += correction.segment<3>(6);
  state_.bias_gyro += correction.segment<3>(9);
  state_.bias_accel += correction.segment<3>(12);

  for (auto& entry : clones_) {
    MsckfClone& clone = entry.second;
    const Eigen::Vector3d clone_theta =
        correction.segment<3>(clone.index);
    clone.q_wb =
        (clone.q_wb * exponentialMap(clone_theta)).normalized();
    clone.p_wb += correction.segment<3>(clone.index + 3);
  }

  bool finite = finiteQuaternion(state_.q_wb) && finiteVector(state_.p_wb) &&
                finiteVector(state_.v_wb) && finiteVector(state_.bias_gyro) &&
                finiteVector(state_.bias_accel);
  for (const auto& entry : clones_) {
    finite = finite && finiteQuaternion(entry.second.q_wb) &&
             finiteVector(entry.second.p_wb);
  }
  return finite;
}

bool Msckf::computeProjectionAndJacobians(
    const CameraRig& rig, const MsckfClone& clone,
    const FeatureMeasurement& measurement, Eigen::Vector2d* predicted,
    Eigen::Matrix<double, 2, 3>* jacobian_feature,
    Eigen::Matrix<double, 2, 6>* jacobian_clone) const {
  if (!predicted || !jacobian_feature || !jacobian_clone) return false;
  const RigCamera* camera = rig.camera(measurement.camera_id);
  if (!camera || !camera->model) return false;

  const Eigen::Matrix3d R_wb = clone.q_wb.toRotationMatrix();
  const Eigen::Vector3d p_b =
      R_wb.transpose() * (measurement.p_w - clone.p_wb);
  Eigen::Vector3d p_c;
  if (!rig.bodyPointToCamera(measurement.camera_id, p_b, &p_c)) return false;
  if (!camera->model->project(p_c, predicted)) return false;

  Eigen::Matrix<double, 2, 3> projection_jacobian;
  const double epsilon = 1e-6;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d p_plus = p_c;
    Eigen::Vector3d p_minus = p_c;
    p_plus(axis) += epsilon;
    p_minus(axis) -= epsilon;
    Eigen::Vector2d uv_plus;
    Eigen::Vector2d uv_minus;
    if (!camera->model->project(p_plus, &uv_plus) ||
        !camera->model->project(p_minus, &uv_minus)) {
      return false;
    }
    projection_jacobian.col(axis) =
        (uv_plus - uv_minus) / (2.0 * epsilon);
  }

  const Eigen::Matrix3d R_c_b = camera->R_b_c;
  const Eigen::Matrix<double, 2, 3> jacobian_body =
      projection_jacobian * R_c_b.transpose();
  *jacobian_feature = jacobian_body * R_wb.transpose();
  jacobian_clone->block<2, 3>(0, 0) = jacobian_body * skew(p_b);
  jacobian_clone->block<2, 3>(0, 3) = -jacobian_body * R_wb.transpose();
  return jacobian_feature->allFinite() && jacobian_clone->allFinite();
}

}  // namespace sphere_vio
