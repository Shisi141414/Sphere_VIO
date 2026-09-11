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

Eigen::Quaterniond rotationBetweenVectors(const Eigen::Vector3d& from,
                                          const Eigen::Vector3d& to) {
  const Eigen::Vector3d source = from.normalized();
  const Eigen::Vector3d target = to.normalized();
  const double dot = std::max(-1.0, std::min(1.0, source.dot(target)));
  if (dot > 1.0 - 1e-12) return Eigen::Quaterniond::Identity();
  if (dot < -1.0 + 1e-12) {
    Eigen::Vector3d axis = Eigen::Vector3d::UnitX().cross(source);
    if (axis.norm() < 1e-9) axis = Eigen::Vector3d::UnitY().cross(source);
    return Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, axis.normalized()));
  }
  const Eigen::Vector3d axis = source.cross(target);
  return Eigen::Quaterniond(1.0 + dot, axis.x(), axis.y(), axis.z())
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

std::vector<MsckfFeature> MsckfFeatureAccumulator::drainInactive(
    std::uint64_t current_frame_index,
    std::uint64_t maximum_frames_without_observation) {
  std::vector<MsckfFeature> drained;
  // Collect keys first, then erase, so iterator invalidation cannot skip a
  // track while iterating the map in place.
  std::vector<TemporalFeatureKey> inactive_keys;
  for (const auto& entry : tracks_) {
    const TrackState& track = entry.second;
    const bool inactive =
        track.last_frame_index <= current_frame_index &&
        current_frame_index - track.last_frame_index >
            maximum_frames_without_observation;
    if (inactive) inactive_keys.push_back(entry.first);
  }
  for (const TemporalFeatureKey& key : inactive_keys) {
    const auto found = tracks_.find(key);
    if (found == tracks_.end()) continue;
    // A track with one observation has no baseline and is useless to MSCKF;
    // it is still removed so it cannot be replayed later.
    if (found->second.observations.size() >= 2U) {
      MsckfFeature feature;
      feature.id = key.feature_id;
      feature.persistent_id = key.feature_id;
      feature.observations = found->second.observations;
      drained.push_back(std::move(feature));
    }
    tracks_.erase(found);
  }
  return drained;
}

std::vector<MsckfFeature> MsckfFeatureAccumulator::segmentBefore(
    Timestamp marginalization_time) {
  std::vector<MsckfFeature> drained;
  if (!std::isfinite(marginalization_time)) return drained;

  for (auto entry = tracks_.begin(); entry != tracks_.end();) {
    TrackState& track = entry->second;
    std::vector<MsckfObservation> oldest_segment;
    std::vector<MsckfObservation> retained;
    for (const MsckfObservation& observation : track.observations) {
      if (observation.timestamp <= marginalization_time) {
        oldest_segment.push_back(observation);
      } else {
        retained.push_back(observation);
      }
    }
    if (oldest_segment.size() >= 2U) {
      MsckfFeature feature;
      feature.id = entry->first.feature_id;
      feature.persistent_id = entry->first.feature_id;
      feature.observations = std::move(oldest_segment);
      drained.push_back(std::move(feature));
    }
    if (retained.empty()) {
      // The whole track was consumed (or had too few observations to be
      // usable). Either way it must not be replayed again.
      entry = tracks_.erase(entry);
    } else {
      track.observations = std::move(retained);
      ++entry;
    }
  }
  return drained;
}

Msckf::Msckf(MsckfOptions options) : options_(std::move(options)) {
  options_.gravity =
      Eigen::Vector3d(0.0, 0.0, -options_.gravity_magnitude);
}

Timestamp Msckf::oldestCloneTimestamp() const {
  if (clones_.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return clones_.begin()->first;
}

bool Msckf::marginalizationPending() const {
  return static_cast<int>(clones_.size()) > options_.maximum_clones;
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

bool Msckf::initialize(const std::vector<ImuMeasurement>& measurements,
                       Timestamp end_time) {
  if (initialized_) return true;
  for (const ImuMeasurement& measurement : measurements) {
    if (measurement.acceleration.allFinite() &&
        measurement.angular_velocity.allFinite()) {
      initialization_buffer_.push_back(measurement);
    }
  }
  if (initialization_buffer_.size() < 2U) return false;
  const double duration =
      initialization_buffer_.back().timestamp -
      initialization_buffer_.front().timestamp;
  if (duration < options_.initialization_duration ||
      initialization_buffer_.size() <
          options_.minimum_initialization_samples) {
    return false;
  }
  return initializeStatic(end_time);
}

bool Msckf::initializeStatic(Timestamp end_time) {
  Eigen::Vector3d mean_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_angular_velocity = Eigen::Vector3d::Zero();
  for (const ImuMeasurement& measurement : initialization_buffer_) {
    mean_acceleration += measurement.acceleration;
    mean_angular_velocity += measurement.angular_velocity;
  }
  const double count = static_cast<double>(initialization_buffer_.size());
  mean_acceleration /= count;
  mean_angular_velocity /= count;

  // Optional stationarity gate. A moving initialization window lets gravity
  // and body acceleration mix together; the recovered yaw/g world alignment
  // then has a large persistent error. Reject such windows before any state
  // is created and clear the buffer so a clean future window can be tried.
  if (options_.stationary_initialization_gate) {
    double acceleration_variance = 0.0;
    double gyroscope_variance = 0.0;
    for (const ImuMeasurement& measurement : initialization_buffer_) {
      acceleration_variance +=
          (measurement.acceleration - mean_acceleration).squaredNorm();
      gyroscope_variance +=
          (measurement.angular_velocity - mean_angular_velocity).squaredNorm();
    }
    const double acceleration_deviation =
        std::sqrt(acceleration_variance / count);
    const double gyroscope_deviation = std::sqrt(gyroscope_variance / count);
    if (acceleration_deviation > options_.maximum_accelerometer_deviation ||
        gyroscope_deviation > options_.maximum_gyroscope_deviation) {
      initialization_buffer_.clear();
      return false;
    }
  }

  const double gravity_norm = mean_acceleration.norm();
  if (!std::isfinite(gravity_norm) ||
      gravity_norm < 0.5 * options_.gravity_magnitude) {
    initialization_buffer_.clear();
    return false;
  }

  // Rotate the measured specific-force direction to the world +Z axis. This
  // leaves yaw unobservable, which is fine for the initial world frame.
  state_ = MsckfCurrentState();
  state_.q_wb = rotationBetweenVectors(
      mean_acceleration, Eigen::Vector3d(0.0, 0.0, 1.0));
  state_.bias_gyro = mean_angular_velocity;
  state_.bias_accel = mean_acceleration -
      state_.q_wb.conjugate() *
          (options_.gravity_magnitude * Eigen::Vector3d::UnitZ());
  state_.timestamp = initialization_buffer_.front().timestamp;
  covariance_ = options_.initial_covariance;
  clones_.clear();
  const std::vector<ImuMeasurement> initialization_window =
      initialization_buffer_;
  initialization_buffer_.clear();
  initialized_ = true;
  return propagate(initialization_window, end_time);
}

bool Msckf::propagate(const std::vector<ImuMeasurement>& measurements,
                      Timestamp end_time) {
  if (!initialized_ || measurements.empty() || end_time <= state_.timestamp) {
    return initialized_;
  }

  // The buffer supplies an interpolated left boundary at the current filter
  // time and an interpolated (or held) right boundary at the image time, so
  // consecutive-pair integration below covers the full interval. The loop
  // deliberately skips pairs that do not advance the filter clock rather than
  // replaying a segment that was already integrated.
  for (std::size_t index = 0U; index + 1U < measurements.size(); ++index) {
    const ImuMeasurement& minus = measurements[index];
    const ImuMeasurement& plus = measurements[index + 1U];
    if (plus.timestamp <= state_.timestamp) continue;
    if (minus.timestamp >= end_time) break;
    propagateSegment(minus, plus);
    if (state_.timestamp >= end_time) break;
  }

  // If no supplied sample reaches the requested image time (for example a
  // very short image gap or a buffered hold), finish the partial segment with
  // the last known values. This preserves strict timestamp monotonicity
  // instead of blindly assigning state_.timestamp = end_time.
  if (state_.timestamp < end_time) {
    const ImuMeasurement& minus = measurements.back();
    ImuMeasurement plus = minus;
    plus.timestamp = end_time;
    propagateSegment(minus, plus);
  }

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

bool Msckf::augmentLandmark(std::uint64_t persistent_id,
                            const Eigen::Vector3d& point_w) {
  if (!initialized_ || persistent_id == 0U || !point_w.allFinite() ||
      landmark_indices_.count(persistent_id) != 0U ||
      static_cast<int>(landmark_positions_.size()) >=
          options_.maximum_landmarks) {
    return false;
  }

  const int old_size = static_cast<int>(covariance_.rows());
  const int new_size = old_size + 3;
  Eigen::MatrixXd new_covariance = Eigen::MatrixXd::Zero(new_size, new_size);
  new_covariance.topLeftCorner(old_size, old_size) = covariance_;
  new_covariance.block<3, 3>(old_size, old_size) =
      0.25 * Eigen::Matrix3d::Identity();
  covariance_ = std::move(new_covariance);
  landmark_indices_[persistent_id] = old_size;
  landmark_positions_[persistent_id] = point_w;
  feature_positions_[persistent_id] = point_w;
  return true;
}

bool Msckf::update(const std::vector<MsckfFeature>& features,
                   const CameraRig& camera_rig) {
  if (!initialized_ || clones_.size() < 2U) return false;

  int state_size = static_cast<int>(covariance_.rows());
  std::vector<Eigen::MatrixXd> projected_H;
  std::vector<Eigen::VectorXd> projected_residual;

  for (const MsckfFeature& feature : features) {
    Eigen::Vector3d point_w;
    std::vector<FeatureMeasurement> measurements;
    if (!buildFeatureMeasurements(feature, camera_rig, &point_w,
                                  &measurements) ||
        measurements.size() < 2U) {
      continue;
    }
    ++update_considered_;

    const int measurement_count = static_cast<int>(measurements.size());
    const auto landmark = landmark_indices_.find(feature.persistent_id);
    const bool is_landmark = landmark != landmark_indices_.end();
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
      if (is_landmark) {
        H_x.block<2, 3>(row, landmark->second) = jacobian_feature;
      }
      row += 2;
    }

    // Two observations are enough for a monocular feature: after removing the
    // three feature-position degrees of freedom there is one useful residual
    // row. Requiring three observations would discard most temporal tracks.
    if (row < 4) continue;
    H_f.conservativeResize(row, 3);
    H_x.conservativeResize(row, state_size);
    residual.conservativeResize(row);

    Eigen::MatrixXd feature_H;
    Eigen::VectorXd feature_residual;
    if (is_landmark) {
      feature_H = H_x;
      feature_residual = residual;
    } else {
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
      feature_H = left_nullspace * H_x;
      feature_residual = left_nullspace * residual;
    }
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
        ++update_rejected_gate_;
        continue;
      }
    }

    projected_H.push_back(std::move(feature_H));
    projected_residual.push_back(std::move(feature_residual));
    ++update_accepted_;
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

  // Joseph-form covariance update. Unlike the short form
  //   P = (I - K H) P,
  // the Joseph form
  //   P = (I - K H) P (I - K H)^T + K R K^T
  // is symmetric and positive semi-definite even when the gain is only
  // approximately optimal or the state was partially corrected. With
  // compressed linearizations this keeps long runs from slowly losing
  // positive-definiteness and then rejecting every update.
  const Eigen::MatrixXd identity =
      Eigen::MatrixXd::Identity(state_size, state_size);
  const Eigen::MatrixXd closed_loop = identity - gain * H_compressed;
  covariance_ =
      closed_loop * covariance_ * closed_loop.transpose() +
      gain * measurement_noise * gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  return covariance_.allFinite();
}

double Msckf::featureReprojectionResidual(
    const MsckfFeature& feature, double time_offset,
    const CameraRig& camera_rig) const {
  if (clones_.size() < 2U || feature.observations.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  std::vector<FeatureMeasurement> measurements;
  for (const MsckfObservation& observation : feature.observations) {
    const MsckfClone* clone = findClone(
        clones_, observation.timestamp + time_offset);
    if (!clone) continue;
    FeatureMeasurement measurement;
    measurement.timestamp = clone->timestamp;
    measurement.camera_id = observation.camera_id;
    measurement.pixel = observation.pixel;
    measurement.clone_index = clone->index;
    measurement.p_w = Eigen::Vector3d::Zero();
    measurements.push_back(measurement);
  }
  if (measurements.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  Eigen::Vector3d point_w;
  if (!estimateFeaturePosition(measurements, camera_rig, nullptr, &point_w)) {
    return std::numeric_limits<double>::infinity();
  }

  double residual_squared = 0.0;
  for (const FeatureMeasurement& measurement : measurements) {
    const auto clone_it = clones_.find(measurement.timestamp);
    if (clone_it == clones_.end()) continue;
    FeatureMeasurement working = measurement;
    working.p_w = point_w;
    Eigen::Vector2d predicted;
    Eigen::Matrix<double, 2, 3> jacobian_feature;
    Eigen::Matrix<double, 2, 6> jacobian_clone;
    if (!computeProjectionAndJacobians(
            camera_rig, clone_it->second, working, &predicted,
            &jacobian_feature, &jacobian_clone)) {
      return std::numeric_limits<double>::infinity();
    }
    residual_squared += (measurement.pixel - predicted).squaredNorm();
  }
  return residual_squared;
}

bool Msckf::estimateTimeOffset(const std::vector<MsckfFeature>& features,
                               const CameraRig& camera_rig) {
  if (clones_.size() < 2U || features.empty()) return false;

  const std::size_t maximum_features_for_search =
      std::min<std::size_t>(features.size(), 120U);
  const auto evaluateOffset = [&](double offset) {
    double total = 0.0;
    std::size_t valid_features = 0U;
    for (std::size_t index = 0U; index < maximum_features_for_search;
         ++index) {
      const double residual =
          featureReprojectionResidual(features[index], offset, camera_rig);
      if (std::isfinite(residual)) {
        total += residual;
        ++valid_features;
      }
    }
    return valid_features == 0U
               ? std::numeric_limits<double>::infinity()
               : total / static_cast<double>(valid_features);
  };

  double best_offset = options_.time_offset_cam_imu;
  double best_cost = evaluateOffset(best_offset);
  for (double offset = -0.025; offset <= 0.025; offset += 0.005) {
    const double cost = evaluateOffset(offset);
    if (std::isfinite(cost) && cost < best_cost) {
      best_cost = cost;
      best_offset = offset;
    }
  }
  for (double offset = best_offset - 0.004;
       offset <= best_offset + 0.004; offset += 0.001) {
    const double cost = evaluateOffset(offset);
    if (std::isfinite(cost) && cost < best_cost) {
      best_cost = cost;
      best_offset = offset;
    }
  }

  options_.time_offset_cam_imu = best_offset;
  return std::isfinite(best_cost);
}

bool Msckf::estimateExtrinsicPerturbation(
    const std::vector<MsckfFeature>& features,
    const CameraRig& camera_rig) {
  if (clones_.size() < 2U || features.empty()) return false;

  const std::size_t maximum_features_for_search =
      std::min<std::size_t>(features.size(), 120U);
  const auto evaluate = [&]() {
    double total = 0.0;
    std::size_t valid_features = 0U;
    for (std::size_t index = 0U; index < maximum_features_for_search;
         ++index) {
      const double residual = featureReprojectionResidual(
          features[index], options_.time_offset_cam_imu, camera_rig);
      if (std::isfinite(residual)) {
        total += residual;
        ++valid_features;
      }
    }
    return valid_features == 0U
               ? std::numeric_limits<double>::infinity()
               : total / static_cast<double>(valid_features);
  };

  const MsckfOptions initial_options = options_;
  double best_cost = evaluate();
  constexpr double rotation_step = 0.002;
  constexpr double translation_step = 0.01;

  for (int pass = 0; pass < 2; ++pass) {
    for (int axis = 0; axis < 3; ++axis) {
      for (double sign : {-1.0, 1.0}) {
        options_.extrinsic_rotation_perturbation[axis] +=
            sign * rotation_step;
        double candidate_cost = evaluate();
        if (candidate_cost < best_cost) {
          best_cost = candidate_cost;
          continue;
        }
        options_.extrinsic_rotation_perturbation[axis] -=
            sign * rotation_step;
      }

      for (double sign : {-1.0, 1.0}) {
        options_.extrinsic_translation_perturbation[axis] +=
            sign * translation_step;
        double candidate_cost = evaluate();
        if (candidate_cost < best_cost) {
          best_cost = candidate_cost;
          continue;
        }
        options_.extrinsic_translation_perturbation[axis] -=
            sign * translation_step;
      }
    }
  }

  // Reject estimates that grew beyond a physically plausible small calibration
  // residual, leaving the initial calibration unchanged in that case.
  const Eigen::Vector3d rotation_delta =
      options_.extrinsic_rotation_perturbation -
      initial_options.extrinsic_rotation_perturbation;
  const Eigen::Vector3d translation_delta =
      options_.extrinsic_translation_perturbation -
      initial_options.extrinsic_translation_perturbation;
  if (rotation_delta.norm() > 0.02 ||
      translation_delta.norm() > 0.15) {
    options_.extrinsic_rotation_perturbation =
        initial_options.extrinsic_rotation_perturbation;
    options_.extrinsic_translation_perturbation =
        initial_options.extrinsic_translation_perturbation;
    return false;
  }
  return std::isfinite(best_cost);
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
  for (auto& entry : landmark_indices_) {
    if (entry.second > index) entry.second -= 6;
  }
}

bool Msckf::buildFeatureMeasurements(
    const MsckfFeature& feature, const CameraRig& camera_rig,
    Eigen::Vector3d* point_w,
    std::vector<FeatureMeasurement>* measurements) {
  if (!point_w || !measurements) return false;
  measurements->clear();
  if (feature.observations.size() < 2U) return false;

  for (const MsckfObservation& observation : feature.observations) {
    const MsckfClone* clone = findClone(
        clones_, observation.timestamp + options_.time_offset_cam_imu);
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

  const auto landmark = landmark_positions_.find(feature.persistent_id);
  if (landmark != landmark_positions_.end()) {
    *point_w = landmark->second;
    for (FeatureMeasurement& measurement : *measurements) {
      measurement.p_w = landmark->second;
    }
    return true;
  }

  const Eigen::Vector3d* prior = nullptr;
  if (feature.persistent_id != 0U) {
    const auto stored = feature_positions_.find(feature.persistent_id);
    if (stored != feature_positions_.end() &&
        stored->second.allFinite()) {
      prior = &stored->second;
    }
  }

  if (!estimateFeaturePosition(*measurements, camera_rig, prior, point_w)) {
    return false;
  }
  if (feature.persistent_id != 0U) {
    feature_positions_[feature.persistent_id] = *point_w;
  }
  for (FeatureMeasurement& measurement : *measurements) {
    measurement.p_w = *point_w;
  }
  return true;
}

bool Msckf::estimateFeaturePosition(
    const std::vector<FeatureMeasurement>& measurements,
    const CameraRig& camera_rig, const Eigen::Vector3d* prior,
    Eigen::Vector3d* point_w) const {
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
  if (prior && prior->allFinite()) {
    estimate = *prior;
  } else {
    if (triangulateRays(anchor_origin_w, anchor_bearing_w, far_origin_w,
                        far_bearing_w, options, &triangulation)) {
      estimate = triangulation.point_common;
    } else {
      // Fallback for nearly parallel rays or degenerate motion: initialize on
      // the anchor ray and let Gauss-Newton correct the depth.
      estimate = anchor_origin_w + 8.0 * anchor_bearing_w;
    }
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
  for (auto& entry : landmark_positions_) {
    const auto index = landmark_indices_.find(entry.first);
    if (index == landmark_indices_.end()) continue;
    entry.second += correction.segment<3>(index->second);
  }

  bool finite = finiteQuaternion(state_.q_wb) && finiteVector(state_.p_wb) &&
                finiteVector(state_.v_wb) && finiteVector(state_.bias_gyro) &&
                finiteVector(state_.bias_accel);
  for (const auto& entry : clones_) {
    finite = finite && finiteQuaternion(entry.second.q_wb) &&
             finiteVector(entry.second.p_wb);
  }
  for (const auto& entry : landmark_positions_) {
    finite = finite && finiteVector(entry.second);
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
  const Eigen::Vector3d perturbed_p_b =
      exponentialMap(options_.extrinsic_rotation_perturbation) * p_b +
      options_.extrinsic_translation_perturbation;
  Eigen::Vector3d p_c;
  if (!rig.bodyPointToCamera(measurement.camera_id, perturbed_p_b, &p_c)) {
    return false;
  }
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
