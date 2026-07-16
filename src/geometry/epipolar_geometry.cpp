#include "sphere_vio/geometry/epipolar_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>
#include <Eigen/LU>

namespace sphere_vio {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kMinimumVectorNorm = 1e-12;
constexpr double kMinimumBaseline = 1e-9;
constexpr double kMinimumPlaneSine = 1e-12;
constexpr double kRotationOrthogonalityTolerance = 1e-9;
constexpr double kRotationDeterminantTolerance = 1e-9;
constexpr std::size_t kMinimumGreatCircleSamples = 3U;

bool normalizeFiniteVector(const Eigen::Vector3d& input,
                           Eigen::Vector3d* normalized) {
  if (!normalized || !input.allFinite()) return false;
  const double norm = input.norm();
  if (!std::isfinite(norm) || norm <= kMinimumVectorNorm) return false;
  const Eigen::Vector3d result = input / norm;
  if (!result.allFinite()) return false;
  *normalized = result;
  return true;
}

bool isProperRotation(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite()) return false;
  const double orthogonality_error =
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();
  const double determinant = rotation.determinant();
  return std::isfinite(orthogonality_error) &&
         orthogonality_error <= kRotationOrthogonalityTolerance &&
         std::isfinite(determinant) &&
         std::abs(determinant - 1.0) <= kRotationDeterminantTolerance;
}

bool validateBaseline(const Eigen::Vector3d& translation,
                      Eigen::Vector3d* normalized_translation = nullptr) {
  if (!translation.allFinite()) return false;
  const double norm = translation.norm();
  if (!std::isfinite(norm) || norm <= kMinimumBaseline) return false;
  if (normalized_translation) {
    *normalized_translation = translation / norm;
  }
  return true;
}

bool normalizedPlaneNormal(const Eigen::Vector3d& bearing_source,
                           const Eigen::Matrix3d& R_target_source,
                           const Eigen::Vector3d& t_target_source,
                           Eigen::Vector3d* plane_normal_target) {
  if (!plane_normal_target || !isProperRotation(R_target_source)) {
    return false;
  }
  Eigen::Vector3d normalized_bearing;
  Eigen::Vector3d normalized_translation;
  if (!normalizeFiniteVector(bearing_source, &normalized_bearing) ||
      !validateBaseline(t_target_source, &normalized_translation)) {
    return false;
  }
  const Eigen::Vector3d rotated_bearing =
      R_target_source * normalized_bearing;
  const Eigen::Vector3d plane_normal =
      normalized_translation.cross(rotated_bearing);
  const double plane_norm = plane_normal.norm();
  if (!std::isfinite(plane_norm) || plane_norm <= kMinimumPlaneSine) {
    return false;
  }
  *plane_normal_target = plane_normal / plane_norm;
  return plane_normal_target->allFinite();
}

}  // namespace

bool relativeCameraPose(const RigCamera& source, const RigCamera& target,
                        RelativePose* relative_pose) {
  if (!relative_pose || !isProperRotation(source.R_b_c) ||
      !isProperRotation(target.R_b_c) || !source.t_b_c.allFinite() ||
      !target.t_b_c.allFinite()) {
    return false;
  }
  RelativePose result;
  result.R_target_source = target.R_b_c.transpose() * source.R_b_c;
  result.t_target_source =
      target.R_b_c.transpose() * (source.t_b_c - target.t_b_c);
  if (!isProperRotation(result.R_target_source) ||
      !validateBaseline(result.t_target_source)) {
    return false;
  }
  *relative_pose = result;
  return true;
}

bool epipolarAlgebraicResidual(
    const Eigen::Vector3d& bearing_source,
    const Eigen::Vector3d& bearing_target,
    const Eigen::Matrix3d& R_target_source,
    const Eigen::Vector3d& t_target_source, double* residual) {
  if (!residual) return false;
  Eigen::Vector3d normalized_source;
  Eigen::Vector3d normalized_target;
  Eigen::Vector3d unused_plane_normal;
  if (!normalizeFiniteVector(bearing_source, &normalized_source) ||
      !normalizeFiniteVector(bearing_target, &normalized_target) ||
      !normalizedPlaneNormal(normalized_source, R_target_source,
                             t_target_source, &unused_plane_normal)) {
    return false;
  }
  const double result = normalized_target.dot(
      t_target_source.cross(R_target_source * normalized_source));
  if (!std::isfinite(result)) return false;
  *residual = result;
  return true;
}

bool epipolarPlaneNormal(const Eigen::Vector3d& bearing_source,
                         const Eigen::Matrix3d& R_target_source,
                         const Eigen::Vector3d& t_target_source,
                         Eigen::Vector3d* plane_normal_target) {
  return normalizedPlaneNormal(bearing_source, R_target_source,
                               t_target_source, plane_normal_target);
}

bool epipolarAngularError(const Eigen::Vector3d& bearing_source,
                          const Eigen::Vector3d& bearing_target,
                          const Eigen::Matrix3d& R_target_source,
                          const Eigen::Vector3d& t_target_source,
                          double* angular_error) {
  if (!angular_error) return false;
  Eigen::Vector3d normalized_target;
  Eigen::Vector3d plane_normal;
  if (!normalizeFiniteVector(bearing_target, &normalized_target) ||
      !normalizedPlaneNormal(bearing_source, R_target_source,
                             t_target_source, &plane_normal)) {
    return false;
  }
  const double sine = std::max(
      -1.0, std::min(1.0, normalized_target.dot(plane_normal)));
  const double result = std::abs(std::asin(sine));
  if (!std::isfinite(result)) return false;
  *angular_error = result;
  return true;
}

bool symmetricEpipolarAngularError(
    const Eigen::Vector3d& bearing_source,
    const Eigen::Vector3d& bearing_target,
    const Eigen::Matrix3d& R_target_source,
    const Eigen::Vector3d& t_target_source, EpipolarError* error) {
  if (!error || !isProperRotation(R_target_source)) return false;
  EpipolarError result;
  if (!epipolarAngularError(bearing_source, bearing_target, R_target_source,
                            t_target_source, &result.forward)) {
    return false;
  }
  const Eigen::Matrix3d R_source_target = R_target_source.transpose();
  const Eigen::Vector3d t_source_target =
      -R_source_target * t_target_source;
  if (!epipolarAngularError(bearing_target, bearing_source, R_source_target,
                            t_source_target, &result.backward)) {
    return false;
  }
  result.maximum = std::max(result.forward, result.backward);
  result.average = 0.5 * (result.forward + result.backward);
  result.valid = true;
  *error = result;
  return true;
}

bool sampleGreatCircle(const Eigen::Vector3d& plane_normal,
                       std::size_t sample_count,
                       std::vector<Eigen::Vector3d>* bearings) {
  if (!bearings || sample_count < kMinimumGreatCircleSamples) return false;
  Eigen::Vector3d normal;
  if (!normalizeFiniteVector(plane_normal, &normal)) return false;

  Eigen::Vector3d helper = Eigen::Vector3d::UnitX();
  if (std::abs(normal.y()) <= std::abs(normal.x()) &&
      std::abs(normal.y()) <= std::abs(normal.z())) {
    helper = Eigen::Vector3d::UnitY();
  } else if (std::abs(normal.z()) <= std::abs(normal.x()) &&
             std::abs(normal.z()) <= std::abs(normal.y())) {
    helper = Eigen::Vector3d::UnitZ();
  }
  const Eigen::Vector3d first_basis = normal.cross(helper).normalized();
  const Eigen::Vector3d second_basis = normal.cross(first_basis).normalized();
  if (!first_basis.allFinite() || !second_basis.allFinite()) return false;

  std::vector<Eigen::Vector3d> result;
  result.reserve(sample_count);
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double theta = 2.0 * kPi * static_cast<double>(index) /
                         static_cast<double>(sample_count);
    const Eigen::Vector3d bearing =
        std::cos(theta) * first_basis + std::sin(theta) * second_basis;
    if (!bearing.allFinite()) return false;
    result.push_back(bearing.normalized());
  }
  *bearings = std::move(result);
  return true;
}

}  // namespace sphere_vio
