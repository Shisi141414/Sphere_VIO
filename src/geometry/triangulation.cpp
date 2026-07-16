#include "sphere_vio/geometry/triangulation.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/LU>
#include <Eigen/QR>

#include "sphere_vio/geometry/spherical_geometry.hpp"

namespace sphere_vio {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kMinimumVectorNorm = 1e-12;
constexpr double kRotationOrthogonalityTolerance = 1e-9;
constexpr double kRotationDeterminantTolerance = 1e-9;

bool normalizeFiniteVector(const Eigen::Vector3d& input,
                           Eigen::Vector3d* normalized) {
  if (!normalized || !input.allFinite()) return false;
  const double norm = input.norm();
  if (!std::isfinite(norm) || norm <= kMinimumVectorNorm) return false;
  const Eigen::Vector3d value = input / norm;
  if (!value.allFinite()) return false;
  *normalized = value;
  return true;
}

bool isProperRotation(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite()) return false;
  const double orthogonality_error =
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();
  const double determinant = rotation.determinant();
  return std::isfinite(orthogonality_error) &&
         orthogonality_error <= kRotationOrthogonalityTolerance &&
         std::isfinite(determinant) && determinant > 0.0 &&
         std::abs(determinant - 1.0) <= kRotationDeterminantTolerance;
}

bool nonNegativeOrPositiveInfinity(double value) {
  return !std::isnan(value) && value >= 0.0;
}

bool optionsAreValid(const TriangulationOptions& options) {
  return std::isfinite(options.minimum_baseline) &&
         options.minimum_baseline >= 0.0 &&
         std::isfinite(options.minimum_ray_angle) &&
         options.minimum_ray_angle >= 0.0 &&
         options.minimum_ray_angle <= 0.5 * kPi &&
         std::isfinite(options.minimum_depth) &&
         options.minimum_depth >= 0.0 &&
         nonNegativeOrPositiveInfinity(
             options.maximum_closest_ray_distance) &&
         nonNegativeOrPositiveInfinity(
             options.maximum_angular_reprojection_error);
}

bool finishFailure(TriangulationStatus status,
                   const TriangulationResult& partial,
                   TriangulationResult* result) {
  TriangulationResult failure = partial;
  failure.valid = false;
  failure.status = status;
  *result = failure;
  return false;
}

}  // namespace

const char* triangulationStatusName(TriangulationStatus status) {
  switch (status) {
    case TriangulationStatus::kSuccess:
      return "success";
    case TriangulationStatus::kInvalidInput:
      return "invalid_input";
    case TriangulationStatus::kZeroBaseline:
      return "zero_baseline";
    case TriangulationStatus::kNearlyParallelRays:
      return "nearly_parallel_rays";
    case TriangulationStatus::kNegativeDepth:
      return "negative_depth";
    case TriangulationStatus::kClosestDistanceTooLarge:
      return "closest_distance_too_large";
    case TriangulationStatus::kReprojectionErrorTooLarge:
      return "reprojection_error_too_large";
    case TriangulationStatus::kNumericalFailure:
      return "numerical_failure";
  }
  return "unknown";
}

bool triangulateRays(const Eigen::Vector3d& origin_1,
                     const Eigen::Vector3d& direction_1,
                     const Eigen::Vector3d& origin_2,
                     const Eigen::Vector3d& direction_2,
                     const TriangulationOptions& options,
                     TriangulationResult* result) {
  if (!result) return false;
  TriangulationResult working;
  if (!origin_1.allFinite() || !origin_2.allFinite() ||
      !optionsAreValid(options)) {
    return finishFailure(TriangulationStatus::kInvalidInput, working, result);
  }

  Eigen::Vector3d normalized_direction_1;
  Eigen::Vector3d normalized_direction_2;
  if (!normalizeFiniteVector(direction_1, &normalized_direction_1) ||
      !normalizeFiniteVector(direction_2, &normalized_direction_2)) {
    return finishFailure(TriangulationStatus::kInvalidInput, working, result);
  }

  working.baseline = (origin_2 - origin_1).norm();
  if (!std::isfinite(working.baseline)) {
    return finishFailure(TriangulationStatus::kNumericalFailure, working,
                         result);
  }
  if (working.baseline <= options.minimum_baseline) {
    return finishFailure(TriangulationStatus::kZeroBaseline, working, result);
  }

  const double direction_dot =
      std::max(-1.0, std::min(1.0, normalized_direction_1.dot(
                                      normalized_direction_2)));
  working.ray_angle = std::acos(direction_dot);
  const double line_angle =
      std::min(working.ray_angle, kPi - working.ray_angle);
  if (!std::isfinite(working.ray_angle) ||
      line_angle < options.minimum_ray_angle) {
    return finishFailure(TriangulationStatus::kNearlyParallelRays, working,
                         result);
  }

  Eigen::Matrix<double, 3, 2> ray_matrix;
  ray_matrix.col(0) = normalized_direction_1;
  ray_matrix.col(1) = -normalized_direction_2;
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 3, 2>> decomposition(
      ray_matrix);
  if (decomposition.rank() < 2) {
    return finishFailure(TriangulationStatus::kNearlyParallelRays, working,
                         result);
  }
  const Eigen::Vector2d depths = decomposition.solve(origin_2 - origin_1);
  if (!depths.allFinite()) {
    return finishFailure(TriangulationStatus::kNumericalFailure, working,
                         result);
  }
  working.depth_1 = depths.x();
  working.depth_2 = depths.y();
  if (working.depth_1 <= options.minimum_depth ||
      working.depth_2 <= options.minimum_depth) {
    return finishFailure(TriangulationStatus::kNegativeDepth, working,
                         result);
  }

  working.closest_point_1 =
      origin_1 + working.depth_1 * normalized_direction_1;
  working.closest_point_2 =
      origin_2 + working.depth_2 * normalized_direction_2;
  working.point_common =
      0.5 * (working.closest_point_1 + working.closest_point_2);
  working.closest_ray_distance =
      (working.closest_point_1 - working.closest_point_2).norm();
  if (!working.closest_point_1.allFinite() ||
      !working.closest_point_2.allFinite() || !working.point_common.allFinite() ||
      !std::isfinite(working.closest_ray_distance)) {
    return finishFailure(TriangulationStatus::kNumericalFailure, working,
                         result);
  }

  Eigen::Vector3d predicted_direction_1;
  Eigen::Vector3d predicted_direction_2;
  if (!normalizeFiniteVector(working.point_common - origin_1,
                             &predicted_direction_1) ||
      !normalizeFiniteVector(working.point_common - origin_2,
                             &predicted_direction_2) ||
      !angularDistance(predicted_direction_1, normalized_direction_1,
                       &working.angular_reprojection_error_1) ||
      !angularDistance(predicted_direction_2, normalized_direction_2,
                       &working.angular_reprojection_error_2)) {
    return finishFailure(TriangulationStatus::kNumericalFailure, working,
                         result);
  }
  working.maximum_angular_reprojection_error =
      std::max(working.angular_reprojection_error_1,
               working.angular_reprojection_error_2);

  if (working.closest_ray_distance >
      options.maximum_closest_ray_distance) {
    return finishFailure(TriangulationStatus::kClosestDistanceTooLarge,
                         working, result);
  }
  if (working.maximum_angular_reprojection_error >
      options.maximum_angular_reprojection_error) {
    return finishFailure(TriangulationStatus::kReprojectionErrorTooLarge,
                         working, result);
  }

  working.valid = true;
  working.status = TriangulationStatus::kSuccess;
  *result = working;
  return true;
}

bool triangulateBearings(const Eigen::Vector3d& bearing_1,
                         const Eigen::Vector3d& bearing_2,
                         const Eigen::Matrix3d& R_2_1,
                         const Eigen::Vector3d& t_2_1,
                         const TriangulationOptions& options,
                         TriangulationResult* result) {
  if (!result) return false;
  if (!isProperRotation(R_2_1) || !t_2_1.allFinite()) {
    TriangulationResult invalid;
    invalid.coordinate_frame = TriangulationCoordinateFrame::kTargetCamera;
    return finishFailure(TriangulationStatus::kInvalidInput, invalid, result);
  }
  const bool success = triangulateRays(
      t_2_1, R_2_1 * bearing_1, Eigen::Vector3d::Zero(), bearing_2,
      options, result);
  result->coordinate_frame = TriangulationCoordinateFrame::kTargetCamera;
  return success;
}

bool triangulateBodyBearings(const RigCamera& camera_1,
                             const Eigen::Vector3d& bearing_c1,
                             const RigCamera& camera_2,
                             const Eigen::Vector3d& bearing_c2,
                             const TriangulationOptions& options,
                             TriangulationResult* result) {
  if (!result) return false;
  if (!isProperRotation(camera_1.R_b_c) ||
      !isProperRotation(camera_2.R_b_c) || !camera_1.t_b_c.allFinite() ||
      !camera_2.t_b_c.allFinite()) {
    TriangulationResult invalid;
    invalid.coordinate_frame = TriangulationCoordinateFrame::kBody;
    return finishFailure(TriangulationStatus::kInvalidInput, invalid, result);
  }
  const bool success = triangulateRays(
      camera_1.t_b_c, camera_1.R_b_c * bearing_c1, camera_2.t_b_c,
      camera_2.R_b_c * bearing_c2, options, result);
  result->coordinate_frame = TriangulationCoordinateFrame::kBody;
  return success;
}

}  // namespace sphere_vio
