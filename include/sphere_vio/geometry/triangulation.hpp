#pragma once

#include <limits>

#include <Eigen/Core>

#include "sphere_vio/camera/camera_rig.hpp"

namespace sphere_vio {

enum class TriangulationStatus {
  kSuccess,
  kInvalidInput,
  kZeroBaseline,
  kNearlyParallelRays,
  kNegativeDepth,
  kClosestDistanceTooLarge,
  kReprojectionErrorTooLarge,
  kNumericalFailure
};

enum class TriangulationCoordinateFrame {
  kCallerCommon,
  kTargetCamera,
  kBody
};

struct TriangulationOptions {
  double minimum_baseline = 1e-9;
  double minimum_ray_angle = 1e-3;
  double minimum_depth = 1e-6;
  double maximum_closest_ray_distance =
      std::numeric_limits<double>::infinity();
  double maximum_angular_reprojection_error =
      std::numeric_limits<double>::infinity();
};

struct TriangulationResult {
  bool valid = false;
  TriangulationStatus status = TriangulationStatus::kInvalidInput;
  TriangulationCoordinateFrame coordinate_frame =
      TriangulationCoordinateFrame::kCallerCommon;

  Eigen::Vector3d point_common = Eigen::Vector3d::Zero();
  Eigen::Vector3d closest_point_1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d closest_point_2 = Eigen::Vector3d::Zero();

  double depth_1 = 0.0;
  double depth_2 = 0.0;
  double baseline = 0.0;
  double ray_angle = 0.0;
  double closest_ray_distance = 0.0;
  double angular_reprojection_error_1 = 0.0;
  double angular_reprojection_error_2 = 0.0;
  double maximum_angular_reprojection_error = 0.0;
};

const char* triangulationStatusName(TriangulationStatus status);

// Triangulates two rays expressed in one caller-defined common frame. On
// success, point_common and both closest points are in that common frame.
bool triangulateRays(const Eigen::Vector3d& origin_1,
                     const Eigen::Vector3d& direction_1,
                     const Eigen::Vector3d& origin_2,
                     const Eigen::Vector3d& direction_2,
                     const TriangulationOptions& options,
                     TriangulationResult* result);

// Transform convention: p_2 = R_2_1 * p_1 + t_2_1. The result coordinates
// are in target camera C2, and the returned depths are ray parameters in C1
// and C2 respectively.
bool triangulateBearings(const Eigen::Vector3d& bearing_1,
                         const Eigen::Vector3d& bearing_2,
                         const Eigen::Matrix3d& R_2_1,
                         const Eigen::Vector3d& t_2_1,
                         const TriangulationOptions& options,
                         TriangulationResult* result);

// Bearings are supplied in their camera frames. Camera centers and rotated
// rays are triangulated in Body, so point_common is point_b on success.
bool triangulateBodyBearings(const RigCamera& camera_1,
                             const Eigen::Vector3d& bearing_c1,
                             const RigCamera& camera_2,
                             const Eigen::Vector3d& bearing_c2,
                             const TriangulationOptions& options,
                             TriangulationResult* result);

}  // namespace sphere_vio
