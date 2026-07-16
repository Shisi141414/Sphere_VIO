#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "sphere_vio/camera/camera_rig.hpp"

namespace sphere_vio {

// Transform convention: p_target = R_target_source * p_source +
// t_target_source. Translation is expressed in the target frame.
struct RelativePose {
  Eigen::Matrix3d R_target_source = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_target_source = Eigen::Vector3d::Zero();
};

struct EpipolarError {
  bool valid = false;
  double forward = 0.0;
  double backward = 0.0;
  double maximum = 0.0;
  double average = 0.0;
};

bool relativeCameraPose(const RigCamera& source, const RigCamera& target,
                        RelativePose* relative_pose);

bool epipolarAlgebraicResidual(
    const Eigen::Vector3d& bearing_source,
    const Eigen::Vector3d& bearing_target,
    const Eigen::Matrix3d& R_target_source,
    const Eigen::Vector3d& t_target_source, double* residual);

bool epipolarPlaneNormal(const Eigen::Vector3d& bearing_source,
                         const Eigen::Matrix3d& R_target_source,
                         const Eigen::Vector3d& t_target_source,
                         Eigen::Vector3d* plane_normal_target);

// Returns the non-negative angle, in radians, between bearing_target and the
// target-frame epipolar plane.
bool epipolarAngularError(const Eigen::Vector3d& bearing_source,
                          const Eigen::Vector3d& bearing_target,
                          const Eigen::Matrix3d& R_target_source,
                          const Eigen::Vector3d& t_target_source,
                          double* angular_error);

bool symmetricEpipolarAngularError(
    const Eigen::Vector3d& bearing_source,
    const Eigen::Vector3d& bearing_target,
    const Eigen::Matrix3d& R_target_source,
    const Eigen::Vector3d& t_target_source, EpipolarError* error);

// Samples a unit-sphere great circle. The first sample is not repeated at the
// end; at least three samples are required.
bool sampleGreatCircle(const Eigen::Vector3d& plane_normal,
                       std::size_t sample_count,
                       std::vector<Eigen::Vector3d>* bearings);

}  // namespace sphere_vio
