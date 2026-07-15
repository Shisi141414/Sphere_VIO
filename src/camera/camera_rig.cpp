#include "sphere_vio/camera/camera_rig.hpp"

#include <cmath>

#include <Eigen/LU>

namespace sphere_vio {
namespace {

constexpr CameraId kMaximumCameraId = 3U;
constexpr double kMinimumBearingNorm = 1e-12;
constexpr double kRotationOrthogonalityTolerance = 1e-9;
constexpr double kRotationDeterminantTolerance = 1e-9;

bool normalizeFiniteVector(const Eigen::Vector3d& input,
                           Eigen::Vector3d* output) {
  if (!output || !input.allFinite()) return false;
  const double norm = input.norm();
  if (!std::isfinite(norm) || norm <= kMinimumBearingNorm) return false;
  const Eigen::Vector3d normalized = input / norm;
  if (!normalized.allFinite()) return false;
  *output = normalized;
  return true;
}

bool isProperRotation(const Eigen::Matrix3d& R_b_c) {
  if (!R_b_c.allFinite()) return false;
  const double orthogonality_error =
      (R_b_c.transpose() * R_b_c - Eigen::Matrix3d::Identity()).norm();
  const double determinant = R_b_c.determinant();
  return std::isfinite(orthogonality_error) &&
         orthogonality_error <= kRotationOrthogonalityTolerance &&
         std::isfinite(determinant) && determinant > 0.0 &&
         std::abs(determinant - 1.0) <= kRotationDeterminantTolerance;
}

}  // namespace

bool CameraRig::addCamera(const RigCamera& camera_to_add) {
  if (camera_to_add.camera_id > kMaximumCameraId ||
      camera_to_add.name.empty() || !camera_to_add.model ||
      !isProperRotation(camera_to_add.R_b_c) ||
      !camera_to_add.t_b_c.allFinite() ||
      (camera_to_add.timeshift_available &&
       !std::isfinite(camera_to_add.timeshift_cam_imu)) ||
      (!camera_to_add.timeshift_available &&
       std::isfinite(camera_to_add.timeshift_cam_imu)) ||
      hasCamera(camera_to_add.camera_id)) {
    return false;
  }
  cameras_.emplace(camera_to_add.camera_id, camera_to_add);
  return true;
}

std::size_t CameraRig::size() const { return cameras_.size(); }

bool CameraRig::hasCamera(CameraId camera_id) const {
  return cameras_.find(camera_id) != cameras_.end();
}

const RigCamera* CameraRig::camera(CameraId camera_id) const {
  const auto iterator = cameras_.find(camera_id);
  return iterator == cameras_.end() ? nullptr : &iterator->second;
}

bool CameraRig::pixelToCameraBearing(CameraId camera_id,
                                     const Eigen::Vector2d& pixel,
                                     Eigen::Vector3d* bearing_c) const {
  const RigCamera* rig_camera = camera(camera_id);
  return bearing_c && rig_camera &&
         rig_camera->model->unproject(pixel, bearing_c);
}

bool CameraRig::cameraBearingToBody(CameraId camera_id,
                                    const Eigen::Vector3d& bearing_c,
                                    Eigen::Vector3d* bearing_b) const {
  const RigCamera* rig_camera = camera(camera_id);
  if (!bearing_b || !rig_camera || !bearing_c.allFinite()) return false;
  return normalizeFiniteVector(rig_camera->R_b_c * bearing_c, bearing_b);
}

bool CameraRig::bodyBearingToCamera(CameraId camera_id,
                                    const Eigen::Vector3d& bearing_b,
                                    Eigen::Vector3d* bearing_c) const {
  const RigCamera* rig_camera = camera(camera_id);
  if (!bearing_c || !rig_camera || !bearing_b.allFinite()) return false;
  return normalizeFiniteVector(rig_camera->R_b_c.transpose() * bearing_b,
                               bearing_c);
}

bool CameraRig::pixelToBodyBearing(CameraId camera_id,
                                   const Eigen::Vector2d& pixel,
                                   Eigen::Vector3d* bearing_b) const {
  if (!bearing_b) return false;
  Eigen::Vector3d bearing_c;
  return pixelToCameraBearing(camera_id, pixel, &bearing_c) &&
         cameraBearingToBody(camera_id, bearing_c, bearing_b);
}

bool CameraRig::cameraPointToBody(CameraId camera_id,
                                  const Eigen::Vector3d& point_c,
                                  Eigen::Vector3d* point_b) const {
  const RigCamera* rig_camera = camera(camera_id);
  if (!point_b || !rig_camera || !point_c.allFinite()) return false;
  const Eigen::Vector3d transformed =
      rig_camera->R_b_c * point_c + rig_camera->t_b_c;
  if (!transformed.allFinite()) return false;
  *point_b = transformed;
  return true;
}

bool CameraRig::bodyPointToCamera(CameraId camera_id,
                                  const Eigen::Vector3d& point_b,
                                  Eigen::Vector3d* point_c) const {
  const RigCamera* rig_camera = camera(camera_id);
  if (!point_c || !rig_camera || !point_b.allFinite()) return false;
  const Eigen::Matrix3d R_c_b = rig_camera->R_b_c.transpose();
  const Eigen::Vector3d t_c_b = -R_c_b * rig_camera->t_b_c;
  const Eigen::Vector3d transformed = R_c_b * point_b + t_c_b;
  if (!transformed.allFinite()) return false;
  *point_c = transformed;
  return true;
}

bool CameraRig::projectBodyPointToCamera(CameraId camera_id,
                                         const Eigen::Vector3d& point_b,
                                         Eigen::Vector2d* pixel) const {
  const RigCamera* rig_camera = camera(camera_id);
  if (!pixel || !rig_camera) return false;
  Eigen::Vector3d point_c;
  return bodyPointToCamera(camera_id, point_b, &point_c) &&
         rig_camera->model->project(point_c, pixel);
}

}  // namespace sphere_vio
