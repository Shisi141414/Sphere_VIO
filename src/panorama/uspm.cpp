#include "sphere_vio/panorama/uspm.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/LU>

namespace sphere_vio {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kMinimumNorm = 1e-12;
constexpr double kIntersectionEpsilon = 1e-12;
constexpr double kRotationTolerance = 1e-9;
constexpr double kDomainTolerance = 1e-12;

bool properRotation(const Eigen::Matrix3d& rotation) {
  return rotation.allFinite() &&
         (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <=
             kRotationTolerance &&
         std::abs(rotation.determinant() - 1.0) <= kRotationTolerance;
}

bool normalize(const Eigen::Vector3d& vector, Eigen::Vector3d* unit) {
  if (!unit || !vector.allFinite()) return false;
  const double norm = vector.norm();
  if (!std::isfinite(norm) || norm <= kMinimumNorm) return false;
  *unit = vector / norm;
  return unit->allFinite();
}

bool cameraPose(const RigCamera& camera, const PanoramaSpec& panorama,
                Eigen::Matrix3d* R_p_c, Eigen::Vector3d* t_p_c) {
  if (!R_p_c || !t_p_c || !camera.model ||
      !properRotation(camera.R_b_c) || !camera.t_b_c.allFinite() ||
      !validatePanoramaSpec(panorama)) return false;
  *R_p_c = panorama.frame.R_p_b * camera.R_b_c;
  *t_p_c = panorama.frame.R_p_b * camera.t_b_c + panorama.frame.t_p_b;
  return R_p_c->allFinite() && t_p_c->allFinite();
}

bool wrapFullPanoramaU(double width, double* u) {
  if (!u || !std::isfinite(*u)) return false;
  *u = std::fmod(*u, width);
  if (*u < 0.0) *u += width;
  if (*u >= width) *u = 0.0;
  return true;
}

}  // namespace

bool validatePanoramaSpec(const PanoramaSpec& panorama) {
  return panorama.width > 0 && panorama.height > 0 &&
         std::isfinite(panorama.horizontal_fov) &&
         panorama.horizontal_fov > 0.0 && panorama.horizontal_fov <= kTwoPi &&
         std::isfinite(panorama.vertical_fov) &&
         panorama.vertical_fov > 0.0 && panorama.vertical_fov <= kPi &&
         std::isfinite(panorama.sphere_radius) && panorama.sphere_radius > 0.0 &&
         properRotation(panorama.frame.R_p_b) &&
         panorama.frame.t_p_b.allFinite();
}

bool validatePanoramaForCamera(const RigCamera& camera,
                               const PanoramaSpec& panorama) {
  Eigen::Matrix3d R_p_c;
  Eigen::Vector3d t_p_c;
  return cameraPose(camera, panorama, &R_p_c, &t_p_c) &&
         t_p_c.norm() < panorama.sphere_radius;
}

bool validatePanoramaForRig(const CameraRig& rig,
                            const PanoramaSpec& panorama) {
  if (!validatePanoramaSpec(panorama) || rig.size() == 0U) return false;
  for (CameraId id = 0; id <= 3U; ++id) {
    const RigCamera* camera = rig.camera(id);
    if (camera && !validatePanoramaForCamera(*camera, panorama)) return false;
  }
  return true;
}

bool sphereBearingToPanorama(const Eigen::Vector3d& sphere_bearing_p,
                             const PanoramaSpec& panorama,
                             Eigen::Vector2d* panorama_coordinate) {
  if (!panorama_coordinate || !validatePanoramaSpec(panorama)) return false;
  Eigen::Vector3d n;
  if (!normalize(sphere_bearing_p, &n)) return false;
  double theta = std::atan2(n.x(), n.z());
  const double phi = std::asin(std::max(-1.0, std::min(1.0, n.y())));
  const double half_h = 0.5 * panorama.horizontal_fov;
  const double half_v = 0.5 * panorama.vertical_fov;
  if (panorama.horizontal_fov == kTwoPi && theta >= half_h) theta = -half_h;
  if (theta < -half_h || theta >= half_h || phi < -half_v - kDomainTolerance ||
      phi > half_v + kDomainTolerance) return false;
  Eigen::Vector2d coordinate(
      panorama.width * (theta / panorama.horizontal_fov + 0.5),
      panorama.height * (phi / panorama.vertical_fov + 0.5));
  if (!coordinate.allFinite()) return false;
  if (panorama.horizontal_fov == kTwoPi &&
      !wrapFullPanoramaU(static_cast<double>(panorama.width), &coordinate.x())) {
    return false;
  }
  coordinate.y() = std::max(0.0, std::min(static_cast<double>(panorama.height),
                                          coordinate.y()));
  *panorama_coordinate = coordinate;
  return true;
}

bool panoramaToSphereBearing(const Eigen::Vector2d& coordinate,
                             const PanoramaSpec& panorama,
                             Eigen::Vector3d* sphere_bearing_p) {
  if (!sphere_bearing_p || !coordinate.allFinite() ||
      !validatePanoramaSpec(panorama)) return false;
  double u = coordinate.x();
  if (panorama.horizontal_fov == kTwoPi) {
    if (!wrapFullPanoramaU(static_cast<double>(panorama.width), &u)) return false;
  } else if (u < 0.0 || u >= static_cast<double>(panorama.width)) {
    return false;
  }
  if (coordinate.y() < 0.0 ||
      coordinate.y() > static_cast<double>(panorama.height)) return false;
  const double theta = (u / panorama.width - 0.5) * panorama.horizontal_fov;
  const double phi = (coordinate.y() / panorama.height - 0.5) *
                     panorama.vertical_fov;
  const double cos_phi = std::cos(phi);
  // This sin/cos ordering is deliberately paired with atan2(x,z) above.
  return normalize(Eigen::Vector3d(cos_phi * std::sin(theta), std::sin(phi),
                                   cos_phi * std::cos(theta)),
                   sphere_bearing_p);
}

bool cameraBearingToSphere(const RigCamera& camera,
                           const Eigen::Vector3d& bearing_c,
                           const PanoramaSpec& panorama,
                           Eigen::Vector3d* sphere_point_p,
                           Eigen::Vector3d* sphere_bearing_p) {
  if (!sphere_point_p || !sphere_bearing_p ||
      !validatePanoramaForCamera(camera, panorama)) return false;
  Eigen::Vector3d unit_c;
  Eigen::Matrix3d R_p_c;
  Eigen::Vector3d origin;
  if (!normalize(bearing_c, &unit_c) ||
      !cameraPose(camera, panorama, &R_p_c, &origin)) return false;
  Eigen::Vector3d direction;
  if (!normalize(R_p_c * unit_c, &direction)) return false;
  const double a = origin.dot(direction);
  double discriminant = a * a - origin.squaredNorm() +
                        panorama.sphere_radius * panorama.sphere_radius;
  if (!std::isfinite(discriminant) || discriminant < -kDomainTolerance) return false;
  discriminant = std::max(0.0, discriminant);
  const double distance = -a + std::sqrt(discriminant);
  if (!std::isfinite(distance) || distance <= kIntersectionEpsilon) return false;
  const Eigen::Vector3d point = origin + distance * direction;
  Eigen::Vector3d unit_point;
  if (!point.allFinite() || !normalize(point, &unit_point)) return false;
  *sphere_point_p = panorama.sphere_radius * unit_point;
  *sphere_bearing_p = unit_point;
  return true;
}

bool cameraBearingToPanorama(const RigCamera& camera,
                             const Eigen::Vector3d& bearing_c,
                             const PanoramaSpec& panorama,
                             Eigen::Vector2d* panorama_coordinate,
                             Eigen::Vector3d* sphere_bearing_p) {
  if (!panorama_coordinate) return false;
  Eigen::Vector3d point;
  Eigen::Vector3d bearing;
  if (!cameraBearingToSphere(camera, bearing_c, panorama, &point, &bearing) ||
      !sphereBearingToPanorama(bearing, panorama, panorama_coordinate)) return false;
  if (sphere_bearing_p) *sphere_bearing_p = bearing;
  return true;
}

bool cameraPixelToPanorama(const RigCamera& camera, const Eigen::Vector2d& pixel,
                           const PanoramaSpec& panorama,
                           Eigen::Vector2d* panorama_coordinate,
                           Eigen::Vector3d* sphere_bearing_p) {
  if (!panorama_coordinate || !camera.model) return false;
  Eigen::Vector3d bearing_c;
  return camera.model->unproject(pixel, &bearing_c) &&
         cameraBearingToPanorama(camera, bearing_c, panorama,
                                 panorama_coordinate, sphere_bearing_p);
}

bool panoramaToCameraBearing(const RigCamera& camera,
                             const Eigen::Vector2d& panorama_coordinate,
                             const PanoramaSpec& panorama,
                             Eigen::Vector3d* bearing_c) {
  if (!bearing_c || !validatePanoramaForCamera(camera, panorama)) return false;
  Eigen::Vector3d n_p;
  Eigen::Matrix3d R_p_c;
  Eigen::Vector3d t_p_c;
  if (!panoramaToSphereBearing(panorama_coordinate, panorama, &n_p) ||
      !cameraPose(camera, panorama, &R_p_c, &t_p_c)) return false;
  return normalize(R_p_c.transpose() *
                       (panorama.sphere_radius * n_p - t_p_c),
                   bearing_c);
}

bool panoramaToCameraPixel(const RigCamera& camera,
                           const Eigen::Vector2d& panorama_coordinate,
                           const PanoramaSpec& panorama,
                           Eigen::Vector2d* pixel) {
  if (!pixel || !camera.model) return false;
  Eigen::Vector3d bearing_c;
  return panoramaToCameraBearing(camera, panorama_coordinate, panorama,
                                 &bearing_c) &&
         camera.model->project(bearing_c, pixel);
}

}  // namespace sphere_vio
