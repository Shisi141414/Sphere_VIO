#include "sphere_vio/geometry/spherical_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace sphere_vio {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kHalfPi = 0.5 * kPi;
constexpr double kMinimumBearingNorm = 1e-12;
constexpr double kPoleHorizontalNorm = 1e-12;

bool normalizeBearing(const Eigen::Vector3d& bearing,
                      Eigen::Vector3d* normalized_bearing) {
  if (!normalized_bearing || !bearing.allFinite()) return false;
  const double norm = bearing.norm();
  if (!std::isfinite(norm) || norm <= kMinimumBearingNorm) return false;
  const Eigen::Vector3d normalized = bearing / norm;
  if (!normalized.allFinite()) return false;
  *normalized_bearing = normalized;
  return true;
}

bool dimensionsAreValid(int width, int height) {
  return width > 0 && height > 0;
}

}  // namespace

bool wrapLongitude(double longitude, double* wrapped_longitude) {
  if (!wrapped_longitude || !std::isfinite(longitude)) return false;
  double wrapped = std::fmod(longitude + kPi, kTwoPi);
  if (!std::isfinite(wrapped)) return false;
  if (wrapped < 0.0) wrapped += kTwoPi;
  wrapped -= kPi;
  if (wrapped >= kPi) wrapped = -kPi;
  if (wrapped < -kPi) wrapped += kTwoPi;
  *wrapped_longitude = wrapped;
  return true;
}

bool wrappedLongitudeDifference(double longitude_a, double longitude_b,
                                double* difference) {
  if (!difference || !std::isfinite(longitude_a) ||
      !std::isfinite(longitude_b)) {
    return false;
  }
  return wrapLongitude(longitude_a - longitude_b, difference);
}

bool bearingToLongitudeLatitude(
    const Eigen::Vector3d& bearing_b,
    Eigen::Vector2d* longitude_latitude) {
  if (!longitude_latitude) return false;
  Eigen::Vector3d normalized_bearing;
  if (!normalizeBearing(bearing_b, &normalized_bearing)) return false;

  const double horizontal_norm =
      std::hypot(normalized_bearing.x(), normalized_bearing.y());
  double longitude = 0.0;
  if (horizontal_norm > kPoleHorizontalNorm) {
    if (!wrapLongitude(
            std::atan2(normalized_bearing.y(), normalized_bearing.x()),
            &longitude)) {
      return false;
    }
  }
  const double latitude =
      std::atan2(normalized_bearing.z(), horizontal_norm);
  const Eigen::Vector2d result(longitude, latitude);
  if (!result.allFinite()) return false;
  *longitude_latitude = result;
  return true;
}

bool longitudeLatitudeToBearing(
    const Eigen::Vector2d& longitude_latitude,
    Eigen::Vector3d* bearing_b) {
  if (!bearing_b || !longitude_latitude.allFinite()) return false;
  const double latitude = longitude_latitude.y();
  if (latitude < -kHalfPi || latitude > kHalfPi) return false;

  double longitude = 0.0;
  if (!wrapLongitude(longitude_latitude.x(), &longitude)) return false;
  const double cos_latitude = std::cos(latitude);
  const Eigen::Vector3d bearing(cos_latitude * std::cos(longitude),
                                cos_latitude * std::sin(longitude),
                                std::sin(latitude));
  return normalizeBearing(bearing, bearing_b);
}

bool bearingToEquirectangular(const Eigen::Vector3d& bearing_b, int width,
                              int height, Eigen::Vector2d* erp_coordinate) {
  if (!erp_coordinate || !dimensionsAreValid(width, height)) return false;
  Eigen::Vector2d longitude_latitude;
  if (!bearingToLongitudeLatitude(bearing_b, &longitude_latitude)) {
    return false;
  }

  double u = static_cast<double>(width) *
             (longitude_latitude.x() + kPi) / kTwoPi;
  double v = static_cast<double>(height) *
             (kHalfPi - longitude_latitude.y()) / kPi;
  if (!std::isfinite(u) || !std::isfinite(v)) return false;
  if (u >= static_cast<double>(width)) u = 0.0;
  u = std::max(0.0, u);
  v = std::max(0.0, std::min(static_cast<double>(height), v));
  *erp_coordinate = Eigen::Vector2d(u, v);
  return true;
}

bool equirectangularToBearing(const Eigen::Vector2d& erp_coordinate,
                              int width, int height,
                              Eigen::Vector3d* bearing_b) {
  if (!bearing_b || !dimensionsAreValid(width, height) ||
      !erp_coordinate.allFinite()) {
    return false;
  }
  const double height_as_double = static_cast<double>(height);
  if (erp_coordinate.y() < 0.0 || erp_coordinate.y() > height_as_double) {
    return false;
  }

  const double width_as_double = static_cast<double>(width);
  double wrapped_u = std::fmod(erp_coordinate.x(), width_as_double);
  if (!std::isfinite(wrapped_u)) return false;
  if (wrapped_u < 0.0) wrapped_u += width_as_double;
  if (wrapped_u >= width_as_double) wrapped_u = 0.0;

  const double longitude = kTwoPi * wrapped_u / width_as_double - kPi;
  const double latitude =
      kHalfPi - kPi * erp_coordinate.y() / height_as_double;
  return longitudeLatitudeToBearing(
      Eigen::Vector2d(longitude, latitude), bearing_b);
}

bool angularDistance(const Eigen::Vector3d& bearing_a,
                     const Eigen::Vector3d& bearing_b, double* angle) {
  if (!angle) return false;
  Eigen::Vector3d normalized_a;
  Eigen::Vector3d normalized_b;
  if (!normalizeBearing(bearing_a, &normalized_a) ||
      !normalizeBearing(bearing_b, &normalized_b)) {
    return false;
  }
  const double dot =
      std::max(-1.0, std::min(1.0, normalized_a.dot(normalized_b)));
  const double result = std::acos(dot);
  if (!std::isfinite(result)) return false;
  *angle = result;
  return true;
}

}  // namespace sphere_vio
