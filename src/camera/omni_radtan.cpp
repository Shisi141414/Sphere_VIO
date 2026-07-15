#include "sphere_vio/camera/omni_radtan.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sphere_vio {
namespace {

constexpr double kDenominatorEpsilon = 1e-12;
constexpr double kVisibilityEpsilon = 1e-12;
constexpr double kMinimumVectorNorm = 1e-12;
constexpr double kUndistortionTolerance = 1e-13;
constexpr double kMinimumJacobianDeterminant = 1e-14;
constexpr double kInverseSquareRootTolerance = 1e-12;
constexpr double kPixelBoundaryTolerance = 1e-9;
constexpr int kMaximumUndistortionIterations = 50;

bool parametersAreFinite(const OmniRadtan::Parameters& parameters) {
  return std::isfinite(parameters.xi) && std::isfinite(parameters.fx) &&
         std::isfinite(parameters.fy) && std::isfinite(parameters.cx) &&
         std::isfinite(parameters.cy) && std::isfinite(parameters.k1) &&
         std::isfinite(parameters.k2) && std::isfinite(parameters.p1) &&
         std::isfinite(parameters.p2);
}

double clampNearImageBoundary(double coordinate, double upper_bound) {
  if (coordinate < 0.0 && coordinate >= -kPixelBoundaryTolerance) {
    return 0.0;
  }
  if (coordinate >= upper_bound &&
      coordinate < upper_bound + kPixelBoundaryTolerance) {
    return std::nextafter(upper_bound, 0.0);
  }
  return coordinate;
}

}  // namespace

OmniRadtan::OmniRadtan(const Parameters& parameters)
    : parameters_(parameters) {
  if (parameters_.width <= 0 || parameters_.height <= 0 ||
      parameters_.fx <= 0.0 || parameters_.fy <= 0.0 ||
      parameters_.xi < 0.0 || !parametersAreFinite(parameters_)) {
    throw std::invalid_argument("Invalid omni-radtan camera parameters");
  }
}

bool OmniRadtan::project(const Eigen::Vector3d& point_c,
                         Eigen::Vector2d* pixel) const {
  if (!pixel || !point_c.allFinite()) return false;

  const double point_norm = point_c.norm();
  if (!std::isfinite(point_norm) || point_norm <= kMinimumVectorNorm) {
    return false;
  }

  const Eigen::Vector3d bearing_c = point_c / point_norm;
  if (!isVisible(bearing_c)) return false;

  const double denominator =
      point_c.z() + parameters_.xi * point_norm;
  if (!std::isfinite(denominator) ||
      denominator <= kDenominatorEpsilon * point_norm) {
    return false;
  }

  const Eigen::Vector2d normalized(point_c.x() / denominator,
                                   point_c.y() / denominator);
  Eigen::Vector2d distorted;
  if (!distort(normalized, &distorted)) return false;

  Eigen::Vector2d projected_pixel(
      parameters_.fx * distorted.x() + parameters_.cx,
      parameters_.fy * distorted.y() + parameters_.cy);
  projected_pixel.x() = clampNearImageBoundary(
      projected_pixel.x(), static_cast<double>(parameters_.width));
  projected_pixel.y() = clampNearImageBoundary(
      projected_pixel.y(), static_cast<double>(parameters_.height));
  if (!isPixelValid(projected_pixel)) return false;

  *pixel = projected_pixel;
  return true;
}

bool OmniRadtan::unproject(const Eigen::Vector2d& pixel,
                           Eigen::Vector3d* bearing_c) const {
  if (!bearing_c || !isPixelValid(pixel)) return false;

  const Eigen::Vector2d distorted(
      (pixel.x() - parameters_.cx) / parameters_.fx,
      (pixel.y() - parameters_.cy) / parameters_.fy);
  Eigen::Vector2d normalized;
  if (!undistort(distorted, &normalized)) return false;

  const double radius_squared = normalized.squaredNorm();
  if (!std::isfinite(radius_squared)) return false;

  double square_root_argument =
      1.0 + (1.0 - parameters_.xi * parameters_.xi) * radius_squared;
  if (!std::isfinite(square_root_argument) ||
      square_root_argument < -kInverseSquareRootTolerance) {
    return false;
  }
  square_root_argument = std::max(0.0, square_root_argument);

  const double denominator = 1.0 + radius_squared;
  if (!std::isfinite(denominator) || denominator <= kDenominatorEpsilon) {
    return false;
  }
  const double scale =
      (parameters_.xi + std::sqrt(square_root_argument)) / denominator;
  Eigen::Vector3d bearing(scale * normalized.x(), scale * normalized.y(),
                          scale - parameters_.xi);
  const double bearing_norm = bearing.norm();
  if (!bearing.allFinite() || !std::isfinite(bearing_norm) ||
      bearing_norm <= kMinimumVectorNorm) {
    return false;
  }
  bearing /= bearing_norm;
  if (!bearing.allFinite() || !isVisible(bearing)) return false;

  *bearing_c = bearing;
  return true;
}

bool OmniRadtan::isPixelValid(const Eigen::Vector2d& pixel) const {
  return pixel.allFinite() && pixel.x() >= 0.0 && pixel.y() >= 0.0 &&
         pixel.x() < static_cast<double>(parameters_.width) &&
         pixel.y() < static_cast<double>(parameters_.height);
}

int OmniRadtan::width() const { return parameters_.width; }

int OmniRadtan::height() const { return parameters_.height; }

std::string OmniRadtan::modelName() const { return "omni_radtan"; }

bool OmniRadtan::distort(const Eigen::Vector2d& normalized,
                         Eigen::Vector2d* distorted) const {
  if (!distorted || !normalized.allFinite()) return false;

  const double x = normalized.x();
  const double y = normalized.y();
  const double radius_squared = normalized.squaredNorm();
  const double radial =
      1.0 + parameters_.k1 * radius_squared +
      parameters_.k2 * radius_squared * radius_squared;
  const Eigen::Vector2d result(
      x * radial + 2.0 * parameters_.p1 * x * y +
          parameters_.p2 * (radius_squared + 2.0 * x * x),
      y * radial + parameters_.p1 * (radius_squared + 2.0 * y * y) +
          2.0 * parameters_.p2 * x * y);
  if (!std::isfinite(radial) || !result.allFinite()) return false;

  *distorted = result;
  return true;
}

bool OmniRadtan::undistort(const Eigen::Vector2d& distorted,
                           Eigen::Vector2d* normalized) const {
  if (!normalized || !distorted.allFinite()) return false;

  Eigen::Vector2d estimate = distorted;
  for (int iteration = 0; iteration < kMaximumUndistortionIterations;
       ++iteration) {
    Eigen::Vector2d predicted;
    if (!distort(estimate, &predicted)) return false;
    const Eigen::Vector2d residual = predicted - distorted;
    if (!residual.allFinite()) return false;
    if (residual.norm() <= kUndistortionTolerance) {
      *normalized = estimate;
      return true;
    }

    const double x = estimate.x();
    const double y = estimate.y();
    const double radius_squared = estimate.squaredNorm();
    const double radial =
        1.0 + parameters_.k1 * radius_squared +
        parameters_.k2 * radius_squared * radius_squared;
    const double radial_x =
        2.0 * parameters_.k1 * x +
        4.0 * parameters_.k2 * radius_squared * x;
    const double radial_y =
        2.0 * parameters_.k1 * y +
        4.0 * parameters_.k2 * radius_squared * y;

    const double j00 = radial + x * radial_x +
                       2.0 * parameters_.p1 * y +
                       6.0 * parameters_.p2 * x;
    const double j01 = x * radial_y + 2.0 * parameters_.p1 * x +
                       2.0 * parameters_.p2 * y;
    const double j10 = y * radial_x + 2.0 * parameters_.p1 * x +
                       2.0 * parameters_.p2 * y;
    const double j11 = radial + y * radial_y +
                       6.0 * parameters_.p1 * y +
                       2.0 * parameters_.p2 * x;
    const double determinant = j00 * j11 - j01 * j10;
    if (!std::isfinite(determinant) ||
        std::abs(determinant) <= kMinimumJacobianDeterminant) {
      return false;
    }

    const Eigen::Vector2d step(
        (j11 * residual.x() - j01 * residual.y()) / determinant,
        (-j10 * residual.x() + j00 * residual.y()) / determinant);
    if (!step.allFinite()) return false;
    estimate -= step;
    if (!estimate.allFinite()) return false;
  }
  return false;
}

bool OmniRadtan::isVisible(const Eigen::Vector3d& bearing_c) const {
  if (!bearing_c.allFinite()) return false;
  const double norm = bearing_c.norm();
  if (!std::isfinite(norm) || norm <= kMinimumVectorNorm) return false;

  const double normalized_z = bearing_c.z() / norm;
  const double visibility_parameter =
      parameters_.xi <= 1.0 ? parameters_.xi : 1.0 / parameters_.xi;
  return std::isfinite(normalized_z) &&
         normalized_z > -visibility_parameter + kVisibilityEpsilon;
}

}  // namespace sphere_vio
