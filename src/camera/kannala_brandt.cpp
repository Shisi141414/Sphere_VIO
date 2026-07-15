#include "sphere_vio/camera/kannala_brandt.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sphere_vio {
namespace {

constexpr double kOpticalAxisEpsilon = 1e-12;
constexpr double kMinimumPointNorm = 1e-12;
constexpr double kMinimumDerivative = 1e-12;
constexpr double kNewtonTolerance = 1e-12;
constexpr int kMaximumNewtonIterations = 50;
constexpr double kMaximumTheta = 3.14159265358979323846 - 1e-8;

bool parametersAreFinite(const KannalaBrandt::Parameters& parameters) {
  return std::isfinite(parameters.fx) && std::isfinite(parameters.fy) &&
         std::isfinite(parameters.cx) && std::isfinite(parameters.cy) &&
         std::isfinite(parameters.k1) && std::isfinite(parameters.k2) &&
         std::isfinite(parameters.k3) && std::isfinite(parameters.k4);
}

}  // namespace

KannalaBrandt::KannalaBrandt(const Parameters& parameters)
    : parameters_(parameters) {
  if (parameters_.width <= 0 || parameters_.height <= 0 ||
      parameters_.fx <= 0.0 || parameters_.fy <= 0.0 ||
      !parametersAreFinite(parameters_)) {
    throw std::invalid_argument("Invalid Kannala-Brandt camera parameters");
  }
}

bool KannalaBrandt::project(const Eigen::Vector3d& point_c,
                            Eigen::Vector2d* pixel) const {
  if (!pixel || !point_c.allFinite() ||
      point_c.norm() <= kMinimumPointNorm) {
    return false;
  }

  const double radial_distance = std::hypot(point_c.x(), point_c.y());
  Eigen::Vector2d projected_pixel;
  if (radial_distance <= kOpticalAxisEpsilon) {
    if (point_c.z() <= 0.0) return false;
    projected_pixel = Eigen::Vector2d(parameters_.cx, parameters_.cy);
  } else {
    const double theta = std::atan2(radial_distance, point_c.z());
    if (!std::isfinite(theta) || theta < 0.0 || theta > kMaximumTheta) {
      return false;
    }
    const double theta_d = distortAngle(theta);
    if (!std::isfinite(theta_d) || theta_d < 0.0) return false;
    projected_pixel.x() =
        parameters_.fx * theta_d * point_c.x() / radial_distance +
        parameters_.cx;
    projected_pixel.y() =
        parameters_.fy * theta_d * point_c.y() / radial_distance +
        parameters_.cy;
  }

  if (!isPixelValid(projected_pixel)) return false;
  *pixel = projected_pixel;
  return true;
}

bool KannalaBrandt::unproject(const Eigen::Vector2d& pixel,
                              Eigen::Vector3d* bearing_c) const {
  if (!bearing_c || !isPixelValid(pixel)) return false;

  const double mx = (pixel.x() - parameters_.cx) / parameters_.fx;
  const double my = (pixel.y() - parameters_.cy) / parameters_.fy;
  const double theta_d = std::hypot(mx, my);
  if (!std::isfinite(theta_d)) return false;

  Eigen::Vector3d bearing;
  if (theta_d <= kOpticalAxisEpsilon) {
    bearing = Eigen::Vector3d::UnitZ();
  } else {
    double theta = 0.0;
    if (!undistortAngle(theta_d, &theta)) return false;
    const double radial_scale = std::sin(theta) / theta_d;
    bearing = Eigen::Vector3d(radial_scale * mx, radial_scale * my,
                              std::cos(theta));
    const double norm = bearing.norm();
    if (!bearing.allFinite() || !std::isfinite(norm) ||
        norm <= kMinimumPointNorm) {
      return false;
    }
    bearing /= norm;
  }

  if (!bearing.allFinite()) return false;
  *bearing_c = bearing;
  return true;
}

bool KannalaBrandt::isPixelValid(const Eigen::Vector2d& pixel) const {
  return pixel.allFinite() && pixel.x() >= 0.0 && pixel.y() >= 0.0 &&
         pixel.x() < static_cast<double>(parameters_.width) &&
         pixel.y() < static_cast<double>(parameters_.height);
}

int KannalaBrandt::width() const { return parameters_.width; }

int KannalaBrandt::height() const { return parameters_.height; }

std::string KannalaBrandt::modelName() const { return "KannalaBrandtKB4"; }

double KannalaBrandt::distortAngle(double theta) const {
  const double theta2 = theta * theta;
  const double theta3 = theta2 * theta;
  return theta + theta3 *
                     (parameters_.k1 +
                      theta2 * (parameters_.k2 +
                                theta2 * (parameters_.k3 +
                                          theta2 * parameters_.k4)));
}

double KannalaBrandt::distortionDerivative(double theta) const {
  const double theta2 = theta * theta;
  const double theta4 = theta2 * theta2;
  const double theta6 = theta4 * theta2;
  const double theta8 = theta4 * theta4;
  return 1.0 + 3.0 * parameters_.k1 * theta2 +
         5.0 * parameters_.k2 * theta4 +
         7.0 * parameters_.k3 * theta6 +
         9.0 * parameters_.k4 * theta8;
}

bool KannalaBrandt::undistortAngle(double theta_d, double* theta) const {
  if (!theta || !std::isfinite(theta_d) || theta_d < 0.0) return false;
  if (theta_d <= kOpticalAxisEpsilon) {
    *theta = 0.0;
    return true;
  }

  double lower = 0.0;
  double upper = kMaximumTheta;
  const double upper_residual = distortAngle(upper) - theta_d;
  if (!std::isfinite(upper_residual) || upper_residual < 0.0) return false;

  double estimate = std::min(theta_d, upper);
  for (int iteration = 0; iteration < kMaximumNewtonIterations; ++iteration) {
    const double residual = distortAngle(estimate) - theta_d;
    if (!std::isfinite(residual)) return false;
    if (std::abs(residual) <= kNewtonTolerance) {
      if (estimate < 0.0 || estimate > kMaximumTheta) return false;
      *theta = estimate;
      return true;
    }

    if (residual < 0.0) {
      lower = estimate;
    } else {
      upper = estimate;
    }

    const double derivative = distortionDerivative(estimate);
    if (!std::isfinite(derivative) || derivative <= kMinimumDerivative) {
      return false;
    }
    const double newton_estimate = estimate - residual / derivative;
    estimate =
        std::isfinite(newton_estimate) && newton_estimate > lower &&
                newton_estimate < upper
            ? newton_estimate
            : 0.5 * (lower + upper);
  }
  return false;
}

}  // namespace sphere_vio
