#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "sphere_vio/camera/omni_radtan.hpp"

namespace sphere_vio {
namespace {

// SYNTHETIC OMNI RADTAN PARAMETERS. These values are only for numerical
// tests and are not measurements or estimates for the real camera system.
OmniRadtan::Parameters syntheticParameters() {
  OmniRadtan::Parameters parameters;
  parameters.width = 640;
  parameters.height = 480;
  parameters.xi = 1.2;
  parameters.fx = 620.0;
  parameters.fy = 615.0;
  parameters.cx = 319.5;
  parameters.cy = 239.5;
  parameters.k1 = -0.05;
  parameters.k2 = 0.08;
  parameters.p1 = 0.002;
  parameters.p2 = -0.0015;
  return parameters;
}

double bearingAngle(const Eigen::Vector3d& first,
                    const Eigen::Vector3d& second) {
  const double dot = std::max(-1.0, std::min(1.0, first.dot(second)));
  return std::atan2(first.cross(second).norm(), dot);
}

TEST(OmniRadtanTest, ReportsModelMetadata) {
  const OmniRadtan camera(syntheticParameters());
  EXPECT_EQ(640, camera.width());
  EXPECT_EQ(480, camera.height());
  EXPECT_EQ("omni_radtan", camera.modelName());
}

TEST(OmniRadtanTest, OpticalAxisAndPrincipalPointAreConsistent) {
  const OmniRadtan camera(syntheticParameters());
  Eigen::Vector2d pixel = Eigen::Vector2d::Constant(-1.0);
  ASSERT_TRUE(camera.project(Eigen::Vector3d(0.0, 0.0, 3.0), &pixel));
  EXPECT_DOUBLE_EQ(319.5, pixel.x());
  EXPECT_DOUBLE_EQ(239.5, pixel.y());

  Eigen::Vector3d bearing = Eigen::Vector3d::Zero();
  ASSERT_TRUE(camera.unproject(pixel, &bearing));
  EXPECT_TRUE(bearing.isApprox(Eigen::Vector3d::UnitZ(), 1e-15));
  EXPECT_NEAR(1.0, bearing.norm(), 1e-15);
}

TEST(OmniRadtanTest, IntermediateAndImageEdgePixelsRoundTrip) {
  const OmniRadtan camera(syntheticParameters());
  const std::vector<Eigen::Vector2d> pixels{
      {417.25, 181.75}, {0.01, 0.01}, {638.99, 0.01},
      {0.01, 478.99},   {638.99, 478.99}};
  for (const Eigen::Vector2d& original_pixel : pixels) {
    Eigen::Vector3d bearing;
    Eigen::Vector2d projected_pixel;
    ASSERT_TRUE(camera.unproject(original_pixel, &bearing));
    ASSERT_TRUE(camera.project(bearing, &projected_pixel));
    EXPECT_LT((projected_pixel - original_pixel).norm(), 1e-8);
    EXPECT_NEAR(1.0, bearing.norm(), 1e-14);
  }
}

TEST(OmniRadtanTest, RandomPixelRoundTripsMeetPrecisionTarget) {
  const OmniRadtan camera(syntheticParameters());
  std::mt19937 generator(20260715U);
  std::uniform_real_distribution<double> horizontal(0.0, 639.0);
  std::uniform_real_distribution<double> vertical(0.0, 479.0);
  double maximum_error = 0.0;
  double error_sum = 0.0;
  int failures = 0;
  constexpr int kSampleCount = 2000;
  for (int index = 0; index < kSampleCount; ++index) {
    const Eigen::Vector2d original_pixel(horizontal(generator),
                                         vertical(generator));
    Eigen::Vector3d bearing;
    Eigen::Vector2d projected_pixel;
    if (!camera.unproject(original_pixel, &bearing) ||
        !camera.project(bearing, &projected_pixel)) {
      ++failures;
      continue;
    }
    const double error = (projected_pixel - original_pixel).norm();
    maximum_error = std::max(maximum_error, error);
    error_sum += error;
    EXPECT_NEAR(1.0, bearing.norm(), 1e-14);
  }
  const int valid_samples = kSampleCount - failures;
  const double average_error =
      valid_samples > 0 ? error_sum / valid_samples : 0.0;
  std::cout << "Synthetic omni-radtan pixel round-trip: samples="
            << kSampleCount << ", valid=" << valid_samples
            << ", failures=" << failures << ", max=" << maximum_error
            << " px, average=" << average_error << " px" << std::endl;
  EXPECT_EQ(0, failures);
  EXPECT_LT(maximum_error, 1e-8);
  EXPECT_LT(average_error, 1e-10);
}

TEST(OmniRadtanTest, UnitBearingRoundTripsMeetPrecisionTarget) {
  const OmniRadtan camera(syntheticParameters());
  std::mt19937 generator(260715U);
  std::uniform_real_distribution<double> theta_distribution(0.0, 0.5);
  std::uniform_real_distribution<double> azimuth_distribution(
      -3.14159265358979323846, 3.14159265358979323846);
  double maximum_angle = 0.0;
  int failures = 0;
  constexpr int kSampleCount = 1000;
  for (int index = 0; index < kSampleCount; ++index) {
    const double theta = theta_distribution(generator);
    const double azimuth = azimuth_distribution(generator);
    const Eigen::Vector3d original_bearing(
        std::sin(theta) * std::cos(azimuth),
        std::sin(theta) * std::sin(azimuth), std::cos(theta));
    Eigen::Vector2d pixel;
    Eigen::Vector3d recovered_bearing;
    if (!camera.project(original_bearing, &pixel) ||
        !camera.unproject(pixel, &recovered_bearing)) {
      ++failures;
      continue;
    }
    maximum_angle =
        std::max(maximum_angle,
                 bearingAngle(original_bearing, recovered_bearing));
  }
  std::cout << "Synthetic omni-radtan bearing round-trip: samples="
            << kSampleCount << ", failures=" << failures
            << ", max=" << maximum_angle << " rad" << std::endl;
  EXPECT_EQ(0, failures);
  EXPECT_LT(maximum_angle, 1e-12);
}

TEST(OmniRadtanTest, RejectsInvalidPointsPixelsAndOutputPointers) {
  const OmniRadtan camera(syntheticParameters());
  Eigen::Vector2d pixel;
  Eigen::Vector3d bearing;
  EXPECT_FALSE(camera.project(Eigen::Vector3d::Zero(), &pixel));
  EXPECT_FALSE(camera.project(Eigen::Vector3d(0.0, 0.0, -1.0), &pixel));
  EXPECT_FALSE(camera.project(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0),
      &pixel));
  EXPECT_FALSE(camera.project(
      Eigen::Vector3d(0.0, std::numeric_limits<double>::infinity(), 1.0),
      &pixel));
  EXPECT_FALSE(camera.project(Eigen::Vector3d::UnitZ(), nullptr));

  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(-0.01, 100.0), &bearing));
  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(640.0, 100.0), &bearing));
  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(100.0, 480.0), &bearing));
  EXPECT_FALSE(camera.unproject(
      Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 1.0),
      &bearing));
  EXPECT_FALSE(camera.unproject(
      Eigen::Vector2d(1.0, std::numeric_limits<double>::infinity()),
      &bearing));
  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(319.5, 239.5), nullptr));
}

TEST(OmniRadtanTest, RejectsProjectionOutsideImage) {
  const OmniRadtan camera(syntheticParameters());
  Eigen::Vector2d pixel;
  EXPECT_FALSE(camera.project(Eigen::Vector3d(1.0, 0.0, 0.01), &pixel));
}

TEST(OmniRadtanTest, InvalidParametersThrow) {
  const auto expect_invalid = [](const OmniRadtan::Parameters& invalid) {
    EXPECT_THROW(
        {
          const OmniRadtan camera(invalid);
          static_cast<void>(camera);
        },
        std::invalid_argument);
  };

  OmniRadtan::Parameters parameters = syntheticParameters();
  parameters.width = 0;
  expect_invalid(parameters);
  parameters = syntheticParameters();
  parameters.height = -1;
  expect_invalid(parameters);
  parameters = syntheticParameters();
  parameters.fx = 0.0;
  expect_invalid(parameters);
  parameters = syntheticParameters();
  parameters.fy = -1.0;
  expect_invalid(parameters);
  parameters = syntheticParameters();
  parameters.xi = -0.01;
  expect_invalid(parameters);
  parameters = syntheticParameters();
  parameters.xi = std::numeric_limits<double>::infinity();
  expect_invalid(parameters);
  parameters = syntheticParameters();
  parameters.cx = std::numeric_limits<double>::quiet_NaN();
  expect_invalid(parameters);
  parameters = syntheticParameters();
  parameters.p2 = std::numeric_limits<double>::infinity();
  expect_invalid(parameters);
}

TEST(OmniRadtanTest, SingularRadtanInverseFailsSafely) {
  OmniRadtan::Parameters parameters = syntheticParameters();
  parameters.fx = 100.0;
  parameters.fy = 100.0;
  parameters.k1 = -4.0 / 3.0;
  parameters.k2 = 0.0;
  parameters.p1 = 0.0;
  parameters.p2 = 0.0;
  const OmniRadtan camera(parameters);
  Eigen::Vector3d bearing;
  EXPECT_FALSE(camera.unproject(
      Eigen::Vector2d(parameters.cx + 50.0, parameters.cy), &bearing));
}

TEST(OmniRadtanTest, VisibilityBoundaryAndSmallDenominatorFailSafely) {
  OmniRadtan::Parameters parameters = syntheticParameters();
  parameters.xi = 0.5;
  const OmniRadtan camera(parameters);
  Eigen::Vector2d pixel;
  const Eigen::Vector3d boundary_point(std::sqrt(0.75), 0.0, -0.5);
  EXPECT_FALSE(camera.project(boundary_point, &pixel));
  EXPECT_FALSE(camera.project(Eigen::Vector3d(0.0, 0.0, -1.0), &pixel));
}

TEST(OmniRadtanTest, OpticalAxisNeighborhoodStaysFinite) {
  const OmniRadtan camera(syntheticParameters());
  Eigen::Vector2d pixel;
  ASSERT_TRUE(camera.project(Eigen::Vector3d(1e-14, -1e-14, 1.0), &pixel));
  EXPECT_TRUE(pixel.allFinite());
  EXPECT_NEAR(319.5, pixel.x(), 1e-11);
  EXPECT_NEAR(239.5, pixel.y(), 1e-11);

  Eigen::Vector3d bearing;
  ASSERT_TRUE(camera.unproject(Eigen::Vector2d(319.5 + 1e-14, 239.5),
                               &bearing));
  EXPECT_TRUE(bearing.allFinite());
  EXPECT_NEAR(1.0, bearing.norm(), 1e-15);
}

struct RealCameraParameters {
  std::uint32_t project_camera_id;
  int kalibr_camera_id;
  const char* name;
  OmniRadtan::Parameters parameters;
};

std::vector<RealCameraParameters> realCameraParametersInProjectOrder() {
  const auto make_parameters =
      [](double xi, double fx, double fy, double cx, double cy, double k1,
         double k2, double p1, double p2) {
        OmniRadtan::Parameters parameters;
        parameters.width = 1088;
        parameters.height = 880;
        parameters.xi = xi;
        parameters.fx = fx;
        parameters.fy = fy;
        parameters.cx = cx;
        parameters.cy = cy;
        parameters.k1 = k1;
        parameters.k2 = k2;
        parameters.p1 = p1;
        parameters.p2 = p2;
        return parameters;
      };

  // Explicit mapping: project C0/C1/C2/C3 <- Kalibr cam0/cam1/cam3/cam2.
  return {
      {0U, 0, "left",
       make_parameters(3.233059595767585, 1693.1762522217418,
                       1680.9070887244018, 544.7595776954244,
                       457.4638042684071, -0.028535050640404982,
                       0.20152680316245483, -0.004062808634440269,
                       0.0008236825668135692)},
      {1U, 1, "right",
       make_parameters(3.159866122804018, 1660.4661219448435,
                       1647.86870918612, 542.0213426089924,
                       451.36780302391327, -0.06220476561956501,
                       0.24391468755547677, -0.0038608433547756703,
                       -0.0033763768099867968)},
      {2U, 3, "bleft",
       make_parameters(3.2053735217233097, 1672.601540116884,
                       1659.978815569607, 536.4198177131659,
                       426.8521699443293, -0.07611565373281055,
                       0.4950567988166919, -0.0030949424971219715,
                       0.002340714875600291)},
      {3U, 2, "bright",
       make_parameters(3.193179479606374, 1671.2264917981015,
                       1658.6247270085908, 545.6604413419071,
                       419.90983067712887, -0.0962939649434078,
                       0.6053676143002162, -0.003790679573829382,
                       -0.00023587592315509642)},
  };
}

std::vector<int> regularCoordinates(int size, int step) {
  std::vector<int> coordinates;
  for (int coordinate = 0; coordinate < size; coordinate += step) {
    coordinates.push_back(coordinate);
  }
  if (coordinates.empty() || coordinates.back() != size - 1) {
    coordinates.push_back(size - 1);
  }
  return coordinates;
}

TEST(OmniRadtanTest, RealCalibrationPixelRoundTripsUseExplicitMapping) {
  const std::vector<RealCameraParameters> calibrations =
      realCameraParametersInProjectOrder();
  ASSERT_EQ(4U, calibrations.size());
  EXPECT_EQ(0, calibrations[0].kalibr_camera_id);
  EXPECT_EQ(1, calibrations[1].kalibr_camera_id);
  EXPECT_EQ(3, calibrations[2].kalibr_camera_id);
  EXPECT_EQ(2, calibrations[3].kalibr_camera_id);

  for (const RealCameraParameters& calibration : calibrations) {
    const OmniRadtan camera(calibration.parameters);
    const std::vector<int> horizontal =
        regularCoordinates(calibration.parameters.width, 16);
    const std::vector<int> vertical =
        regularCoordinates(calibration.parameters.height, 16);
    int samples = 0;
    int valid = 0;
    int invalid_model_domain = 0;
    int round_trip_failures = 0;
    double maximum_error = 0.0;
    double error_sum = 0.0;
    double maximum_norm_error = 0.0;

    for (const int v : vertical) {
      for (const int u : horizontal) {
        ++samples;
        const Eigen::Vector2d original_pixel(u, v);
        Eigen::Vector3d bearing;
        if (!camera.unproject(original_pixel, &bearing)) {
          ++invalid_model_domain;
          continue;
        }
        ++valid;
        maximum_norm_error =
            std::max(maximum_norm_error, std::abs(bearing.norm() - 1.0));
        Eigen::Vector2d projected_pixel;
        if (!camera.project(bearing, &projected_pixel)) {
          ++round_trip_failures;
          continue;
        }
        const double error = (projected_pixel - original_pixel).norm();
        maximum_error = std::max(maximum_error, error);
        error_sum += error;
      }
    }

    const double average_error =
        valid > round_trip_failures
            ? error_sum / static_cast<double>(valid - round_trip_failures)
            : 0.0;
    std::cout << "Real omni-radtan C" << calibration.project_camera_id
              << " <- Kalibr cam" << calibration.kalibr_camera_id << " ("
              << calibration.name << "): samples=" << samples
              << ", valid=" << valid
              << ", invalid_model_domain=" << invalid_model_domain
              << ", round_trip_failures=" << round_trip_failures
              << ", max_pixel_error=" << maximum_error
              << " px, average_pixel_error=" << average_error
              << " px, max_bearing_norm_error=" << maximum_norm_error
              << std::endl;

    EXPECT_EQ(3864, samples);
    EXPECT_GT(valid, 3000);
    EXPECT_EQ(0, round_trip_failures);
    EXPECT_LT(maximum_error, 1e-7);
    EXPECT_LT(average_error, 1e-9);
    EXPECT_LT(maximum_norm_error, 1e-14);
  }
}

}  // namespace
}  // namespace sphere_vio
