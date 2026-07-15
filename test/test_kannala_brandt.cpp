#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "sphere_vio/camera/kannala_brandt.hpp"

namespace sphere_vio {
namespace {

// Synthetic parameters used only for numerical unit tests. They are not
// measurements or estimates for the real four-camera system.
KannalaBrandt::Parameters syntheticParameters() {
  KannalaBrandt::Parameters parameters;
  parameters.width = 640;
  parameters.height = 480;
  parameters.fx = 320.0;
  parameters.fy = 315.0;
  parameters.cx = 319.5;
  parameters.cy = 239.5;
  parameters.k1 = 0.01;
  parameters.k2 = -0.001;
  parameters.k3 = 0.0001;
  parameters.k4 = -0.000005;
  return parameters;
}

double bearingAngle(const Eigen::Vector3d& first,
                    const Eigen::Vector3d& second) {
  const double dot = std::max(-1.0, std::min(1.0, first.dot(second)));
  return std::atan2(first.cross(second).norm(), dot);
}

TEST(KannalaBrandtTest, ReportsModelMetadata) {
  const KannalaBrandt camera(syntheticParameters());
  EXPECT_EQ(640, camera.width());
  EXPECT_EQ(480, camera.height());
  EXPECT_EQ("KannalaBrandtKB4", camera.modelName());
}

TEST(KannalaBrandtTest, PositiveOpticalAxisProjectsToPrincipalPoint) {
  const KannalaBrandt camera(syntheticParameters());
  Eigen::Vector2d pixel = Eigen::Vector2d::Constant(-1.0);
  ASSERT_TRUE(camera.project(Eigen::Vector3d(0.0, 0.0, 3.0), &pixel));
  EXPECT_DOUBLE_EQ(319.5, pixel.x());
  EXPECT_DOUBLE_EQ(239.5, pixel.y());
}

TEST(KannalaBrandtTest, PrincipalPointUnprojectsToPositiveOpticalAxis) {
  const KannalaBrandt camera(syntheticParameters());
  Eigen::Vector3d bearing = Eigen::Vector3d::Zero();
  ASSERT_TRUE(camera.unproject(Eigen::Vector2d(319.5, 239.5), &bearing));
  EXPECT_TRUE(bearing.isApprox(Eigen::Vector3d::UnitZ(), 1e-15));
}

TEST(KannalaBrandtTest, IntermediatePixelRoundTripIsAccurate) {
  const KannalaBrandt camera(syntheticParameters());
  const Eigen::Vector2d original_pixel(417.25, 181.75);
  Eigen::Vector3d bearing;
  Eigen::Vector2d projected_pixel;
  ASSERT_TRUE(camera.unproject(original_pixel, &bearing));
  ASSERT_TRUE(camera.project(bearing, &projected_pixel));
  EXPECT_LT((projected_pixel - original_pixel).norm(), 1e-9);
}

TEST(KannalaBrandtTest, ImageEdgePixelRoundTripsAreAccurate) {
  const KannalaBrandt camera(syntheticParameters());
  const std::vector<Eigen::Vector2d> pixels{
      {0.01, 0.01}, {638.99, 0.01}, {0.01, 478.99}, {638.99, 478.99}};
  for (const Eigen::Vector2d& original_pixel : pixels) {
    Eigen::Vector3d bearing;
    Eigen::Vector2d projected_pixel;
    ASSERT_TRUE(camera.unproject(original_pixel, &bearing));
    ASSERT_TRUE(camera.project(bearing, &projected_pixel));
    EXPECT_LT((projected_pixel - original_pixel).norm(), 1e-8);
  }
}

TEST(KannalaBrandtTest, RandomPixelRoundTripsMeetPrecisionTarget) {
  const KannalaBrandt camera(syntheticParameters());
  std::mt19937 generator(20260715U);
  std::uniform_real_distribution<double> horizontal(0.0, 639.0);
  std::uniform_real_distribution<double> vertical(0.0, 479.0);
  double maximum_error = 0.0;
  double error_sum = 0.0;
  constexpr int kSampleCount = 2000;
  for (int index = 0; index < kSampleCount; ++index) {
    const Eigen::Vector2d original_pixel(horizontal(generator),
                                         vertical(generator));
    Eigen::Vector3d bearing;
    Eigen::Vector2d projected_pixel;
    ASSERT_TRUE(camera.unproject(original_pixel, &bearing));
    ASSERT_TRUE(camera.project(bearing, &projected_pixel));
    const double error = (projected_pixel - original_pixel).norm();
    maximum_error = std::max(maximum_error, error);
    error_sum += error;
    EXPECT_NEAR(1.0, bearing.norm(), 1e-14);
  }
  const double average_error = error_sum / kSampleCount;
  std::cout << "Synthetic KB4 pixel round-trip: max=" << maximum_error
            << " px, average=" << average_error << " px" << std::endl;
  EXPECT_LT(maximum_error, 1e-6);
  EXPECT_LT(average_error, 1e-8);
}

TEST(KannalaBrandtTest, UnitBearingRoundTripsMeetPrecisionTarget) {
  const KannalaBrandt camera(syntheticParameters());
  std::mt19937 generator(260715U);
  std::uniform_real_distribution<double> theta_distribution(0.0, 0.7);
  std::uniform_real_distribution<double> azimuth_distribution(
      -3.14159265358979323846, 3.14159265358979323846);
  double maximum_angle = 0.0;
  constexpr int kSampleCount = 1000;
  for (int index = 0; index < kSampleCount; ++index) {
    const double theta = theta_distribution(generator);
    const double azimuth = azimuth_distribution(generator);
    const Eigen::Vector3d original_bearing(
        std::sin(theta) * std::cos(azimuth),
        std::sin(theta) * std::sin(azimuth), std::cos(theta));
    Eigen::Vector2d pixel;
    Eigen::Vector3d recovered_bearing;
    ASSERT_TRUE(camera.project(original_bearing, &pixel));
    ASSERT_TRUE(camera.unproject(pixel, &recovered_bearing));
    maximum_angle =
        std::max(maximum_angle,
                 bearingAngle(original_bearing, recovered_bearing));
  }
  std::cout << "Synthetic KB4 bearing round-trip: max=" << maximum_angle
            << " rad" << std::endl;
  EXPECT_LT(maximum_angle, 1e-12);
}

TEST(KannalaBrandtTest, RejectsZeroAndNonFiniteThreeDimensionalPoints) {
  const KannalaBrandt camera(syntheticParameters());
  Eigen::Vector2d pixel;
  EXPECT_FALSE(camera.project(Eigen::Vector3d::Zero(), &pixel));
  EXPECT_FALSE(camera.project(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0),
      &pixel));
  EXPECT_FALSE(camera.project(
      Eigen::Vector3d(0.0, std::numeric_limits<double>::infinity(), 1.0),
      &pixel));
  EXPECT_FALSE(camera.project(Eigen::Vector3d(0.0, 0.0, -1.0), &pixel));
}

TEST(KannalaBrandtTest, RejectsInvalidPixelsAndNonFinitePixels) {
  const KannalaBrandt camera(syntheticParameters());
  Eigen::Vector3d bearing;
  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(-0.01, 100.0), &bearing));
  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(640.0, 100.0), &bearing));
  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(100.0, 480.0), &bearing));
  EXPECT_FALSE(camera.unproject(
      Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 1.0),
      &bearing));
  EXPECT_FALSE(camera.unproject(
      Eigen::Vector2d(1.0, std::numeric_limits<double>::infinity()),
      &bearing));
  EXPECT_FALSE(camera.isPixelValid(Eigen::Vector2d(
      1.0, std::numeric_limits<double>::infinity())));
}

TEST(KannalaBrandtTest, RejectsProjectionOutsideImage) {
  const KannalaBrandt camera(syntheticParameters());
  Eigen::Vector2d pixel;
  EXPECT_FALSE(camera.project(Eigen::Vector3d(1.0, 0.0, 0.0), &pixel));
}

TEST(KannalaBrandtTest, NullOutputPointersFailSafely) {
  const KannalaBrandt camera(syntheticParameters());
  EXPECT_FALSE(camera.project(Eigen::Vector3d::UnitZ(), nullptr));
  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(319.5, 239.5), nullptr));
}

TEST(KannalaBrandtTest, InvalidParametersThrow) {
  const auto expect_invalid = [](const KannalaBrandt::Parameters& invalid) {
    EXPECT_THROW(
        {
          const KannalaBrandt camera(invalid);
          static_cast<void>(camera);
        },
        std::invalid_argument);
  };
  KannalaBrandt::Parameters parameters = syntheticParameters();
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
  parameters.cx = std::numeric_limits<double>::quiet_NaN();
  expect_invalid(parameters);
  parameters = syntheticParameters();
  parameters.k4 = std::numeric_limits<double>::infinity();
  expect_invalid(parameters);
}

TEST(KannalaBrandtTest, NonInvertibleDistortionFailsSafely) {
  KannalaBrandt::Parameters parameters = syntheticParameters();
  parameters.k1 = -1.0;
  parameters.k2 = 0.0;
  parameters.k3 = 0.0;
  parameters.k4 = 0.0;
  const KannalaBrandt camera(parameters);
  Eigen::Vector3d bearing;
  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(500.0, 239.5), &bearing));
}

TEST(KannalaBrandtTest, OpticalAxisNeighborhoodStaysFinite) {
  const KannalaBrandt camera(syntheticParameters());
  Eigen::Vector2d pixel;
  ASSERT_TRUE(camera.project(Eigen::Vector3d(1e-14, -1e-14, 1.0), &pixel));
  EXPECT_TRUE(pixel.allFinite());
  EXPECT_DOUBLE_EQ(319.5, pixel.x());
  EXPECT_DOUBLE_EQ(239.5, pixel.y());

  Eigen::Vector3d bearing;
  ASSERT_TRUE(camera.unproject(Eigen::Vector2d(319.5 + 1e-14, 239.5),
                               &bearing));
  EXPECT_TRUE(bearing.allFinite());
  EXPECT_NEAR(1.0, bearing.norm(), 1e-15);
}

}  // namespace
}  // namespace sphere_vio
