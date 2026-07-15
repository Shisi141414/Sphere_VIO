#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/geometry/spherical_geometry.hpp"

namespace sphere_vio {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHalfPi = 0.5 * kPi;
constexpr int kErpWidth = 2048;
constexpr int kErpHeight = 1024;

std::string realCameraConfigPath() {
  return std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml";
}

double robustBearingAngle(const Eigen::Vector3d& first,
                          const Eigen::Vector3d& second) {
  const Eigen::Vector3d normalized_first = first.normalized();
  const Eigen::Vector3d normalized_second = second.normalized();
  const double dot = std::max(
      -1.0, std::min(1.0, normalized_first.dot(normalized_second)));
  return std::atan2(normalized_first.cross(normalized_second).norm(), dot);
}

double periodicCoordinateError(double first, double second, double period) {
  double difference = std::fmod(first - second, period);
  if (difference < -0.5 * period) difference += period;
  if (difference >= 0.5 * period) difference -= period;
  return std::abs(difference);
}

TEST(SphericalGeometryTest, CardinalAxesAndPolesFollowConvention) {
  struct Case {
    Eigen::Vector3d bearing;
    double longitude;
    double latitude;
  };
  const std::vector<Case> cases{
      {Eigen::Vector3d::UnitX(), 0.0, 0.0},
      {Eigen::Vector3d::UnitY(), kHalfPi, 0.0},
      {-Eigen::Vector3d::UnitY(), -kHalfPi, 0.0},
      {-Eigen::Vector3d::UnitX(), -kPi, 0.0},
      {Eigen::Vector3d::UnitZ(), 0.0, kHalfPi},
      {-Eigen::Vector3d::UnitZ(), 0.0, -kHalfPi},
  };

  for (const Case& test_case : cases) {
    Eigen::Vector2d longitude_latitude;
    ASSERT_TRUE(bearingToLongitudeLatitude(test_case.bearing,
                                           &longitude_latitude));
    EXPECT_NEAR(test_case.longitude, longitude_latitude.x(), 1e-15);
    EXPECT_NEAR(test_case.latitude, longitude_latitude.y(), 1e-15);

    Eigen::Vector3d recovered;
    ASSERT_TRUE(longitudeLatitudeToBearing(longitude_latitude, &recovered));
    EXPECT_LT(robustBearingAngle(test_case.bearing, recovered), 1e-15);
    EXPECT_NEAR(1.0, recovered.norm(), 1e-15);
  }
}

TEST(SphericalGeometryTest, LongitudeWrappingIsPeriodicAndSeamSafe) {
  double wrapped = 0.0;
  ASSERT_TRUE(wrapLongitude(kPi, &wrapped));
  EXPECT_DOUBLE_EQ(-kPi, wrapped);
  ASSERT_TRUE(wrapLongitude(-kPi, &wrapped));
  EXPECT_DOUBLE_EQ(-kPi, wrapped);
  ASSERT_TRUE(wrapLongitude(5.0 * kPi, &wrapped));
  EXPECT_NEAR(-kPi, wrapped, 1e-15);
  ASSERT_TRUE(wrapLongitude(-4.25 * kPi, &wrapped));
  EXPECT_GE(wrapped, -kPi);
  EXPECT_LT(wrapped, kPi);
  ASSERT_TRUE(wrapLongitude(std::numeric_limits<double>::max(), &wrapped));
  EXPECT_GE(wrapped, -kPi);
  EXPECT_LT(wrapped, kPi);

  constexpr double kSeamOffset = 1e-8;
  double difference = 0.0;
  ASSERT_TRUE(wrappedLongitudeDifference(-kPi + kSeamOffset,
                                         kPi - kSeamOffset, &difference));
  EXPECT_NEAR(2.0 * kSeamOffset, difference, 1e-15);
}

TEST(SphericalGeometryTest, LongitudeLatitudeRoundTripsRepresentativeAngles) {
  const std::vector<Eigen::Vector2d> longitude_latitudes{
      {0.3, -0.7}, {-2.8, 1.1}, {kPi - 1e-10, -1.2},
      {-kPi + 1e-10, 1.2}, {7.0 * kPi + 0.2, 0.4}};
  for (const Eigen::Vector2d& original : longitude_latitudes) {
    Eigen::Vector3d bearing;
    ASSERT_TRUE(longitudeLatitudeToBearing(original, &bearing));
    Eigen::Vector2d recovered;
    ASSERT_TRUE(bearingToLongitudeLatitude(bearing, &recovered));
    double longitude_error = 0.0;
    ASSERT_TRUE(wrappedLongitudeDifference(recovered.x(), original.x(),
                                           &longitude_error));
    EXPECT_LT(std::abs(longitude_error), 1e-14);
    EXPECT_NEAR(original.y(), recovered.y(), 1e-14);
  }
}

TEST(SphericalGeometryTest, RandomBearingRoundTripsMeetPrecisionTarget) {
  std::mt19937 generator(20260717U);
  std::normal_distribution<double> component(0.0, 1.0);
  double maximum_bearing_angle = 0.0;
  double maximum_longitude_error = 0.0;
  double maximum_latitude_error = 0.0;
  double maximum_erp_bearing_angle = 0.0;
  int failures = 0;
  constexpr int kSampleCount = 10000;

  for (int index = 0; index < kSampleCount; ++index) {
    Eigen::Vector3d original(component(generator), component(generator),
                             component(generator));
    if (original.norm() < 1e-9) {
      --index;
      continue;
    }
    original.normalize();
    Eigen::Vector2d longitude_latitude;
    Eigen::Vector3d recovered;
    if (!bearingToLongitudeLatitude(original, &longitude_latitude) ||
        !longitudeLatitudeToBearing(longitude_latitude, &recovered)) {
      ++failures;
      continue;
    }
    maximum_bearing_angle = std::max(
        maximum_bearing_angle, robustBearingAngle(original, recovered));
    Eigen::Vector2d recovered_longitude_latitude;
    ASSERT_TRUE(bearingToLongitudeLatitude(recovered,
                                           &recovered_longitude_latitude));
    double longitude_error = 0.0;
    ASSERT_TRUE(wrappedLongitudeDifference(
        recovered_longitude_latitude.x(), longitude_latitude.x(),
        &longitude_error));
    maximum_longitude_error =
        std::max(maximum_longitude_error, std::abs(longitude_error));
    maximum_latitude_error =
        std::max(maximum_latitude_error,
                 std::abs(recovered_longitude_latitude.y() -
                          longitude_latitude.y()));

    Eigen::Vector2d erp;
    Eigen::Vector3d erp_recovered;
    ASSERT_TRUE(bearingToEquirectangular(original, kErpWidth, kErpHeight,
                                         &erp));
    ASSERT_TRUE(equirectangularToBearing(erp, kErpWidth, kErpHeight,
                                         &erp_recovered));
    maximum_erp_bearing_angle =
        std::max(maximum_erp_bearing_angle,
                 robustBearingAngle(original, erp_recovered));
  }

  std::cout << "Synthetic spherical bearing round-trip: samples="
            << kSampleCount << ", failures=" << failures
            << ", max_bearing_angle=" << maximum_bearing_angle
            << " rad, max_longitude_error=" << maximum_longitude_error
            << " rad, max_latitude_error=" << maximum_latitude_error
            << " rad, max_ERP_bearing_angle="
            << maximum_erp_bearing_angle << " rad" << std::endl;
  EXPECT_EQ(0, failures);
  EXPECT_LT(maximum_bearing_angle, 1e-14);
  EXPECT_LT(maximum_longitude_error, 1e-14);
  EXPECT_LT(maximum_latitude_error, 1e-14);
  EXPECT_LT(maximum_erp_bearing_angle, 1e-14);
}

TEST(SphericalGeometryTest, NonUnitBearingsAreNormalized) {
  const Eigen::Vector3d non_unit(3.0, -4.0, 5.0);
  Eigen::Vector2d longitude_latitude;
  ASSERT_TRUE(bearingToLongitudeLatitude(non_unit, &longitude_latitude));
  Eigen::Vector3d bearing;
  ASSERT_TRUE(longitudeLatitudeToBearing(longitude_latitude, &bearing));
  EXPECT_NEAR(1.0, bearing.norm(), 1e-15);
  EXPECT_LT(robustBearingAngle(non_unit, bearing), 1e-15);
}

TEST(SphericalGeometryTest, ErpCoordinatesUseContinuousPeriodicDefinition) {
  Eigen::Vector2d erp;
  ASSERT_TRUE(bearingToEquirectangular(-Eigen::Vector3d::UnitX(),
                                       kErpWidth, kErpHeight, &erp));
  EXPECT_DOUBLE_EQ(0.0, erp.x());
  EXPECT_DOUBLE_EQ(0.5 * kErpHeight, erp.y());
  ASSERT_TRUE(bearingToEquirectangular(Eigen::Vector3d::UnitX(),
                                       kErpWidth, kErpHeight, &erp));
  EXPECT_DOUBLE_EQ(0.5 * kErpWidth, erp.x());
  EXPECT_DOUBLE_EQ(0.5 * kErpHeight, erp.y());
  ASSERT_TRUE(bearingToEquirectangular(Eigen::Vector3d::UnitZ(),
                                       kErpWidth, kErpHeight, &erp));
  EXPECT_DOUBLE_EQ(0.5 * kErpWidth, erp.x());
  EXPECT_DOUBLE_EQ(0.0, erp.y());
  ASSERT_TRUE(bearingToEquirectangular(-Eigen::Vector3d::UnitZ(),
                                       kErpWidth, kErpHeight, &erp));
  EXPECT_DOUBLE_EQ(0.5 * kErpWidth, erp.x());
  EXPECT_DOUBLE_EQ(static_cast<double>(kErpHeight), erp.y());

  Eigen::Vector3d from_zero;
  Eigen::Vector3d from_width;
  Eigen::Vector3d from_negative;
  ASSERT_TRUE(equirectangularToBearing(Eigen::Vector2d(0.0, 512.0),
                                       kErpWidth, kErpHeight, &from_zero));
  ASSERT_TRUE(equirectangularToBearing(
      Eigen::Vector2d(kErpWidth, 512.0), kErpWidth, kErpHeight, &from_width));
  ASSERT_TRUE(equirectangularToBearing(
      Eigen::Vector2d(-kErpWidth, 512.0), kErpWidth, kErpHeight,
      &from_negative));
  EXPECT_LT(robustBearingAngle(from_zero, from_width), 1e-15);
  EXPECT_LT(robustBearingAngle(from_zero, from_negative), 1e-15);
}

TEST(SphericalGeometryTest, RandomErpCoordinatesRoundTripContinuously) {
  std::mt19937 generator(17072026U);
  std::uniform_real_distribution<double> horizontal(-2.0 * kErpWidth,
                                                     3.0 * kErpWidth);
  std::uniform_real_distribution<double> vertical(1e-8,
                                                   kErpHeight - 1e-8);
  double maximum_erp_error = 0.0;
  int failures = 0;
  constexpr int kSampleCount = 10000;

  for (int index = 0; index < kSampleCount; ++index) {
    const Eigen::Vector2d original(horizontal(generator), vertical(generator));
    Eigen::Vector3d bearing;
    Eigen::Vector2d recovered;
    if (!equirectangularToBearing(original, kErpWidth, kErpHeight,
                                  &bearing) ||
        !bearingToEquirectangular(bearing, kErpWidth, kErpHeight,
                                  &recovered)) {
      ++failures;
      continue;
    }
    const double horizontal_error = periodicCoordinateError(
        recovered.x(), original.x(), static_cast<double>(kErpWidth));
    const double vertical_error = std::abs(recovered.y() - original.y());
    maximum_erp_error =
        std::max(maximum_erp_error, std::hypot(horizontal_error,
                                               vertical_error));
  }

  std::cout << "Synthetic continuous ERP round-trip: samples="
            << kSampleCount << ", failures=" << failures
            << ", max_coordinate_error=" << maximum_erp_error << std::endl;
  EXPECT_EQ(0, failures);
  EXPECT_LT(maximum_erp_error, 1e-9);
}

TEST(SphericalGeometryTest, AngularDistanceUsesGreatCircleAngle) {
  double angle = -1.0;
  ASSERT_TRUE(angularDistance(Eigen::Vector3d::UnitX(),
                              3.0 * Eigen::Vector3d::UnitX(), &angle));
  EXPECT_NEAR(0.0, angle, 1e-15);
  ASSERT_TRUE(angularDistance(Eigen::Vector3d::UnitX(),
                              -Eigen::Vector3d::UnitX(), &angle));
  EXPECT_NEAR(kPi, angle, 1e-15);
  ASSERT_TRUE(angularDistance(Eigen::Vector3d::UnitX(),
                              Eigen::Vector3d::UnitY(), &angle));
  EXPECT_NEAR(kHalfPi, angle, 1e-15);

  Eigen::Vector3d seam_a;
  Eigen::Vector3d seam_b;
  ASSERT_TRUE(longitudeLatitudeToBearing(
      Eigen::Vector2d(-kPi + 1e-6, 0.2), &seam_a));
  ASSERT_TRUE(longitudeLatitudeToBearing(
      Eigen::Vector2d(kPi - 1e-6, 0.2), &seam_b));
  ASSERT_TRUE(angularDistance(seam_a, seam_b, &angle));
  EXPECT_LT(angle, 2.1e-6);
}

TEST(SphericalGeometryTest, InvalidInputsAndNullOutputsFailSafely) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  Eigen::Vector2d two_dimensional;
  Eigen::Vector3d bearing;
  double scalar = 0.0;

  EXPECT_FALSE(wrapLongitude(nan, &scalar));
  EXPECT_FALSE(wrapLongitude(infinity, &scalar));
  EXPECT_FALSE(wrapLongitude(0.0, nullptr));
  EXPECT_FALSE(wrappedLongitudeDifference(0.0, nan, &scalar));
  EXPECT_FALSE(wrappedLongitudeDifference(0.0, 0.0, nullptr));
  EXPECT_FALSE(bearingToLongitudeLatitude(Eigen::Vector3d::Zero(),
                                          &two_dimensional));
  EXPECT_FALSE(bearingToLongitudeLatitude(Eigen::Vector3d(nan, 0.0, 1.0),
                                          &two_dimensional));
  EXPECT_FALSE(bearingToLongitudeLatitude(Eigen::Vector3d::UnitX(), nullptr));
  EXPECT_FALSE(longitudeLatitudeToBearing(Eigen::Vector2d(0.0, kHalfPi + 0.1),
                                          &bearing));
  EXPECT_FALSE(longitudeLatitudeToBearing(Eigen::Vector2d(infinity, 0.0),
                                          &bearing));
  EXPECT_FALSE(longitudeLatitudeToBearing(Eigen::Vector2d::Zero(), nullptr));
  EXPECT_FALSE(bearingToEquirectangular(Eigen::Vector3d::UnitX(), 0, 100,
                                        &two_dimensional));
  EXPECT_FALSE(bearingToEquirectangular(Eigen::Vector3d::UnitX(), 100, -1,
                                        &two_dimensional));
  EXPECT_FALSE(bearingToEquirectangular(Eigen::Vector3d::UnitX(), 100, 100,
                                        nullptr));
  EXPECT_FALSE(equirectangularToBearing(Eigen::Vector2d(0.0, -0.1), 100, 50,
                                        &bearing));
  EXPECT_FALSE(equirectangularToBearing(Eigen::Vector2d(0.0, 50.1), 100, 50,
                                        &bearing));
  EXPECT_FALSE(equirectangularToBearing(Eigen::Vector2d(nan, 20.0), 100, 50,
                                        &bearing));
  EXPECT_FALSE(equirectangularToBearing(Eigen::Vector2d::Zero(), 100, 50,
                                        nullptr));
  EXPECT_FALSE(angularDistance(Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::UnitX(), &scalar));
  EXPECT_FALSE(angularDistance(Eigen::Vector3d::UnitX(),
                               Eigen::Vector3d::UnitY(), nullptr));
}

struct RealCameraSampleConfig {
  CameraId camera_id = 0;
  int kalibr_camera_id = -1;
  std::string name;
  Eigen::Vector2d principal_point = Eigen::Vector2d::Zero();
};

std::vector<RealCameraSampleConfig> readRealCameraSampleConfig() {
  const YAML::Node cameras = YAML::LoadFile(realCameraConfigPath())["cameras"];
  std::vector<RealCameraSampleConfig> result;
  for (const YAML::Node& camera : cameras) {
    RealCameraSampleConfig config;
    config.camera_id = camera["camera_id"].as<CameraId>();
    config.kalibr_camera_id = camera["kalibr_camera_id"].as<int>();
    config.name = camera["name"].as<std::string>();
    config.principal_point =
        Eigen::Vector2d(camera["intrinsics"][3].as<double>(),
                        camera["intrinsics"][4].as<double>());
    result.push_back(config);
  }
  return result;
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

TEST(SphericalGeometryTest, RealRigGridSamplesRoundTripThroughBodyErp) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &error))
      << error;
  const std::vector<RealCameraSampleConfig> camera_configs =
      readRealCameraSampleConfig();
  ASSERT_EQ(4U, camera_configs.size());

  for (const RealCameraSampleConfig& config : camera_configs) {
    const RigCamera* camera = rig.camera(config.camera_id);
    ASSERT_NE(nullptr, camera);
    const std::vector<int> horizontal =
        regularCoordinates(camera->model->width(), 16);
    const std::vector<int> vertical =
        regularCoordinates(camera->model->height(), 16);
    int sample_count = 0;
    int model_domain_invalid = 0;
    int successful_bearings = 0;
    int conversion_failures = 0;
    double maximum_bearing_angle = 0.0;
    Eigen::Vector2d minimum_longitude_latitude =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector2d maximum_longitude_latitude =
        Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity());
    Eigen::Vector2d minimum_erp =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector2d maximum_erp =
        Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity());

    for (const int v : vertical) {
      for (const int u : horizontal) {
        ++sample_count;
        Eigen::Vector3d bearing_b;
        if (!rig.pixelToBodyBearing(config.camera_id,
                                    Eigen::Vector2d(u, v), &bearing_b)) {
          ++model_domain_invalid;
          continue;
        }
        Eigen::Vector2d longitude_latitude;
        Eigen::Vector2d erp;
        Eigen::Vector3d recovered_bearing_b;
        if (!bearingToLongitudeLatitude(bearing_b, &longitude_latitude) ||
            !bearingToEquirectangular(bearing_b, kErpWidth, kErpHeight,
                                      &erp) ||
            !equirectangularToBearing(erp, kErpWidth, kErpHeight,
                                      &recovered_bearing_b)) {
          ++conversion_failures;
          continue;
        }
        ++successful_bearings;
        maximum_bearing_angle =
            std::max(maximum_bearing_angle,
                     robustBearingAngle(bearing_b, recovered_bearing_b));
        minimum_longitude_latitude =
            minimum_longitude_latitude.cwiseMin(longitude_latitude);
        maximum_longitude_latitude =
            maximum_longitude_latitude.cwiseMax(longitude_latitude);
        minimum_erp = minimum_erp.cwiseMin(erp);
        maximum_erp = maximum_erp.cwiseMax(erp);
      }
    }

    std::cout << "Real spherical coverage C" << config.camera_id
              << " <- Kalibr cam" << config.kalibr_camera_id << " ("
              << config.name << "): samples=" << sample_count
              << ", model_domain_invalid=" << model_domain_invalid
              << ", valid=" << successful_bearings
              << ", conversion_failures=" << conversion_failures
              << ", max_bearing_angle=" << maximum_bearing_angle
              << " rad, longitude=[" << minimum_longitude_latitude.x()
              << ", " << maximum_longitude_latitude.x() << "]"
              << ", latitude=[" << minimum_longitude_latitude.y() << ", "
              << maximum_longitude_latitude.y() << "]"
              << ", ERP u=[" << minimum_erp.x() << ", " << maximum_erp.x()
              << "] v=[" << minimum_erp.y() << ", " << maximum_erp.y()
              << "]" << std::endl;

    EXPECT_EQ(3864, sample_count);
    EXPECT_GT(model_domain_invalid, 0);
    EXPECT_GT(successful_bearings, 3000);
    EXPECT_EQ(sample_count,
              model_domain_invalid + successful_bearings +
                  conversion_failures);
    EXPECT_EQ(0, conversion_failures);
    EXPECT_LT(maximum_bearing_angle, 1e-14);
  }
}

TEST(SphericalGeometryTest, RealCameraCenterBearingsCoverDistinctDirections) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &error))
      << error;
  const std::vector<RealCameraSampleConfig> camera_configs =
      readRealCameraSampleConfig();
  std::array<Eigen::Vector3d, 4> center_bearings;

  const int expected_kalibr_ids[4] = {0, 1, 3, 2};
  for (const RealCameraSampleConfig& config : camera_configs) {
    ASSERT_EQ(expected_kalibr_ids[config.camera_id], config.kalibr_camera_id);
    Eigen::Vector3d bearing_b;
    Eigen::Vector2d longitude_latitude;
    Eigen::Vector2d erp;
    ASSERT_TRUE(rig.pixelToBodyBearing(config.camera_id,
                                       config.principal_point, &bearing_b));
    ASSERT_TRUE(bearingToLongitudeLatitude(bearing_b, &longitude_latitude));
    ASSERT_TRUE(bearingToEquirectangular(bearing_b, kErpWidth, kErpHeight,
                                         &erp));
    center_bearings[config.camera_id] = bearing_b;
    std::cout << "Real camera center C" << config.camera_id
              << " <- Kalibr cam" << config.kalibr_camera_id << " ("
              << config.name << "): bearing_b=[" << bearing_b.transpose()
              << "], longitude=" << longitude_latitude.x()
              << ", latitude=" << longitude_latitude.y() << ", ERP=["
              << erp.transpose() << "]" << std::endl;
    EXPECT_TRUE(bearing_b.allFinite());
    EXPECT_NEAR(1.0, bearing_b.norm(), 1e-14);
  }

  double minimum_pair_angle = kPi;
  for (std::size_t first = 0; first < center_bearings.size(); ++first) {
    for (std::size_t second = first + 1; second < center_bearings.size();
         ++second) {
      double angle = 0.0;
      ASSERT_TRUE(
          angularDistance(center_bearings[first], center_bearings[second],
                          &angle));
      minimum_pair_angle = std::min(minimum_pair_angle, angle);
    }
  }
  std::cout << "Minimum real camera center-bearing separation: "
            << minimum_pair_angle << " rad" << std::endl;
  EXPECT_GT(minimum_pair_angle, 1.4);
}

}  // namespace
}  // namespace sphere_vio
