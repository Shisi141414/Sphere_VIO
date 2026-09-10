#include "sphere_vio/backend/msckf.hpp"

#include "gtest/gtest.h"

namespace {

using sphere_vio::ImuMeasurement;
using sphere_vio::Msckf;
using sphere_vio::MsckfOptions;

ImuMeasurement makeImu(double time) {
  ImuMeasurement measurement;
  measurement.timestamp = time;
  measurement.acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
  measurement.angular_velocity = Eigen::Vector3d::Zero();
  return measurement;
}

TEST(MsckfTest, PropagatesAugmentsAndMarginalizesClones) {
  MsckfOptions options;
  options.maximum_clones = 2;
  Msckf msckf(options);

  ASSERT_TRUE(msckf.initialize(makeImu(0.0)));
  for (double timestamp = 0.1; timestamp <= 0.4; timestamp += 0.1) {
    std::vector<ImuMeasurement> measurements;
    measurements.push_back(makeImu(timestamp - 0.1));
    measurements.push_back(makeImu(timestamp));
    ASSERT_TRUE(msckf.propagate(measurements, timestamp));
    ASSERT_TRUE(msckf.augmentClone(timestamp));
    msckf.marginalizeOldestClone();
  }

  EXPECT_LE(msckf.cloneCount(), 2U);
  EXPECT_EQ(msckf.covariance().rows(), 15 + 2 * 6);
  EXPECT_TRUE(msckf.covariance().allFinite());
  EXPECT_TRUE(msckf.state().q_wb.coeffs().allFinite());
}

TEST(MsckfTest, EmptyFeatureUpdateIsSafe) {
  Msckf msckf;
  ASSERT_TRUE(msckf.initialize(makeImu(0.0)));
  std::vector<sphere_vio::MsckfFeature> features;
  sphere_vio::CameraRig rig;
  EXPECT_FALSE(msckf.update(features, rig));
}

TEST(MsckfFeatureAccumulatorTest, RecordsAndPrunesObservations) {
  sphere_vio::MsckfFeatureAccumulator accumulator(2U);
  accumulator.add(0U, 10U, 1.0, Eigen::Vector2d(1.0, 2.0), 1U);
  accumulator.add(0U, 10U, 1.1, Eigen::Vector2d(1.1, 2.1), 2U);
  accumulator.add(0U, 10U, 1.2, Eigen::Vector2d(1.2, 2.2), 3U);
  accumulator.add(1U, 11U, 1.0, Eigen::Vector2d(3.0, 4.0), 1U);

  const std::vector<sphere_vio::MsckfObservation>* observations =
      accumulator.observations(0U, 10U);
  ASSERT_NE(observations, nullptr);
  EXPECT_EQ(observations->size(), 2U);
  EXPECT_DOUBLE_EQ(observations->front().timestamp, 1.1);
  EXPECT_DOUBLE_EQ(observations->back().timestamp, 1.2);

  accumulator.prune(5U, 2U);
  EXPECT_EQ(accumulator.observations(1U, 11U), nullptr);
  EXPECT_EQ(accumulator.keys().size(), 1U);
}

TEST(MsckfTest, InitializesGravityAndBiasesFromStationaryWindow) {
  MsckfOptions options;
  options.initialization_duration = 0.5;
  options.minimum_initialization_samples = 3U;
  options.gravity_magnitude = 9.81;
  Msckf msckf(options);

  const Eigen::Vector3d gravity_body =
      Eigen::Vector3d(0.28, 0.35, 0.894).normalized();
  const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.005);
  std::vector<ImuMeasurement> measurements;
  for (double timestamp = 0.0; timestamp <= 0.6; timestamp += 0.1) {
    ImuMeasurement measurement = makeImu(timestamp);
    measurement.acceleration = 9.81 * gravity_body;
    measurement.angular_velocity = gyro_bias;
    measurements.push_back(measurement);
  }

  ASSERT_TRUE(msckf.initialize(measurements, 0.6));
  EXPECT_TRUE(msckf.initialized());
  EXPECT_TRUE(msckf.state().bias_gyro.isApprox(gyro_bias, 1e-6));
  const Eigen::Vector3d recovered_gravity =
      msckf.state().q_wb.conjugate() *
      (9.81 * Eigen::Vector3d::UnitZ());
  EXPECT_TRUE(recovered_gravity.isApprox(9.81 * gravity_body, 1e-4));
  EXPECT_LT(msckf.state().bias_accel.norm(), 1e-4);
}

TEST(MsckfTest, AugmentsLandmarkCovarianceBlock) {
  MsckfOptions options;
  options.maximum_landmarks = 1;
  Msckf msckf(options);
  ASSERT_TRUE(msckf.initialize(makeImu(0.0)));

  EXPECT_TRUE(msckf.augmentLandmark(42U, Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_EQ(msckf.landmarkCount(), 1U);
  EXPECT_EQ(msckf.covariance().rows(), 18);
  EXPECT_TRUE(msckf.covariance().allFinite());
}

}  // namespace
