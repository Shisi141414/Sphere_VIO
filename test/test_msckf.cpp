#include "sphere_vio/backend/msckf.hpp"

#include "gtest/gtest.h"

#include <cmath>

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

TEST(MsckfFeatureAccumulatorTest, DrainsInactiveTracksExactlyOnce) {
  sphere_vio::MsckfFeatureAccumulator accumulator(4U);
  accumulator.add(0U, 10U, 1.0, Eigen::Vector2d(1.0, 2.0), 1U);
  accumulator.add(0U, 10U, 1.1, Eigen::Vector2d(1.1, 2.1), 2U);
  accumulator.add(0U, 10U, 1.2, Eigen::Vector2d(1.2, 2.2), 3U);

  // Still active: frame 4 is within the two-frame inactivity window.
  EXPECT_TRUE(accumulator.drainInactive(4U, 2U).empty());
  EXPECT_EQ(accumulator.keys().size(), 1U);

  // Frame 6 is three frames after the last observation: drain once, remove
  // the track, and never offer the same observations again.
  std::vector<sphere_vio::MsckfFeature> drained =
      accumulator.drainInactive(6U, 2U);
  ASSERT_EQ(drained.size(), 1U);
  EXPECT_EQ(drained[0].persistent_id, 10U);
  EXPECT_EQ(drained[0].observations.size(), 3U);
  EXPECT_TRUE(accumulator.drainInactive(7U, 2U).empty());
  EXPECT_TRUE(accumulator.keys().empty());
}

TEST(MsckfFeatureAccumulatorTest, SegmentsBeforeMarginalization) {
  sphere_vio::MsckfFeatureAccumulator accumulator(4U);
  accumulator.add(0U, 11U, 1.0, Eigen::Vector2d(1.0, 2.0), 1U);
  accumulator.add(0U, 11U, 1.1, Eigen::Vector2d(1.1, 2.1), 2U);
  accumulator.add(0U, 11U, 1.2, Eigen::Vector2d(1.2, 2.2), 3U);

  std::vector<sphere_vio::MsckfFeature> segmented =
      accumulator.segmentBefore(1.1);
  ASSERT_EQ(segmented.size(), 1U);
  EXPECT_EQ(segmented[0].observations.size(), 2U);
  EXPECT_DOUBLE_EQ(segmented[0].observations.front().timestamp, 1.0);
  EXPECT_DOUBLE_EQ(segmented[0].observations.back().timestamp, 1.1);

  // The newer observation remains and is consumed by a second segmentation.
  const std::vector<sphere_vio::MsckfObservation>* retained =
      accumulator.observations(0U, 11U);
  ASSERT_NE(retained, nullptr);
  ASSERT_EQ(retained->size(), 1U);
  EXPECT_DOUBLE_EQ(retained->front().timestamp, 1.2);
  segmented = accumulator.segmentBefore(1.2);
  EXPECT_EQ(segmented.size(), 1U);
  EXPECT_EQ(accumulator.observations(0U, 11U), nullptr);
}

TEST(MsckfTest, StationaryInitializationGateRejectsMovingWindow) {
  MsckfOptions options;
  options.initialization_duration = 0.5;
  options.minimum_initialization_samples = 3U;
  options.gravity_magnitude = 9.81;
  options.stationary_initialization_gate = true;
  options.maximum_accelerometer_deviation = 0.1;
  options.maximum_gyroscope_deviation = 0.01;
  Msckf msckf(options);

  std::vector<ImuMeasurement> measurements;
  for (double timestamp = 0.0; timestamp <= 0.6; timestamp += 0.1) {
    ImuMeasurement measurement = makeImu(timestamp);
    // A swinging acceleration norm should fail the stationarity gate even
    // though the mean still points mostly along gravity.
    measurement.acceleration =
        Eigen::Vector3d(0.0, 0.0, 9.81) +
        Eigen::Vector3d(0.5 * std::sin(4.0 * timestamp), 0.0, 0.0);
    measurements.push_back(measurement);
  }

  EXPECT_FALSE(msckf.initialize(measurements, 0.6));
  EXPECT_FALSE(msckf.initialized());
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
