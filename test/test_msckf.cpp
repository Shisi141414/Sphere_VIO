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

}  // namespace
