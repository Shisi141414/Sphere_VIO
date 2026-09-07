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
  std::vector<sphere_vio::LandmarkTrack> tracks;
  sphere_vio::CameraRig rig;
  EXPECT_FALSE(msckf.update(tracks, rig));
}

}  // namespace
