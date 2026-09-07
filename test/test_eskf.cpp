#include "sphere_vio/backend/eskf.hpp"
#include "sphere_vio/backend/landmark_map.hpp"

#include "gtest/gtest.h"

namespace {

using sphere_vio::BackendLandmarkMap;
using sphere_vio::Eskf;
using sphere_vio::EskfOptions;
using sphere_vio::ImuMeasurement;
using sphere_vio::LandmarkTrack;
using sphere_vio::LandmarkTrackId;

ImuMeasurement makeImu(double time, double acceleration_z = 9.81) {
  ImuMeasurement measurement;
  measurement.timestamp = time;
  measurement.acceleration = Eigen::Vector3d::Zero();
  measurement.acceleration.z() = acceleration_z;
  measurement.angular_velocity = Eigen::Vector3d::Zero();
  return measurement;
}

TEST(EskfTest, PropagatesMeanAndCovariance) {
  EskfOptions options;
  options.initial_covariance = 0.01 * Eigen::Matrix<double, 15, 15>::Identity();
  Eskf eskf(options);

  ASSERT_TRUE(eskf.initialize(makeImu(1.0)));
  std::vector<ImuMeasurement> measurements;
  measurements.push_back(makeImu(1.0));
  measurements.push_back(makeImu(1.01));
  measurements.push_back(makeImu(1.02));
  ASSERT_TRUE(eskf.propagate(measurements, 1.02));

  EXPECT_NEAR(eskf.state().timestamp, 1.02, 1e-9);
  EXPECT_TRUE(eskf.state().q_wb.coeffs().allFinite());
  EXPECT_TRUE(eskf.state().p_wb.allFinite());
  EXPECT_TRUE(eskf.state().v_wb.allFinite());
  EXPECT_TRUE(eskf.covariance().allFinite());
}

TEST(EskfTest, PositionUpdateChangesState) {
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(makeImu(0.0)));
  const Eigen::Vector3d body_point(1.0, 2.0, 3.0);
  const Eigen::Vector3d measured_world(1.5, 2.0, 3.0);
  ASSERT_TRUE(eskf.updatePosition(measured_world, body_point, 0.1));
  EXPECT_TRUE(eskf.state().p_wb.allFinite());
  EXPECT_TRUE(eskf.covariance().allFinite());
}

TEST(LandmarkMapTest, AddsAndReobservesLandmark) {
  BackendLandmarkMap map;
  LandmarkTrack track;
  track.id = LandmarkTrackId{7U};
  track.last_observation_timestamp = 2.0;
  track.has_latest_triangulation_diagnostic = true;
  track.latest_triangulation_diagnostic.admitted = true;
  track.latest_triangulation_diagnostic.triangulation.valid = true;
  track.latest_triangulation_diagnostic.point_b = Eigen::Vector3d(1.0, 0.0, 2.0);

  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(makeImu(0.0)));

  Eigen::Vector3d body;
  Eigen::Vector3d world;
  bool is_new = false;
  ASSERT_TRUE(map.observe(track, eskf.state(), &body, &world, &is_new));
  EXPECT_TRUE(is_new);
  EXPECT_EQ(map.landmarks().size(), 1U);

  ASSERT_TRUE(map.observe(track, eskf.state(), &body, &world, &is_new));
  EXPECT_FALSE(is_new);
  EXPECT_EQ(map.landmarks().size(), 1U);
}

}  // namespace
