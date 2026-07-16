#include <algorithm>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "frontend_test_utils.hpp"
#include "sphere_vio/frontend/feature_detector.hpp"

namespace sphere_vio {
namespace {

TEST(FeatureDetectorTest, BlankImageAndFeatureLimitAreHandled) {
  const CameraRig rig = frontend_test::makeRig();
  FeatureDetector detector;
  std::vector<FeatureObservation> observations;
  FeatureDetectionStatistics statistics;
  ASSERT_TRUE(detector.detect(cv::Mat(240, 320, CV_8UC1, cv::Scalar(0)),
                              1.0, 0U, rig, {}, 250U, &observations,
                              &statistics));
  EXPECT_TRUE(observations.empty());
  EXPECT_EQ(0U, statistics.accepted_count);

  ASSERT_TRUE(detector.detect(frontend_test::makeCheckerTexture(), 1.0, 0U,
                              rig, {}, 17U, &observations, &statistics));
  EXPECT_EQ(17U, observations.size());
}

TEST(FeatureDetectorTest, GridDistributionDistanceAndBearingsAreValid) {
  FeatureDetectorOptions options;
  options.maximum_features = 96U;
  options.minimum_feature_distance = 14.0;
  FeatureDetector detector(options);
  const CameraRig rig = frontend_test::makeRig();
  std::vector<FeatureObservation> observations;
  FeatureDetectionStatistics statistics;
  ASSERT_TRUE(detector.detect(frontend_test::makeCheckerTexture(), 2.0, 2U,
                              rig, {}, 96U, &observations, &statistics));
  ASSERT_GT(observations.size(), 24U);
  EXPECT_LE(observations.size(), 96U);
  EXPECT_GT(statistics.border_rejections, 0U);

  std::set<std::pair<int, int>> occupied_cells;
  for (std::size_t i = 0; i < observations.size(); ++i) {
    const FeatureObservation& observation = observations[i];
    EXPECT_EQ(2U, observation.camera_id);
    EXPECT_EQ(2.0, observation.timestamp);
    EXPECT_GE(observation.pixel.x(), options.border_margin);
    EXPECT_GE(observation.pixel.y(), options.border_margin);
    EXPECT_LT(observation.pixel.x(), 320 - options.border_margin);
    EXPECT_LT(observation.pixel.y(), 240 - options.border_margin);
    EXPECT_NEAR(1.0, observation.bearing_c.norm(), 1e-12);
    EXPECT_NEAR(1.0, observation.bearing_b.norm(), 1e-12);
    occupied_cells.emplace(
        static_cast<int>(observation.pixel.y() * options.grid_rows / 240),
        static_cast<int>(observation.pixel.x() * options.grid_columns / 320));
    for (std::size_t j = 0; j < i; ++j) {
      EXPECT_GE((observation.pixel - observations[j].pixel).norm(),
                options.minimum_feature_distance - 1e-9);
    }
  }
  EXPECT_GE(occupied_cells.size(), 20U);
}

TEST(FeatureDetectorTest, ExistingTracksMaskNearbyRedetections) {
  FeatureDetectorOptions options;
  options.maximum_features = 80U;
  options.minimum_feature_distance = 18.0;
  FeatureDetector detector(options);
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat image = frontend_test::makeCheckerTexture();
  std::vector<FeatureObservation> initial;
  FeatureDetectionStatistics statistics;
  ASSERT_TRUE(detector.detect(image, 1.0, 0U, rig, {}, 25U, &initial,
                              &statistics));
  std::vector<Eigen::Vector2d> occupied;
  for (const FeatureObservation& observation : initial)
    occupied.push_back(observation.pixel);

  std::vector<FeatureObservation> added;
  ASSERT_TRUE(detector.detect(image, 1.0, 0U, rig, occupied, 80U, &added,
                              &statistics));
  EXPECT_GT(statistics.distance_rejections, 0U);
  for (const FeatureObservation& observation : added) {
    for (const Eigen::Vector2d& existing : occupied) {
      EXPECT_GE((observation.pixel - existing).norm(),
                options.minimum_feature_distance - 1e-9);
    }
  }
}

TEST(FeatureDetectorTest, IdenticalInputHasDeterministicOrdering) {
  const FeatureDetector detector;
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat image = frontend_test::makeCheckerTexture();
  std::vector<FeatureObservation> first;
  std::vector<FeatureObservation> second;
  FeatureDetectionStatistics first_statistics;
  FeatureDetectionStatistics second_statistics;
  ASSERT_TRUE(detector.detect(image, 1.0, 0U, rig, {}, 100U, &first,
                              &first_statistics));
  ASSERT_TRUE(detector.detect(image, 1.0, 0U, rig, {}, 100U, &second,
                              &second_statistics));
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i)
    EXPECT_TRUE(first[i].pixel.isApprox(second[i].pixel, 0.0));
  EXPECT_EQ(first_statistics.candidate_count,
            second_statistics.candidate_count);
}

TEST(FeatureDetectorTest, InvalidImagesAndArgumentsFailWithoutOutput) {
  const FeatureDetector detector;
  const CameraRig rig = frontend_test::makeRig();
  std::vector<FeatureObservation> observations(1U);
  FeatureDetectionStatistics statistics;
  EXPECT_FALSE(detector.detect(cv::Mat(), 1.0, 0U, rig, {}, 10U,
                               &observations, &statistics));
  EXPECT_TRUE(observations.empty());
  EXPECT_FALSE(detector.detect(cv::Mat(240, 320, CV_8UC3), 1.0, 0U, rig, {},
                               10U, &observations, &statistics));
  EXPECT_FALSE(detector.detect(frontend_test::makeCheckerTexture(), 1.0, 9U,
                               rig, {}, 10U, &observations, &statistics));
  EXPECT_FALSE(detector.detect(frontend_test::makeCheckerTexture(), 1.0, 0U,
                               rig, {}, 10U, nullptr, &statistics));
}

}  // namespace
}  // namespace sphere_vio
