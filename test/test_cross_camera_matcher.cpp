#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "frontend_test_utils.hpp"
#include "sphere_vio/camera/kannala_brandt.hpp"
#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/geometry/epipolar_geometry.hpp"

namespace sphere_vio {
namespace {

CrossCameraMatcherOptions logicOptions() {
  CrossCameraMatcherOptions options;
  options.camera_pairs = {{0U, 1U}, {2U, 3U}};
  options.maximum_descriptor_distance = 256.0;
  options.ratio_test = 0.8;
  options.maximum_epipolar_angle = 1.5707963267948966;
  return options;
}

cv::Mat makeDescriptors(const std::vector<std::vector<std::uint8_t>>& rows) {
  cv::Mat descriptors(static_cast<int>(rows.size()), 32, CV_8UC1,
                      cv::Scalar(0));
  for (std::size_t row = 0U; row < rows.size(); ++row) {
    for (std::size_t column = 0U;
         column < rows[row].size() && column < 32U; ++column) {
      descriptors.at<std::uint8_t>(static_cast<int>(row),
                                   static_cast<int>(column)) = rows[row][column];
    }
  }
  return descriptors;
}

std::vector<std::uint8_t> filled(std::uint8_t value) {
  return std::vector<std::uint8_t>(32U, value);
}

CameraDescriptorSet makeSet(CameraId camera_id,
                            const std::vector<FeatureId>& ids,
                            const cv::Mat& descriptors,
                            const std::vector<Eigen::Vector3d>& bearings = {}) {
  CameraDescriptorSet set;
  set.camera_id = camera_id;
  set.timestamp = 1.0;
  set.feature_ids = ids;
  set.descriptors = descriptors.clone();
  for (std::size_t index = 0U; index < ids.size(); ++index) {
    set.pixels.emplace_back(50.0 + 10.0 * index, 60.0 + 5.0 * index);
    set.bearings_c.push_back(
        bearings.empty() ? Eigen::Vector3d::UnitZ() : bearings[index]);
    set.bearings_b.push_back(set.bearings_c.back());
  }
  return set;
}

TEST(CrossCameraMatcherTest, IdenticalDescriptorsPassAllFilters) {
  const cv::Mat descriptors =
      makeDescriptors({filled(0x00U), filled(0xFFU)});
  const CameraDescriptorSet first = makeSet(0U, {10U, 11U}, descriptors);
  const CameraDescriptorSet second = makeSet(1U, {20U, 21U}, descriptors);
  CrossCameraMatcher matcher(logicOptions());
  CrossCameraPairResult result;
  ASSERT_TRUE(matcher.matchPair(first, second, frontend_test::makeRig(),
                                &result));
  ASSERT_EQ(2U, result.matches.size());
  EXPECT_EQ(2U, result.raw_candidates);
  EXPECT_EQ(2U, result.absolute_distance_accepted);
  EXPECT_EQ(2U, result.ratio_accepted);
  EXPECT_EQ(2U, result.mutual_accepted);
  EXPECT_EQ(2U, result.epipolar_accepted);
  EXPECT_EQ(0.0, result.matches[0].descriptor_distance);
  EXPECT_EQ(10U, result.matches[0].feature_id_1);
  EXPECT_EQ(20U, result.matches[0].feature_id_2);
}

TEST(CrossCameraMatcherTest, AbsoluteDistanceAndRatioFiltersAreIndependent) {
  CrossCameraMatcherOptions absolute_options = logicOptions();
  absolute_options.maximum_descriptor_distance = 10.0;
  CrossCameraMatcher absolute_matcher(absolute_options);
  const CameraDescriptorSet source = makeSet(
      0U, {1U, 2U}, makeDescriptors({filled(0x00U), filled(0xFFU)}));
  const CameraDescriptorSet distant = makeSet(
      1U, {3U, 4U}, makeDescriptors({filled(0x0FU), filled(0xF0U)}));
  CrossCameraPairResult result;
  ASSERT_TRUE(absolute_matcher.matchPair(source, distant,
                                         frontend_test::makeRig(), &result));
  EXPECT_EQ(2U, result.rejected_absolute_distance);
  EXPECT_TRUE(result.matches.empty());

  CrossCameraMatcher ratio_matcher(logicOptions());
  const CameraDescriptorSet ambiguous = makeSet(
      1U, {3U, 4U}, makeDescriptors({filled(0x00U), filled(0x00U)}));
  ASSERT_TRUE(ratio_matcher.matchPair(source, ambiguous,
                                      frontend_test::makeRig(), &result));
  EXPECT_EQ(2U, result.rejected_ratio);
  EXPECT_TRUE(result.matches.empty());
}

TEST(CrossCameraMatcherTest, FewerThanTwoCandidatesCannotPassRatio) {
  CrossCameraMatcher matcher(logicOptions());
  const CameraDescriptorSet source = makeSet(
      0U, {1U, 2U}, makeDescriptors({filled(0x00U), filled(0xFFU)}));
  const CameraDescriptorSet target =
      makeSet(1U, {3U}, makeDescriptors({filled(0x00U)}));
  CrossCameraPairResult result;
  ASSERT_TRUE(matcher.matchPair(source, target, frontend_test::makeRig(),
                                &result));
  EXPECT_EQ(2U, result.rejected_ratio);
  EXPECT_TRUE(result.matches.empty());
}

TEST(CrossCameraMatcherTest, NonMutualBestIsRejected) {
  CrossCameraMatcher matcher(logicOptions());
  std::vector<std::uint8_t> one_bit = filled(0x00U);
  one_bit[0] = 0x01U;
  const CameraDescriptorSet source = makeSet(
      0U, {1U, 2U}, makeDescriptors({filled(0x00U), one_bit}));
  const CameraDescriptorSet target = makeSet(
      1U, {3U, 4U}, makeDescriptors({filled(0x00U), filled(0xFFU)}));
  CrossCameraPairResult result;
  ASSERT_TRUE(matcher.matchPair(source, target, frontend_test::makeRig(),
                                &result));
  EXPECT_EQ(1U, result.matches.size());
  EXPECT_EQ(1U, result.rejected_non_mutual);
  EXPECT_EQ(1U, result.matches.front().feature_id_1);
}

TEST(CrossCameraMatcherTest, PairLocalFeatureIdConflictsUseStableRanking) {
  CrossCameraMatcher matcher(logicOptions());
  const cv::Mat descriptors =
      makeDescriptors({filled(0x00U), filled(0xFFU)});
  CrossCameraPairResult result;
  ASSERT_TRUE(matcher.matchPair(makeSet(0U, {7U, 7U}, descriptors),
                                makeSet(1U, {20U, 21U}, descriptors),
                                frontend_test::makeRig(), &result));
  ASSERT_EQ(1U, result.matches.size());
  EXPECT_EQ(1U, result.rejected_duplicate);
  EXPECT_EQ(20U, result.matches.front().feature_id_2);

  ASSERT_TRUE(matcher.matchPair(makeSet(0U, {7U, 8U}, descriptors),
                                makeSet(1U, {20U, 20U}, descriptors),
                                frontend_test::makeRig(), &result));
  ASSERT_EQ(1U, result.matches.size());
  EXPECT_EQ(1U, result.rejected_duplicate);
  EXPECT_EQ(7U, result.matches.front().feature_id_1);
}

TEST(CrossCameraMatcherTest, EmptyAndInvalidDescriptorSetsAreSafe) {
  CrossCameraMatcher matcher(logicOptions());
  CameraDescriptorSet empty_0 = makeSet(0U, {}, cv::Mat());
  CameraDescriptorSet empty_1 = makeSet(1U, {}, cv::Mat());
  CrossCameraPairResult result;
  ASSERT_TRUE(matcher.matchPair(empty_0, empty_1, frontend_test::makeRig(),
                                &result));
  EXPECT_TRUE(result.matches.empty());

  CameraDescriptorSet invalid_type =
      makeSet(0U, {1U}, cv::Mat(1, 32, CV_32FC1));
  CameraDescriptorSet valid =
      makeSet(1U, {2U}, makeDescriptors({filled(0x00U)}));
  EXPECT_FALSE(matcher.matchPair(invalid_type, valid,
                                 frontend_test::makeRig(), &result));
  CameraDescriptorSet wrong_length =
      makeSet(0U, {1U}, cv::Mat(1, 16, CV_8UC1));
  EXPECT_FALSE(matcher.matchPair(wrong_length, valid,
                                 frontend_test::makeRig(), &result));
  valid.camera_id = 0U;
  EXPECT_FALSE(
      matcher.matchPair(wrong_length, valid, frontend_test::makeRig(), &result));
}

TEST(CrossCameraMatcherTest, UnconfiguredAndSameCameraPairsAreRejected) {
  CrossCameraMatcher matcher(logicOptions());
  const cv::Mat descriptors =
      makeDescriptors({filled(0x00U), filled(0xFFU)});
  CrossCameraPairResult result;
  EXPECT_FALSE(matcher.matchPair(makeSet(0U, {1U, 2U}, descriptors),
                                 makeSet(3U, {3U, 4U}, descriptors),
                                 frontend_test::makeRig(), &result));
  EXPECT_FALSE(matcher.matchPair(makeSet(0U, {1U, 2U}, descriptors),
                                 makeSet(0U, {3U, 4U}, descriptors),
                                 frontend_test::makeRig(), &result));
  EXPECT_FALSE(matcher.isConfiguredPair(0U, 3U));
  EXPECT_TRUE(matcher.isConfiguredPair(1U, 0U));
}

bool realPointDescriptorSet(const CameraRig& rig, CameraId camera_id,
                            const std::vector<Eigen::Vector3d>& points_b,
                            const std::vector<FeatureId>& ids,
                            const cv::Mat& descriptors,
                            CameraDescriptorSet* set) {
  if (!set || points_b.size() != ids.size()) return false;
  *set = CameraDescriptorSet{};
  set->camera_id = camera_id;
  set->timestamp = 2.0;
  set->feature_ids = ids;
  set->descriptors = descriptors.clone();
  const RigCamera* camera = rig.camera(camera_id);
  if (!camera) return false;
  for (const Eigen::Vector3d& point_b : points_b) {
    Eigen::Vector3d point_c;
    Eigen::Vector2d pixel;
    if (!rig.bodyPointToCamera(camera_id, point_b, &point_c) ||
        !camera->model->project(point_c, &pixel)) {
      return false;
    }
    set->pixels.push_back(pixel);
    set->bearings_c.push_back(point_c.normalized());
    set->bearings_b.push_back((point_b - camera->t_b_c).normalized());
  }
  return true;
}

TEST(CrossCameraMatcherTest, RealRigCorrectCorrespondencesPassEpipolarFilter) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(
      std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml", &rig,
      &error))
      << error;
  const std::vector<Eigen::Vector3d> points_b{{5.0, 0.0, 0.0},
                                               {5.0, 0.0, 1.0}};
  const cv::Mat descriptors =
      makeDescriptors({filled(0x00U), filled(0xFFU)});
  CameraDescriptorSet first;
  CameraDescriptorSet second;
  ASSERT_TRUE(realPointDescriptorSet(rig, 0U, points_b, {100U, 101U},
                                     descriptors, &first));
  ASSERT_TRUE(realPointDescriptorSet(rig, 1U, points_b, {200U, 201U},
                                     descriptors, &second));
  CrossCameraMatcherOptions options = logicOptions();
  options.maximum_epipolar_angle = 1e-8;
  CrossCameraMatcher matcher(options);
  CrossCameraPairResult result;
  ASSERT_TRUE(matcher.matchPair(first, second, rig, &result));
  ASSERT_EQ(2U, result.matches.size());
  EXPECT_LT(result.matches[0].epipolar_error_maximum, 1e-10);
  EXPECT_LT(result.matches[1].epipolar_error_maximum, 1e-10);
}

TEST(CrossCameraMatcherTest, EpipolarFilterRejectsDescriptorSimilarOutlier) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(
      std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml", &rig,
      &error));
  const std::vector<Eigen::Vector3d> source_points{{5.0, 0.0, -1.0},
                                                   {5.0, 0.0, 2.0}};
  const std::vector<Eigen::Vector3d> target_points{{5.0, 0.0, 2.0},
                                                   {5.0, 0.0, -1.0}};
  const cv::Mat descriptors =
      makeDescriptors({filled(0x00U), filled(0xFFU)});
  CameraDescriptorSet first;
  CameraDescriptorSet second;
  ASSERT_TRUE(realPointDescriptorSet(rig, 0U, source_points, {1U, 2U},
                                     descriptors, &first));
  ASSERT_TRUE(realPointDescriptorSet(rig, 1U, target_points, {3U, 4U},
                                     descriptors, &second));
  CrossCameraMatcherOptions options = logicOptions();
  options.maximum_epipolar_angle = 0.003;
  CrossCameraMatcher matcher(options);
  CrossCameraPairResult result;
  ASSERT_TRUE(matcher.matchPair(first, second, rig, &result));
  EXPECT_TRUE(result.matches.empty());
  EXPECT_EQ(2U, result.rejected_epipolar);
}

TEST(CrossCameraMatcherTest, GreatCircleCandidateNeedsDescriptorDisambiguation) {
  const CameraRig rig = frontend_test::makeRig();
  const RigCamera* camera_0 = rig.camera(0U);
  const RigCamera* camera_1 = rig.camera(1U);
  ASSERT_NE(nullptr, camera_0);
  ASSERT_NE(nullptr, camera_1);
  RelativePose pose;
  ASSERT_TRUE(relativeCameraPose(*camera_0, *camera_1, &pose));
  Eigen::Vector3d normal;
  ASSERT_TRUE(epipolarPlaneNormal(Eigen::Vector3d::UnitZ(),
                                  pose.R_target_source,
                                  pose.t_target_source, &normal));
  std::vector<Eigen::Vector3d> great_circle;
  ASSERT_TRUE(sampleGreatCircle(normal, 32U, &great_circle));
  const cv::Mat descriptors =
      makeDescriptors({filled(0x00U), filled(0xFFU)});
  const CameraDescriptorSet first = makeSet(
      0U, {1U, 2U}, descriptors,
      {Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()});
  const CameraDescriptorSet second = makeSet(
      1U, {3U, 4U}, descriptors, {great_circle[3], great_circle[11]});
  CrossCameraMatcherOptions options = logicOptions();
  options.maximum_epipolar_angle = 1e-8;
  CrossCameraMatcher matcher(options);
  CrossCameraPairResult result;
  ASSERT_TRUE(matcher.matchPair(first, second, rig, &result));
  EXPECT_GE(result.matches.size(), 1U);
  EXPECT_LT(result.matches.front().epipolar_error_maximum, 1e-8);
}

TEST(CrossCameraMatcherTest, DegenerateGeometryAndC2C3MappingAreExplicit) {
  const CameraRig source_rig = frontend_test::makeRig();
  CameraRig degenerate;
  RigCamera first = *source_rig.camera(0U);
  RigCamera second = *source_rig.camera(1U);
  second.t_b_c = first.t_b_c;
  ASSERT_TRUE(degenerate.addCamera(first));
  ASSERT_TRUE(degenerate.addCamera(second));
  const cv::Mat descriptors =
      makeDescriptors({filled(0x00U), filled(0xFFU)});
  CrossCameraMatcher matcher(logicOptions());
  CrossCameraPairResult result;
  ASSERT_TRUE(matcher.matchPair(makeSet(0U, {1U, 2U}, descriptors),
                                makeSet(1U, {3U, 4U}, descriptors), degenerate,
                                &result));
  EXPECT_EQ(2U, result.rejected_degenerate_geometry);

  CameraRig real_rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(
      std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml", &real_rig,
      &error));
  ASSERT_EQ(3, real_rig.camera(2U)->kalibr_camera_id);
  ASSERT_EQ(2, real_rig.camera(3U)->kalibr_camera_id);
  const std::vector<Eigen::Vector3d> points_b{{-5.0, 0.0, 0.0},
                                               {-5.0, 0.0, 1.0}};
  CameraDescriptorSet c2;
  CameraDescriptorSet c3;
  ASSERT_TRUE(realPointDescriptorSet(real_rig, 2U, points_b, {20U, 21U},
                                     descriptors, &c2));
  ASSERT_TRUE(realPointDescriptorSet(real_rig, 3U, points_b, {30U, 31U},
                                     descriptors, &c3));
  CrossCameraMatcherOptions options = logicOptions();
  options.maximum_epipolar_angle = 1e-8;
  CrossCameraMatcher real_matcher(options);
  ASSERT_TRUE(real_matcher.matchPair(c2, c3, real_rig, &result));
  EXPECT_EQ(2U, result.matches.size());
}

TEST(CrossCameraMatcherTest, ConfiguredPairBatchOrderIsDeterministic) {
  CrossCameraMatcher matcher(logicOptions());
  const cv::Mat descriptors =
      makeDescriptors({filled(0x00U), filled(0xFFU)});
  std::vector<CameraDescriptorSet> sets{
      makeSet(3U, {30U, 31U}, descriptors),
      makeSet(1U, {10U, 11U}, descriptors),
      makeSet(2U, {20U, 21U}, descriptors),
      makeSet(0U, {1U, 2U}, descriptors)};
  std::vector<CrossCameraPairResult> first;
  std::vector<CrossCameraPairResult> second;
  ASSERT_TRUE(
      matcher.matchConfiguredPairs(sets, frontend_test::makeRig(), &first));
  ASSERT_TRUE(
      matcher.matchConfiguredPairs(sets, frontend_test::makeRig(), &second));
  ASSERT_EQ(2U, first.size());
  EXPECT_EQ(0U, first[0].camera_id_1);
  EXPECT_EQ(1U, first[0].camera_id_2);
  EXPECT_EQ(2U, first[1].camera_id_1);
  EXPECT_EQ(3U, first[1].camera_id_2);
  ASSERT_EQ(first[0].matches.size(), second[0].matches.size());
  for (std::size_t index = 0U; index < first[0].matches.size(); ++index) {
    EXPECT_EQ(first[0].matches[index].feature_id_1,
              second[0].matches[index].feature_id_1);
    EXPECT_EQ(first[0].matches[index].feature_id_2,
              second[0].matches[index].feature_id_2);
  }
}

}  // namespace
}  // namespace sphere_vio
