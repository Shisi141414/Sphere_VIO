#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "frontend_test_utils.hpp"
#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/frontend/triangulation_candidate_evaluator.hpp"

namespace sphere_vio {
namespace {

std::string realCameraConfigPath() {
  return std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml";
}

CameraDescriptorSet makeSet(CameraId camera_id, FeatureId feature_id,
                            const Eigen::Vector3d& bearing_c,
                            double timestamp = 1.0) {
  CameraDescriptorSet set;
  set.camera_id = camera_id;
  set.timestamp = timestamp;
  set.feature_ids.push_back(feature_id);
  set.pixels.emplace_back(100.0 + camera_id, 80.0 + camera_id);
  set.bearings_c.push_back(bearing_c);
  set.bearings_b.push_back(bearing_c);
  return set;
}

CrossCameraMatch makeMatch(CameraId camera_id_1 = 0U,
                           CameraId camera_id_2 = 1U,
                           FeatureId feature_id_1 = 10U,
                           FeatureId feature_id_2 = 20U) {
  CrossCameraMatch match;
  match.camera_id_1 = camera_id_1;
  match.camera_id_2 = camera_id_2;
  match.feature_id_1 = feature_id_1;
  match.feature_id_2 = feature_id_2;
  match.pixel_1 = Eigen::Vector2d(100.0, 80.0);
  match.pixel_2 = Eigen::Vector2d(101.0, 81.0);
  match.descriptor_distance = 12.0;
  match.ratio = 0.5;
  match.epipolar_error_forward = 1e-5;
  match.epipolar_error_backward = 2e-5;
  match.epipolar_error_maximum = 2e-5;
  return match;
}

bool makePointSets(const CameraRig& rig, CameraId camera_id_1,
                   CameraId camera_id_2,
                   const Eigen::Vector3d& point_for_camera_1,
                   const Eigen::Vector3d& point_for_camera_2,
                   CameraDescriptorSet* descriptors_1,
                   CameraDescriptorSet* descriptors_2) {
  if (!descriptors_1 || !descriptors_2) return false;
  const RigCamera* camera_1 = rig.camera(camera_id_1);
  const RigCamera* camera_2 = rig.camera(camera_id_2);
  if (!camera_1 || !camera_2) return false;
  const Eigen::Vector3d direction_1 =
      camera_1->R_b_c.transpose() *
      (point_for_camera_1 - camera_1->t_b_c);
  const Eigen::Vector3d direction_2 =
      camera_2->R_b_c.transpose() *
      (point_for_camera_2 - camera_2->t_b_c);
  *descriptors_1 = makeSet(camera_id_1, 10U, direction_1.normalized());
  *descriptors_2 = makeSet(camera_id_2, 20U, direction_2.normalized());
  return true;
}

TriangulationDiagnostic evaluatePoint(
    const CameraRig& rig, const Eigen::Vector3d& point_1,
    const Eigen::Vector3d& point_2,
    const TriangulationCandidateOptions& options = {}) {
  CameraDescriptorSet first;
  CameraDescriptorSet second;
  EXPECT_TRUE(makePointSets(rig, 0U, 1U, point_1, point_2, &first, &second));
  TriangulationDiagnostic diagnostic;
  EXPECT_TRUE(TriangulationCandidateEvaluator(options).evaluateMatch(
      makeMatch(), first, second, rig, &diagnostic));
  return diagnostic;
}

TEST(TriangulationCandidateEvaluatorTest,
     RealRigExactPointIsAcceptedInBodyFrame) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &error))
      << error;
  const Eigen::Vector3d point_b(5.0, 0.0, 0.4);
  const TriangulationDiagnostic diagnostic =
      evaluatePoint(rig, point_b, point_b);
  EXPECT_TRUE(diagnostic.triangulation_succeeded);
  EXPECT_TRUE(diagnostic.admitted);
  EXPECT_EQ(TriangulationCandidateStatus::kAccepted, diagnostic.status);
  EXPECT_EQ(TriangulationCoordinateFrame::kBody,
            diagnostic.triangulation.coordinate_frame);
  EXPECT_LT((diagnostic.point_b - point_b).norm(), 1e-10);
}

TEST(TriangulationCandidateEvaluatorTest,
     ExactGeometryPreservesDepthAndDerivedMetrics) {
  const CameraRig rig = frontend_test::makeRig();
  const Eigen::Vector3d point_b(0.03, 0.02, 3.0);
  const TriangulationDiagnostic diagnostic =
      evaluatePoint(rig, point_b, point_b);
  ASSERT_TRUE(diagnostic.triangulation_succeeded);
  EXPECT_DOUBLE_EQ(diagnostic.triangulation.depth_1, diagnostic.depth_1);
  EXPECT_DOUBLE_EQ(diagnostic.triangulation.depth_2, diagnostic.depth_2);
  EXPECT_DOUBLE_EQ(std::min(diagnostic.depth_1, diagnostic.depth_2),
                   diagnostic.minimum_depth);
  EXPECT_DOUBLE_EQ(std::max(diagnostic.depth_1, diagnostic.depth_2),
                   diagnostic.maximum_depth);
  EXPECT_NEAR(std::abs(diagnostic.depth_1 - diagnostic.depth_2) /
                  diagnostic.maximum_depth,
              diagnostic.relative_depth_difference, 1e-15);
  EXPECT_NEAR(diagnostic.closest_ray_distance /
                  diagnostic.triangulation.baseline,
              diagnostic.closest_distance_to_baseline_ratio, 1e-15);
}

TEST(TriangulationCandidateEvaluatorTest,
     NegativeRayParametersAreNotCameraZChecks) {
  const CameraRig rig = frontend_test::makeRig();
  const Eigen::Vector3d point_b(0.0, 0.0, 2.0);
  CameraDescriptorSet first;
  CameraDescriptorSet second;
  ASSERT_TRUE(makePointSets(rig, 0U, 1U, point_b, point_b, &first, &second));
  first.bearings_c[0] *= -1.0;
  second.bearings_c[0] *= -1.0;
  TriangulationDiagnostic diagnostic;
  ASSERT_TRUE(TriangulationCandidateEvaluator().evaluateMatch(
      makeMatch(), first, second, rig, &diagnostic));
  EXPECT_FALSE(diagnostic.admitted);
  EXPECT_EQ(TriangulationCandidateStatus::kNegativeDepth,
            diagnostic.status);
  EXPECT_LT(diagnostic.depth_1, 0.0);
  EXPECT_LT(diagnostic.depth_2, 0.0);
}

TEST(TriangulationCandidateEvaluatorTest, NearParallelRaysFailCoreExplicitly) {
  const CameraRig rig = frontend_test::makeRig();
  const RigCamera* first_camera = rig.camera(0U);
  const RigCamera* second_camera = rig.camera(1U);
  ASSERT_NE(nullptr, first_camera);
  ASSERT_NE(nullptr, second_camera);
  const Eigen::Vector3d common_direction = Eigen::Vector3d::UnitZ();
  const CameraDescriptorSet first = makeSet(
      0U, 10U, first_camera->R_b_c.transpose() * common_direction);
  const CameraDescriptorSet second = makeSet(
      1U, 20U, second_camera->R_b_c.transpose() * common_direction);
  TriangulationDiagnostic diagnostic;
  ASSERT_TRUE(TriangulationCandidateEvaluator().evaluateMatch(
      makeMatch(), first, second, rig, &diagnostic));
  EXPECT_EQ(TriangulationCandidateStatus::kTriangulationFailed,
            diagnostic.status);
  EXPECT_FALSE(diagnostic.triangulation_succeeded);
}

TEST(TriangulationCandidateEvaluatorTest, SmallRayAngleGatePrecedesDepthGate) {
  const CameraRig rig = frontend_test::makeRig();
  const Eigen::Vector3d far_point(0.0, 0.0, 100.0);
  const TriangulationDiagnostic diagnostic =
      evaluatePoint(rig, far_point, far_point);
  ASSERT_TRUE(diagnostic.triangulation_succeeded);
  EXPECT_EQ(TriangulationCandidateStatus::kRayAngleTooSmall,
            diagnostic.status);
}

TEST(TriangulationCandidateEvaluatorTest, MinimumDepthGateRejectsNearPoint) {
  const CameraRig rig = frontend_test::makeRig();
  TriangulationCandidateOptions options;
  options.minimum_ray_angle = 0.0;
  const Eigen::Vector3d near_point(0.04, 0.01, 0.03);
  const TriangulationDiagnostic diagnostic =
      evaluatePoint(rig, near_point, near_point, options);
  ASSERT_TRUE(diagnostic.triangulation_succeeded);
  EXPECT_EQ(TriangulationCandidateStatus::kDepthOutOfRange,
            diagnostic.status);
}

TEST(TriangulationCandidateEvaluatorTest, MaximumDepthGateRejectsFarPoint) {
  const CameraRig rig = frontend_test::makeRig();
  TriangulationCandidateOptions options;
  options.minimum_ray_angle = 0.0;
  const Eigen::Vector3d far_point(0.0, 0.0, 60.0);
  const TriangulationDiagnostic diagnostic =
      evaluatePoint(rig, far_point, far_point, options);
  ASSERT_TRUE(diagnostic.triangulation_succeeded);
  EXPECT_EQ(TriangulationCandidateStatus::kDepthOutOfRange,
            diagnostic.status);
}

TEST(TriangulationCandidateEvaluatorTest,
     ClosestRayDistanceGateRejectsSkewRaysFirst) {
  const CameraRig rig = frontend_test::makeRig();
  TriangulationCandidateOptions options;
  options.minimum_ray_angle = 0.0;
  options.maximum_closest_ray_distance = 1e-5;
  options.maximum_angular_reprojection_error = 1.0;
  const TriangulationDiagnostic diagnostic = evaluatePoint(
      rig, Eigen::Vector3d(0.0, 0.0, 3.0),
      Eigen::Vector3d(0.0, 0.03, 3.0), options);
  ASSERT_TRUE(diagnostic.triangulation_succeeded);
  EXPECT_GT(diagnostic.closest_ray_distance,
            options.maximum_closest_ray_distance);
  EXPECT_EQ(TriangulationCandidateStatus::kClosestDistanceTooLarge,
            diagnostic.status);
}

TEST(TriangulationCandidateEvaluatorTest,
     AngularReprojectionGateCanBeIsolatedFromClosestDistance) {
  const CameraRig rig = frontend_test::makeRig();
  TriangulationCandidateOptions options;
  options.minimum_ray_angle = 0.0;
  options.maximum_closest_ray_distance = 10.0;
  options.maximum_angular_reprojection_error = 1e-7;
  const TriangulationDiagnostic diagnostic = evaluatePoint(
      rig, Eigen::Vector3d(0.0, 0.0, 3.0),
      Eigen::Vector3d(0.0, 0.03, 3.0), options);
  ASSERT_TRUE(diagnostic.triangulation_succeeded);
  EXPECT_GT(diagnostic.maximum_angular_reprojection_error,
            options.maximum_angular_reprojection_error);
  EXPECT_EQ(TriangulationCandidateStatus::
                kAngularReprojectionErrorTooLarge,
            diagnostic.status);
}

TEST(TriangulationCandidateEvaluatorTest,
     StoredEpipolarMetricHasItsOwnFinalGate) {
  const CameraRig rig = frontend_test::makeRig();
  const Eigen::Vector3d point_b(0.0, 0.0, 3.0);
  CameraDescriptorSet first;
  CameraDescriptorSet second;
  ASSERT_TRUE(makePointSets(rig, 0U, 1U, point_b, point_b, &first, &second));
  CrossCameraMatch match = makeMatch();
  match.epipolar_error_maximum = 0.004;
  TriangulationDiagnostic diagnostic;
  ASSERT_TRUE(TriangulationCandidateEvaluator().evaluateMatch(
      match, first, second, rig, &diagnostic));
  EXPECT_EQ(TriangulationCandidateStatus::kEpipolarErrorTooLarge,
            diagnostic.status);
}

TEST(TriangulationCandidateEvaluatorTest,
     MissingDuplicateAndWrongPairFeaturesAreInvalidMatches) {
  const CameraRig rig = frontend_test::makeRig();
  const Eigen::Vector3d point_b(0.0, 0.0, 3.0);
  CameraDescriptorSet first;
  CameraDescriptorSet second;
  ASSERT_TRUE(makePointSets(rig, 0U, 1U, point_b, point_b, &first, &second));
  TriangulationCandidateEvaluator evaluator;
  TriangulationDiagnostic diagnostic;
  CrossCameraMatch missing = makeMatch();
  missing.feature_id_2 = 999U;
  ASSERT_TRUE(evaluator.evaluateMatch(missing, first, second, rig,
                                      &diagnostic));
  EXPECT_EQ(TriangulationCandidateStatus::kInvalidMatch, diagnostic.status);
  first.feature_ids.push_back(first.feature_ids.front());
  first.bearings_c.push_back(first.bearings_c.front());
  ASSERT_TRUE(evaluator.evaluateMatch(makeMatch(), first, second, rig,
                                      &diagnostic));
  EXPECT_EQ(TriangulationCandidateStatus::kInvalidMatch, diagnostic.status);
  CrossCameraMatch wrong_pair = makeMatch();
  wrong_pair.camera_id_2 = 2U;
  ASSERT_TRUE(evaluator.evaluateMatch(wrong_pair, first, second, rig,
                                      &diagnostic));
  EXPECT_EQ(TriangulationCandidateStatus::kInvalidMatch, diagnostic.status);
}

TEST(TriangulationCandidateEvaluatorTest, NonFiniteMatchIsNeverHidden) {
  const CameraRig rig = frontend_test::makeRig();
  CameraDescriptorSet first;
  CameraDescriptorSet second;
  const Eigen::Vector3d point_b(0.0, 0.0, 3.0);
  ASSERT_TRUE(makePointSets(rig, 0U, 1U, point_b, point_b, &first, &second));
  CrossCameraMatch match = makeMatch();
  match.ratio = std::numeric_limits<double>::quiet_NaN();
  TriangulationDiagnostic diagnostic;
  ASSERT_TRUE(TriangulationCandidateEvaluator().evaluateMatch(
      match, first, second, rig, &diagnostic));
  EXPECT_EQ(TriangulationCandidateStatus::kInvalidMatch, diagnostic.status);
}

TEST(TriangulationCandidateEvaluatorTest,
     AdmissionReevaluationDoesNotAlterGeometry) {
  const CameraRig rig = frontend_test::makeRig();
  const Eigen::Vector3d point_b(0.0, 0.0, 3.0);
  TriangulationDiagnostic diagnostic = evaluatePoint(rig, point_b, point_b);
  ASSERT_TRUE(diagnostic.admitted);
  const TriangulationResult original = diagnostic.triangulation;
  TriangulationCandidateOptions strict;
  strict.minimum_ray_angle = diagnostic.ray_angle + 1e-6;
  ASSERT_TRUE(applyTriangulationCandidateAdmission(strict, &diagnostic));
  EXPECT_EQ(TriangulationCandidateStatus::kRayAngleTooSmall,
            diagnostic.status);
  EXPECT_DOUBLE_EQ(original.depth_1, diagnostic.triangulation.depth_1);
  EXPECT_TRUE(original.point_common.isApprox(
      diagnostic.triangulation.point_common, 0.0));
}

TEST(TriangulationCandidateEvaluatorTest,
     NonFiniteDiagnosticGetsExplicitAdmissionStatus) {
  TriangulationDiagnostic diagnostic;
  diagnostic.triangulation_succeeded = true;
  diagnostic.status = TriangulationCandidateStatus::kAccepted;
  diagnostic.depth_1 = 1.0;
  diagnostic.depth_2 = 1.0;
  diagnostic.minimum_depth = 1.0;
  diagnostic.maximum_depth = 1.0;
  diagnostic.ray_angle = 0.1;
  diagnostic.epipolar_error = 0.0;
  diagnostic.closest_ray_distance = 0.0;
  diagnostic.relative_depth_difference = 0.0;
  diagnostic.closest_distance_to_baseline_ratio =
      std::numeric_limits<double>::infinity();
  ASSERT_TRUE(applyTriangulationCandidateAdmission(
      TriangulationCandidateOptions{}, &diagnostic));
  EXPECT_EQ(TriangulationCandidateStatus::kNonFiniteResult,
            diagnostic.status);
}

TEST(TriangulationCandidateEvaluatorTest,
     EmptyAndNonEmptyPairResultsAreDeterministic) {
  const CameraRig rig = frontend_test::makeRig();
  const Eigen::Vector3d point_b(0.0, 0.0, 3.0);
  CameraDescriptorSet first;
  CameraDescriptorSet second;
  ASSERT_TRUE(makePointSets(rig, 0U, 1U, point_b, point_b, &first, &second));
  CrossCameraPairResult matches;
  matches.camera_id_1 = 0U;
  matches.camera_id_2 = 1U;
  TriangulationCandidateEvaluator evaluator;
  TriangulationCandidatePairResult empty;
  ASSERT_TRUE(evaluator.evaluatePair(matches, first, second, rig, &empty));
  EXPECT_EQ(0U, empty.input_matches);
  EXPECT_TRUE(empty.diagnostics.empty());
  matches.matches.push_back(makeMatch());
  TriangulationCandidatePairResult run_1;
  TriangulationCandidatePairResult run_2;
  ASSERT_TRUE(evaluator.evaluatePair(matches, first, second, rig, &run_1));
  ASSERT_TRUE(evaluator.evaluatePair(matches, first, second, rig, &run_2));
  ASSERT_EQ(1U, run_1.candidates.size());
  EXPECT_EQ(run_1.diagnostics[0].status, run_2.diagnostics[0].status);
  EXPECT_TRUE(run_1.diagnostics[0].point_b.isApprox(
      run_2.diagnostics[0].point_b, 0.0));
}

TEST(TriangulationCandidateEvaluatorTest,
     QuantilesUseDocumentedInterpolationAndAreEmptySafe) {
  DeterministicQuantiles empty;
  ASSERT_TRUE(computeDeterministicQuantiles({}, &empty));
  EXPECT_FALSE(empty.valid);
  EXPECT_EQ(0U, empty.count);
  DeterministicQuantiles quantiles;
  ASSERT_TRUE(computeDeterministicQuantiles({4.0, 1.0, 3.0, 2.0},
                                             &quantiles));
  EXPECT_TRUE(quantiles.valid);
  EXPECT_DOUBLE_EQ(1.0, quantiles.minimum);
  EXPECT_DOUBLE_EQ(1.3, quantiles.p10);
  EXPECT_DOUBLE_EQ(2.5, quantiles.median);
  EXPECT_DOUBLE_EQ(3.7, quantiles.p90);
  EXPECT_DOUBLE_EQ(3.85, quantiles.p95);
  EXPECT_DOUBLE_EQ(4.0, quantiles.maximum);
  EXPECT_FALSE(computeDeterministicQuantiles(
      {1.0, std::numeric_limits<double>::infinity()}, &quantiles));
}

TEST(TriangulationCandidateEvaluatorTest,
     InvalidOptionsFailWithoutPartialEvaluation) {
  TriangulationCandidateOptions options;
  options.minimum_depth = 2.0;
  options.maximum_depth = 1.0;
  const CameraRig rig = frontend_test::makeRig();
  CameraDescriptorSet first = makeSet(0U, 10U, Eigen::Vector3d::UnitZ());
  CameraDescriptorSet second = makeSet(1U, 20U, Eigen::Vector3d::UnitZ());
  TriangulationDiagnostic diagnostic;
  EXPECT_FALSE(TriangulationCandidateEvaluator(options).evaluateMatch(
      makeMatch(), first, second, rig, &diagnostic));
}

std::vector<std::uint8_t> filled(std::uint8_t value) {
  return std::vector<std::uint8_t>(32U, value);
}

void appendDescriptor(CameraDescriptorSet* set, FeatureId feature_id,
                      const Eigen::Vector3d& bearing,
                      const std::vector<std::uint8_t>& descriptor) {
  set->feature_ids.push_back(feature_id);
  set->pixels.emplace_back(100.0 + set->feature_ids.size(), 80.0);
  set->bearings_c.push_back(bearing);
  set->bearings_b.push_back(bearing);
  cv::Mat row(1, 32, CV_8UC1,
              const_cast<std::uint8_t*>(descriptor.data()));
  set->descriptors.push_back(row.clone());
}

TEST(TriangulationCandidateEvaluatorTest,
     MatcherOutputFeedsEvaluatorWithoutTrackMutation) {
  const CameraRig rig = frontend_test::makeRig();
  const RigCamera* camera_0 = rig.camera(0U);
  const RigCamera* camera_1 = rig.camera(1U);
  ASSERT_NE(nullptr, camera_0);
  ASSERT_NE(nullptr, camera_1);
  CameraDescriptorSet first;
  first.camera_id = 0U;
  first.timestamp = 5.0;
  CameraDescriptorSet second;
  second.camera_id = 1U;
  second.timestamp = 5.0;
  const std::array<Eigen::Vector3d, 2> points{{
      Eigen::Vector3d(0.0, 0.0, 3.0),
      Eigen::Vector3d(0.2, 0.1, 2.0)}};
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const Eigen::Vector3d bearing_0 = camera_0->R_b_c.transpose() *
        (points[index] - camera_0->t_b_c).normalized();
    const Eigen::Vector3d bearing_1 = camera_1->R_b_c.transpose() *
        (points[index] - camera_1->t_b_c).normalized();
    appendDescriptor(&first, 10U + index, bearing_0,
                     filled(index == 0U ? 0x00U : 0xFFU));
    appendDescriptor(&second, 20U + index, bearing_1,
                     filled(index == 0U ? 0x00U : 0xFFU));
  }
  CrossCameraMatcherOptions matcher_options;
  matcher_options.camera_pairs = {{0U, 1U}};
  matcher_options.maximum_descriptor_distance = 64.0;
  matcher_options.maximum_epipolar_angle = 0.003;
  CrossCameraPairResult matching;
  ASSERT_TRUE(CrossCameraMatcher(matcher_options).matchPair(
      first, second, rig, &matching));
  ASSERT_EQ(2U, matching.matches.size());
  TriangulationCandidatePairResult evaluated;
  ASSERT_TRUE(TriangulationCandidateEvaluator().evaluatePair(
      matching, first, second, rig, &evaluated));
  EXPECT_EQ(2U, evaluated.input_matches);
  EXPECT_EQ(2U, evaluated.triangulation_successes);
  EXPECT_EQ(2U, evaluated.candidates.size());
  EXPECT_EQ(10U, evaluated.candidates[0].match.feature_id_1);
  EXPECT_EQ(20U, evaluated.candidates[0].match.feature_id_2);
}

}  // namespace
}  // namespace sphere_vio
