#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/geometry/epipolar_geometry.hpp"
#include "sphere_vio/geometry/triangulation.hpp"

namespace sphere_vio {
namespace {

std::string realCameraConfigPath() {
  return std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml";
}

Eigen::Matrix3d rotation(double angle, const Eigen::Vector3d& axis) {
  return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

TriangulationOptions permissiveOptions() {
  TriangulationOptions options;
  options.minimum_ray_angle = 1e-9;
  options.minimum_depth = 0.0;
  return options;
}

void expectSuccess(const TriangulationResult& result) {
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(TriangulationStatus::kSuccess, result.status);
}

TEST(TriangulationTest, ExactIntersectingRaysRecoverPointAndDepths) {
  const Eigen::Vector3d origin_1(0.0, 0.0, 0.0);
  const Eigen::Vector3d origin_2(0.7, -0.2, 0.1);
  const Eigen::Vector3d expected_point(0.3, 0.5, 2.4);
  const Eigen::Vector3d ray_1 = expected_point - origin_1;
  const Eigen::Vector3d ray_2 = expected_point - origin_2;
  TriangulationResult result;
  ASSERT_TRUE(triangulateRays(origin_1, ray_1, origin_2, ray_2,
                              permissiveOptions(), &result));
  expectSuccess(result);
  EXPECT_EQ(TriangulationCoordinateFrame::kCallerCommon,
            result.coordinate_frame);
  EXPECT_LT((result.point_common - expected_point).norm(), 2e-15);
  EXPECT_NEAR(ray_1.norm(), result.depth_1, 2e-15);
  EXPECT_NEAR(ray_2.norm(), result.depth_2, 2e-15);
  EXPECT_LT(result.closest_ray_distance, 2e-15);
  EXPECT_LT(result.maximum_angular_reprojection_error, 2e-8);
  std::cout << "Exact ray triangulation: position_error="
            << (result.point_common - expected_point).norm()
            << " m, depth_1_error="
            << std::abs(result.depth_1 - ray_1.norm())
            << " m, depth_2_error="
            << std::abs(result.depth_2 - ray_2.norm())
            << " m, closest=" << result.closest_ray_distance
            << " m, max_angular="
            << result.maximum_angular_reprojection_error << " rad"
            << std::endl;
}

TEST(TriangulationTest, NonUnitSwapAndRigidFrameTransformAreConsistent) {
  const Eigen::Vector3d origin_1(-0.3, 0.2, 0.1);
  const Eigen::Vector3d origin_2(0.5, -0.1, 0.4);
  const Eigen::Vector3d point(0.2, 0.7, 3.1);
  TriangulationResult original;
  TriangulationResult swapped;
  ASSERT_TRUE(triangulateRays(
      origin_1, 7.0 * (point - origin_1), origin_2,
      0.3 * (point - origin_2), permissiveOptions(), &original));
  ASSERT_TRUE(triangulateRays(
      origin_2, point - origin_2, origin_1, point - origin_1,
      permissiveOptions(), &swapped));
  EXPECT_LT((original.point_common - point).norm(), 5e-15);
  EXPECT_LT((swapped.point_common - point).norm(), 5e-15);
  EXPECT_LT((original.point_common - swapped.point_common).norm(), 5e-15);
  EXPECT_NEAR(original.depth_1, swapped.depth_2, 5e-15);
  EXPECT_NEAR(original.depth_2, swapped.depth_1, 5e-15);

  const Eigen::Matrix3d R_new_old =
      rotation(0.53, Eigen::Vector3d(0.3, -0.7, 0.4));
  const Eigen::Vector3d t_new_old(1.2, -0.8, 0.4);
  const auto transform_point = [&](const Eigen::Vector3d& value) {
    return R_new_old * value + t_new_old;
  };
  TriangulationResult transformed;
  ASSERT_TRUE(triangulateRays(
      transform_point(origin_1), R_new_old * (point - origin_1),
      transform_point(origin_2), R_new_old * (point - origin_2),
      permissiveOptions(), &transformed));
  EXPECT_LT((transformed.point_common - transform_point(point)).norm(),
            5e-15);
  EXPECT_NEAR(original.depth_1, transformed.depth_1, 5e-15);
  EXPECT_NEAR(original.depth_2, transformed.depth_2, 5e-15);
}

TEST(TriangulationTest, RelativePoseWrapperMatchesTargetFrameRays) {
  const Eigen::Matrix3d R_2_1 =
      rotation(-0.31, Eigen::Vector3d(0.4, 0.2, 1.0));
  const Eigen::Vector3d t_2_1(0.24, -0.08, 0.05);
  const Eigen::Vector3d point_1(0.3, -0.2, 3.4);
  const Eigen::Vector3d point_2 = R_2_1 * point_1 + t_2_1;
  const Eigen::Vector3d bearing_1 = point_1.normalized();
  const Eigen::Vector3d bearing_2 = point_2.normalized();
  TriangulationResult wrapped;
  TriangulationResult low_level;
  ASSERT_TRUE(triangulateBearings(bearing_1, bearing_2, R_2_1, t_2_1,
                                  permissiveOptions(), &wrapped));
  ASSERT_TRUE(triangulateRays(t_2_1, R_2_1 * bearing_1,
                              Eigen::Vector3d::Zero(), bearing_2,
                              permissiveOptions(), &low_level));
  EXPECT_EQ(TriangulationCoordinateFrame::kTargetCamera,
            wrapped.coordinate_frame);
  EXPECT_LT((wrapped.point_common - point_2).norm(), 2e-14);
  EXPECT_LT((wrapped.point_common - low_level.point_common).norm(), 1e-15);
  EXPECT_NEAR(point_1.norm(), wrapped.depth_1, 2e-14);
  EXPECT_NEAR(point_2.norm(), wrapped.depth_2, 2e-14);
}

TEST(TriangulationTest, BodyCameraWrapperMatchesCommonFrameRays) {
  RigCamera camera_1;
  camera_1.R_b_c =
      rotation(0.24, Eigen::Vector3d(0.2, 0.5, -0.1));
  camera_1.t_b_c = Eigen::Vector3d(-0.12, 0.03, 0.08);
  RigCamera camera_2;
  camera_2.R_b_c =
      rotation(-0.19, Eigen::Vector3d(0.7, -0.1, 0.3));
  camera_2.t_b_c = Eigen::Vector3d(0.18, -0.04, 0.02);
  const Eigen::Vector3d point_b(0.4, 0.3, 2.8);
  const Eigen::Vector3d bearing_c1 =
      camera_1.R_b_c.transpose() * (point_b - camera_1.t_b_c);
  const Eigen::Vector3d bearing_c2 =
      camera_2.R_b_c.transpose() * (point_b - camera_2.t_b_c);
  TriangulationResult wrapped;
  TriangulationResult low_level;
  ASSERT_TRUE(triangulateBodyBearings(camera_1, bearing_c1, camera_2,
                                      bearing_c2, permissiveOptions(),
                                      &wrapped));
  ASSERT_TRUE(triangulateRays(
      camera_1.t_b_c, camera_1.R_b_c * bearing_c1, camera_2.t_b_c,
      camera_2.R_b_c * bearing_c2, permissiveOptions(), &low_level));
  EXPECT_EQ(TriangulationCoordinateFrame::kBody, wrapped.coordinate_frame);
  EXPECT_LT((wrapped.point_common - point_b).norm(), 1e-14);
  EXPECT_LT((wrapped.point_common - low_level.point_common).norm(), 1e-15);
}

TEST(TriangulationTest, InvalidInputsAndOptionsHaveExplicitStatus) {
  const Eigen::Vector3d origin_1 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d origin_2(1.0, 0.0, 0.0);
  const Eigen::Vector3d direction_1(0.0, 0.0, 1.0);
  const Eigen::Vector3d direction_2(-0.3, 0.0, 1.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  TriangulationResult result;
  EXPECT_FALSE(triangulateRays(origin_1, direction_1, origin_2, direction_2,
                               permissiveOptions(), nullptr));
  EXPECT_FALSE(triangulateRays(origin_1, Eigen::Vector3d::Zero(), origin_2,
                               direction_2, permissiveOptions(), &result));
  EXPECT_EQ(TriangulationStatus::kInvalidInput, result.status);
  EXPECT_FALSE(triangulateRays(Eigen::Vector3d(nan, 0.0, 0.0), direction_1,
                               origin_2, direction_2, permissiveOptions(),
                               &result));
  EXPECT_EQ(TriangulationStatus::kInvalidInput, result.status);
  TriangulationOptions invalid_options = permissiveOptions();
  invalid_options.minimum_ray_angle = -1.0;
  EXPECT_FALSE(triangulateRays(origin_1, direction_1, origin_2, direction_2,
                               invalid_options, &result));
  EXPECT_EQ(TriangulationStatus::kInvalidInput, result.status);
  invalid_options = permissiveOptions();
  invalid_options.maximum_closest_ray_distance = nan;
  EXPECT_FALSE(triangulateRays(origin_1, direction_1, origin_2, direction_2,
                               invalid_options, &result));
  EXPECT_EQ(TriangulationStatus::kInvalidInput, result.status);

  Eigen::Matrix3d invalid_rotation = Eigen::Matrix3d::Identity();
  invalid_rotation(0, 0) = 2.0;
  EXPECT_FALSE(triangulateBearings(direction_1, direction_2,
                                   invalid_rotation, origin_2,
                                   permissiveOptions(), &result));
  EXPECT_EQ(TriangulationStatus::kInvalidInput, result.status);
  EXPECT_EQ(TriangulationCoordinateFrame::kTargetCamera,
            result.coordinate_frame);
}

TEST(TriangulationTest, BaselineParallelAndNegativeDepthAreRejected) {
  TriangulationResult result;
  TriangulationOptions options = permissiveOptions();
  options.minimum_baseline = 1e-3;
  EXPECT_FALSE(triangulateRays(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.0, 1.0), options,
      &result));
  EXPECT_EQ(TriangulationStatus::kZeroBaseline, result.status);
  EXPECT_FALSE(triangulateRays(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d(1e-4, 0.0, 0.0), Eigen::Vector3d(0.1, 0.0, 1.0),
      options, &result));
  EXPECT_EQ(TriangulationStatus::kZeroBaseline, result.status);

  options.minimum_baseline = 1e-9;
  options.minimum_ray_angle = 1e-3;
  EXPECT_FALSE(triangulateRays(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::UnitZ(), options,
      &result));
  EXPECT_EQ(TriangulationStatus::kNearlyParallelRays, result.status);
  EXPECT_FALSE(triangulateRays(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(5e-4, 0.0, 1.0), options, &result));
  EXPECT_EQ(TriangulationStatus::kNearlyParallelRays, result.status);
  EXPECT_FALSE(triangulateRays(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(),
      Eigen::Vector3d(1.0, 0.0, 0.0), -Eigen::Vector3d::UnitX(), options,
      &result));
  EXPECT_EQ(TriangulationStatus::kNearlyParallelRays, result.status);

  const Eigen::Vector3d origin_1 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d origin_2(1.0, 0.0, 0.0);
  const Eigen::Vector3d point(0.4, 0.6, 2.0);
  EXPECT_FALSE(triangulateRays(origin_1, origin_1 - point, origin_2,
                               origin_2 - point, permissiveOptions(),
                               &result));
  EXPECT_EQ(TriangulationStatus::kNegativeDepth, result.status);
  EXPECT_LT(result.depth_1, 0.0);
  EXPECT_LT(result.depth_2, 0.0);
  EXPECT_FALSE(triangulateRays(origin_1, point - origin_1, origin_2,
                               origin_2 - point, permissiveOptions(),
                               &result));
  EXPECT_EQ(TriangulationStatus::kNegativeDepth, result.status);
  EXPECT_GT(result.depth_1, 0.0);
  EXPECT_LT(result.depth_2, 0.0);
  EXPECT_FALSE(triangulateRays(origin_2, origin_2 - point, origin_1,
                               point - origin_1, permissiveOptions(),
                               &result));
  EXPECT_EQ(TriangulationStatus::kNegativeDepth, result.status);
  EXPECT_LT(result.depth_1, 0.0);
  EXPECT_GT(result.depth_2, 0.0);

  const double huge = 1e308;
  EXPECT_FALSE(triangulateRays(
      Eigen::Vector3d(huge, 0.0, 0.0), Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d(-huge, 0.0, 0.0), Eigen::Vector3d(0.1, 0.0, 1.0),
      permissiveOptions(), &result));
  EXPECT_EQ(TriangulationStatus::kNumericalFailure, result.status);
}

TEST(TriangulationTest, DistanceAndAngularThresholdsRejectNoisyRays) {
  const Eigen::Vector3d origin_1 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d origin_2(0.5, 0.0, 0.0);
  const Eigen::Vector3d direction_1(0.1, 0.0, 1.0);
  const Eigen::Vector3d direction_2(-0.15, 0.02, 1.0);
  TriangulationOptions options = permissiveOptions();
  options.maximum_closest_ray_distance = 1e-5;
  TriangulationResult result;
  EXPECT_FALSE(triangulateRays(origin_1, direction_1, origin_2, direction_2,
                               options, &result));
  EXPECT_EQ(TriangulationStatus::kClosestDistanceTooLarge, result.status);
  EXPECT_GT(result.closest_ray_distance,
            options.maximum_closest_ray_distance);

  options.maximum_closest_ray_distance =
      std::numeric_limits<double>::infinity();
  options.maximum_angular_reprojection_error = 1e-7;
  EXPECT_FALSE(triangulateRays(origin_1, direction_1, origin_2, direction_2,
                               options, &result));
  EXPECT_EQ(TriangulationStatus::kReprojectionErrorTooLarge, result.status);
  EXPECT_GT(result.maximum_angular_reprojection_error,
            options.maximum_angular_reprojection_error);
}

TEST(TriangulationTest, SphericalNoiseShowsIncreasingError) {
  const Eigen::Vector3d origin_1 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d origin_2(0.3, 0.0, 0.0);
  const Eigen::Vector3d true_point(0.08, 0.04, 2.0);
  const Eigen::Vector3d direction_1 = (true_point - origin_1).normalized();
  const Eigen::Vector3d direction_2 = (true_point - origin_2).normalized();
  const Eigen::Vector3d plane_normal =
      (origin_2 - origin_1).cross(direction_1).normalized();
  const std::array<double, 5> noises{{1e-6, 1e-5, 1e-4, 1e-3, 1e-2}};
  double first_position_error = 0.0;
  double last_position_error = 0.0;
  double previous_combined_metric = -1.0;
  std::cout << "Triangulation angular-noise trend:";
  for (std::size_t index = 0; index < noises.size(); ++index) {
    const double noise = noises[index];
    const Eigen::Vector3d perturbed_direction_2 =
        (std::cos(noise) * direction_2 +
         std::sin(noise) * plane_normal)
            .normalized();
    TriangulationResult result;
    ASSERT_TRUE(triangulateRays(origin_1, direction_1, origin_2,
                                perturbed_direction_2, permissiveOptions(),
                                &result));
    const double position_error =
        (result.point_common - true_point).norm();
    const double depth_error =
        std::abs(result.depth_1 - (true_point - origin_1).norm());
    const double combined_metric = position_error + depth_error +
                                   result.closest_ray_distance +
                                   result.maximum_angular_reprojection_error;
    if (index == 0U) first_position_error = position_error;
    last_position_error = position_error;
    EXPECT_GT(combined_metric, previous_combined_metric);
    previous_combined_metric = combined_metric;
    std::cout << " " << noise << "->(position=" << position_error
              << ",depth=" << depth_error
              << ",closest=" << result.closest_ray_distance
              << ",angular="
              << result.maximum_angular_reprojection_error << ")";
  }
  std::cout << std::endl;
  EXPECT_GT(last_position_error, 1000.0 * first_position_error);
}

TEST(TriangulationTest, DepthParallaxAndFixedNoiseShowExpectedTrend) {
  const Eigen::Vector3d origin_1 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d origin_2(0.1, 0.0, 0.0);
  const std::array<double, 7> depths{{0.5, 1.0, 2.0, 5.0, 10.0, 20.0,
                                      50.0}};
  const double angular_noise = 1e-4;
  double previous_angle = std::numeric_limits<double>::infinity();
  double first_noisy_depth_error = 0.0;
  double last_noisy_depth_error = 0.0;
  std::cout << "Depth/parallax trend:";
  for (std::size_t index = 0; index < depths.size(); ++index) {
    const Eigen::Vector3d point(0.02, 0.01, depths[index]);
    const Eigen::Vector3d direction_1 = (point - origin_1).normalized();
    const Eigen::Vector3d direction_2 = (point - origin_2).normalized();
    TriangulationResult exact;
    ASSERT_TRUE(triangulateRays(origin_1, direction_1, origin_2, direction_2,
                                permissiveOptions(), &exact));
    EXPECT_LT(exact.ray_angle, previous_angle);
    previous_angle = exact.ray_angle;

    const Eigen::Vector3d noise_axis =
        direction_2.cross(Eigen::Vector3d::UnitY()).norm() > 1e-6
            ? Eigen::Vector3d::UnitY()
            : Eigen::Vector3d::UnitX();
    const Eigen::Vector3d noisy_direction_2 =
        Eigen::AngleAxisd(angular_noise, noise_axis) * direction_2;
    TriangulationResult noisy;
    ASSERT_TRUE(triangulateRays(origin_1, direction_1, origin_2,
                                noisy_direction_2, permissiveOptions(),
                                &noisy));
    const double noisy_depth_error =
        std::abs(noisy.depth_1 - (point - origin_1).norm());
    if (index == 0U) first_noisy_depth_error = noisy_depth_error;
    last_noisy_depth_error = noisy_depth_error;
    std::cout << " " << depths[index] << "m->(angle=" << exact.ray_angle
              << ",fixed-noise-depth-error=" << noisy_depth_error << ")";
  }
  std::cout << std::endl;
  EXPECT_GT(last_noisy_depth_error, 1000.0 * first_noisy_depth_error);

  const Eigen::Vector3d far_point(0.0, 0.0, 1000.0);
  TriangulationOptions rejecting_options = permissiveOptions();
  rejecting_options.minimum_ray_angle = 1e-3;
  TriangulationResult rejected;
  EXPECT_FALSE(triangulateRays(
      origin_1, far_point - origin_1, origin_2, far_point - origin_2,
      rejecting_options, &rejected));
  EXPECT_EQ(TriangulationStatus::kNearlyParallelRays, rejected.status);
}

TEST(TriangulationTest, AllRealRigPairsRecoverSyntheticBodyPoints) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &error))
      << error;
  const int expected_kalibr_ids[4] = {0, 1, 3, 2};
  const std::array<double, 5> depths{{0.5, 1.0, 2.0, 5.0, 10.0}};
  double global_maximum_position_error = 0.0;
  double global_maximum_relative_depth_error = 0.0;
  double global_maximum_closest_distance = 0.0;
  double global_maximum_angular_error = 0.0;
  std::size_t global_successes = 0U;

  for (CameraId source_id = 0U; source_id < 4U; ++source_id) {
    const RigCamera* source = rig.camera(source_id);
    ASSERT_NE(nullptr, source);
    ASSERT_EQ(expected_kalibr_ids[source_id], source->kalibr_camera_id);
    for (CameraId target_id = 0U; target_id < 4U; ++target_id) {
      if (source_id == target_id) continue;
      const RigCamera* target = rig.camera(target_id);
      ASSERT_NE(nullptr, target);
      std::size_t attempted = 0U;
      std::size_t source_invalid = 0U;
      std::size_t target_not_visible = 0U;
      std::size_t geometric_degeneracy = 0U;
      std::size_t triangulation_rejection = 0U;
      std::size_t successes = 0U;
      double position_error_sum = 0.0;
      double maximum_position_error = 0.0;
      double maximum_relative_depth_error = 0.0;
      double maximum_closest_distance = 0.0;
      double minimum_ray_angle = std::numeric_limits<double>::infinity();
      double maximum_angular_error = 0.0;

      for (int v = 0; v < source->model->height(); v += 110) {
        for (int u = 0; u < source->model->width(); u += 136) {
          Eigen::Vector3d bearing_c1;
          if (!source->model->unproject(Eigen::Vector2d(u, v),
                                        &bearing_c1)) {
            attempted += depths.size();
            source_invalid += depths.size();
            continue;
          }
          for (const double depth_1_true : depths) {
            ++attempted;
            const Eigen::Vector3d point_c1 = depth_1_true * bearing_c1;
            const Eigen::Vector3d point_b =
                source->R_b_c * point_c1 + source->t_b_c;
            const Eigen::Vector3d point_c2 =
                target->R_b_c.transpose() * (point_b - target->t_b_c);
            Eigen::Vector2d pixel_2;
            if (!target->model->project(point_c2, &pixel_2)) {
              ++target_not_visible;
              continue;
            }
            Eigen::Vector3d bearing_c2;
            ASSERT_TRUE(target->model->unproject(pixel_2, &bearing_c2));
            TriangulationResult result;
            if (!triangulateBodyBearings(*source, bearing_c1, *target,
                                         bearing_c2, permissiveOptions(),
                                         &result)) {
              if (result.status == TriangulationStatus::kZeroBaseline ||
                  result.status ==
                      TriangulationStatus::kNearlyParallelRays) {
                ++geometric_degeneracy;
              } else {
                ++triangulation_rejection;
              }
              continue;
            }
            ++successes;
            const double position_error =
                (result.point_common - point_b).norm();
            const double target_depth_true = point_c2.norm();
            const double relative_depth_error = std::max(
                std::abs(result.depth_1 - depth_1_true) / depth_1_true,
                std::abs(result.depth_2 - target_depth_true) /
                    target_depth_true);
            position_error_sum += position_error;
            maximum_position_error =
                std::max(maximum_position_error, position_error);
            maximum_relative_depth_error = std::max(
                maximum_relative_depth_error, relative_depth_error);
            maximum_closest_distance = std::max(
                maximum_closest_distance, result.closest_ray_distance);
            minimum_ray_angle =
                std::min(minimum_ray_angle, result.ray_angle);
            maximum_angular_error = std::max(
                maximum_angular_error,
                result.maximum_angular_reprojection_error);
          }
        }
      }
      ASSERT_GT(successes, 20U) << "C" << source_id << " -> C"
                                << target_id;
      const double average_position_error = position_error_sum / successes;
      EXPECT_EQ(attempted, source_invalid + target_not_visible +
                               geometric_degeneracy +
                               triangulation_rejection + successes);
      EXPECT_EQ(0U, geometric_degeneracy);
      EXPECT_EQ(0U, triangulation_rejection);
      EXPECT_LT(maximum_position_error, 5e-9);
      EXPECT_LT(maximum_relative_depth_error, 1e-9);
      global_successes += successes;
      global_maximum_position_error =
          std::max(global_maximum_position_error, maximum_position_error);
      global_maximum_relative_depth_error = std::max(
          global_maximum_relative_depth_error, maximum_relative_depth_error);
      global_maximum_closest_distance = std::max(
          global_maximum_closest_distance, maximum_closest_distance);
      global_maximum_angular_error =
          std::max(global_maximum_angular_error, maximum_angular_error);
      std::cout << "Real triangulation C" << source_id << " -> C"
                << target_id << ": attempted=" << attempted
                << ", source_invalid=" << source_invalid
                << ", target_not_visible=" << target_not_visible
                << ", degeneracy=" << geometric_degeneracy
                << ", rejected=" << triangulation_rejection
                << ", success=" << successes
                << ", max_position=" << maximum_position_error
                << " m, avg_position=" << average_position_error
                << " m, max_relative_depth="
                << maximum_relative_depth_error
                << ", max_closest=" << maximum_closest_distance
                << " m, min_ray_angle=" << minimum_ray_angle
                << " rad, max_angular=" << maximum_angular_error << " rad"
                << std::endl;
    }
  }
  std::cout << "Real triangulation global: success=" << global_successes
            << ", max_position=" << global_maximum_position_error
            << " m, max_relative_depth="
            << global_maximum_relative_depth_error
            << ", max_closest=" << global_maximum_closest_distance
            << " m, max_angular=" << global_maximum_angular_error << " rad"
            << std::endl;
}

}  // namespace
}  // namespace sphere_vio
