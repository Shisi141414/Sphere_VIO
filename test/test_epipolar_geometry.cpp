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
#include <yaml-cpp/yaml.h>

#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/geometry/epipolar_geometry.hpp"

namespace sphere_vio {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::string realCameraConfigPath() {
  return std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml";
}

Eigen::Matrix3d rotation(double angle, const Eigen::Vector3d& axis) {
  return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

double robustAngle(const Eigen::Vector3d& first,
                   const Eigen::Vector3d& second) {
  const Eigen::Vector3d normalized_first = first.normalized();
  const Eigen::Vector3d normalized_second = second.normalized();
  return std::atan2(normalized_first.cross(normalized_second).norm(),
                    std::max(-1.0, std::min(
                                           1.0, normalized_first.dot(
                                                    normalized_second))));
}

Eigen::Matrix4d parseTransform(const YAML::Node& node) {
  Eigen::Matrix4d transform;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      transform(row, column) =
          node["data"][static_cast<std::size_t>(row * 4 + column)]
              .as<double>();
    }
  }
  return transform;
}

std::array<Eigen::Matrix4d, 4> readTCamImuByProjectCamera() {
  std::array<Eigen::Matrix4d, 4> transforms;
  const YAML::Node cameras = YAML::LoadFile(realCameraConfigPath())["cameras"];
  for (const YAML::Node& camera : cameras) {
    transforms[camera["camera_id"].as<CameraId>()] =
        parseTransform(camera["T_cam_imu"]);
  }
  return transforms;
}

Eigen::Vector3d correspondingTargetBearing(
    const Eigen::Vector3d& bearing_source, double depth,
    const RelativePose& relative_pose) {
  return (relative_pose.R_target_source *
              (depth * bearing_source.normalized()) +
          relative_pose.t_target_source)
      .normalized();
}

TEST(EpipolarGeometryTest, RelativePoseFollowsTargetSourceConvention) {
  RigCamera source;
  source.R_b_c = rotation(0.31, Eigen::Vector3d(0.2, -0.4, 1.0));
  source.t_b_c = Eigen::Vector3d(0.3, -0.1, 0.2);
  RigCamera target;
  target.R_b_c = rotation(-0.27, Eigen::Vector3d(1.0, 0.3, -0.2));
  target.t_b_c = Eigen::Vector3d(-0.2, 0.4, 0.1);

  RelativePose relative_pose;
  ASSERT_TRUE(relativeCameraPose(source, target, &relative_pose));
  const Eigen::Matrix3d expected_rotation =
      target.R_b_c.transpose() * source.R_b_c;
  const Eigen::Vector3d expected_translation =
      target.R_b_c.transpose() * (source.t_b_c - target.t_b_c);
  EXPECT_TRUE(relative_pose.R_target_source.isApprox(expected_rotation,
                                                     1e-15));
  EXPECT_TRUE(relative_pose.t_target_source.isApprox(expected_translation,
                                                     1e-15));

  const Eigen::Vector3d point_source(0.4, -0.7, 2.3);
  const Eigen::Vector3d point_body =
      source.R_b_c * point_source + source.t_b_c;
  const Eigen::Vector3d point_target =
      target.R_b_c.transpose() * (point_body - target.t_b_c);
  const Eigen::Vector3d relative_point_target =
      relative_pose.R_target_source * point_source +
      relative_pose.t_target_source;
  EXPECT_LT((relative_point_target - point_target).norm(), 1e-15);
}

TEST(EpipolarGeometryTest, SyntheticCorrespondencesReachNumericalPrecision) {
  const std::array<RelativePose, 3> poses{{
      {Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.23, 0.0, 0.0)},
      {rotation(0.13, Eigen::Vector3d::UnitY()),
       Eigen::Vector3d(0.17, -0.04, 0.02)},
      {rotation(-0.29, Eigen::Vector3d(0.3, 0.8, -0.4)),
       Eigen::Vector3d(-0.12, 0.19, 0.08)}}};
  const std::array<double, 5> depths{{0.5, 1.0, 2.0, 5.0, 10.0}};
  std::size_t valid_samples = 0U;
  std::size_t degeneracies = 0U;
  double maximum_algebraic = 0.0;
  double maximum_forward = 0.0;
  double maximum_backward = 0.0;
  double maximum_symmetric = 0.0;

  for (const RelativePose& pose : poses) {
    for (int y = -4; y <= 4; ++y) {
      for (int x = -5; x <= 5; ++x) {
        const Eigen::Vector3d bearing_source =
            Eigen::Vector3d(0.09 * x, 0.08 * y, 1.0).normalized();
        for (const double depth : depths) {
          const Eigen::Vector3d point_target =
              pose.R_target_source * (depth * bearing_source) +
              pose.t_target_source;
          if (point_target.z() <= 0.0 || point_target.norm() <= 1e-12) {
            continue;
          }
          const Eigen::Vector3d bearing_target = point_target.normalized();
          double algebraic = 0.0;
          EpipolarError error;
          if (!epipolarAlgebraicResidual(
                  bearing_source, bearing_target, pose.R_target_source,
                  pose.t_target_source, &algebraic) ||
              !symmetricEpipolarAngularError(
                  bearing_source, bearing_target, pose.R_target_source,
                  pose.t_target_source, &error)) {
            ++degeneracies;
            continue;
          }
          ++valid_samples;
          maximum_algebraic = std::max(maximum_algebraic,
                                       std::abs(algebraic));
          maximum_forward = std::max(maximum_forward, error.forward);
          maximum_backward = std::max(maximum_backward, error.backward);
          maximum_symmetric = std::max(maximum_symmetric, error.maximum);
        }
      }
    }
  }
  std::cout << "Synthetic epipolar correspondences: valid=" << valid_samples
            << ", degeneracies=" << degeneracies
            << ", max_algebraic=" << maximum_algebraic
            << ", max_forward=" << maximum_forward
            << " rad, max_backward=" << maximum_backward
            << " rad, max_symmetric=" << maximum_symmetric << " rad"
            << std::endl;
  EXPECT_GT(valid_samples, 1400U);
  EXPECT_LT(maximum_algebraic, 1e-15);
  EXPECT_LT(maximum_forward, 2e-15);
  EXPECT_LT(maximum_backward, 2e-15);
  EXPECT_LT(maximum_symmetric, 2e-15);
}

TEST(EpipolarGeometryTest, AngularErrorIsIndependentOfBaselineScale) {
  const Eigen::Matrix3d R_target_source =
      rotation(0.17, Eigen::Vector3d(0.2, 1.0, 0.1));
  const Eigen::Vector3d bearing_source =
      Eigen::Vector3d(0.2, -0.1, 1.0).normalized();
  const Eigen::Vector3d translation(0.3, 0.08, -0.04);
  const Eigen::Vector3d exact_target = correspondingTargetBearing(
      bearing_source, 3.0,
      {R_target_source, translation});
  Eigen::Vector3d normal;
  ASSERT_TRUE(epipolarPlaneNormal(bearing_source, R_target_source,
                                  translation, &normal));
  const double perturbation = 1e-3;
  const Eigen::Vector3d perturbed_target =
      (std::cos(perturbation) * exact_target +
       std::sin(perturbation) * normal)
          .normalized();

  double reference_error = 0.0;
  double scaled_error = 0.0;
  double reference_residual = 0.0;
  double scaled_residual = 0.0;
  ASSERT_TRUE(epipolarAngularError(
      bearing_source, perturbed_target, R_target_source, translation,
      &reference_error));
  ASSERT_TRUE(epipolarAngularError(
      bearing_source, perturbed_target, R_target_source, 7.0 * translation,
      &scaled_error));
  ASSERT_TRUE(epipolarAlgebraicResidual(
      bearing_source, perturbed_target, R_target_source, translation,
      &reference_residual));
  ASSERT_TRUE(epipolarAlgebraicResidual(
      bearing_source, perturbed_target, R_target_source, 7.0 * translation,
      &scaled_residual));
  EXPECT_NEAR(reference_error, scaled_error, 1e-15);
  EXPECT_NEAR(7.0 * reference_residual, scaled_residual, 1e-15);
}

TEST(EpipolarGeometryTest, NormalAndTangentNoiseHaveExpectedTrend) {
  const RelativePose pose{
      rotation(0.11, Eigen::Vector3d(-0.2, 0.7, 0.3)),
      Eigen::Vector3d(0.19, -0.06, 0.04)};
  const Eigen::Vector3d bearing_source =
      Eigen::Vector3d(0.25, -0.18, 1.0).normalized();
  const Eigen::Vector3d bearing_target =
      correspondingTargetBearing(bearing_source, 4.0, pose);
  Eigen::Vector3d normal;
  ASSERT_TRUE(epipolarPlaneNormal(bearing_source, pose.R_target_source,
                                  pose.t_target_source, &normal));
  const Eigen::Vector3d tangent = normal.cross(bearing_target).normalized();
  const std::array<double, 5> perturbations{
      {1e-6, 1e-5, 1e-4, 1e-3, 1e-2}};
  double previous_normal_error = -1.0;

  std::cout << "Epipolar spherical noise trend:";
  for (const double perturbation : perturbations) {
    const Eigen::Vector3d normal_perturbed =
        std::cos(perturbation) * bearing_target +
        std::sin(perturbation) * normal;
    const Eigen::Vector3d tangent_perturbed =
        std::cos(perturbation) * bearing_target +
        std::sin(perturbation) * tangent;
    double normal_error = 0.0;
    double tangent_error = 0.0;
    ASSERT_TRUE(epipolarAngularError(
        bearing_source, normal_perturbed, pose.R_target_source,
        pose.t_target_source, &normal_error));
    ASSERT_TRUE(epipolarAngularError(
        bearing_source, tangent_perturbed, pose.R_target_source,
        pose.t_target_source, &tangent_error));
    std::cout << " " << perturbation << "->(" << normal_error << ","
              << tangent_error << ")";
    EXPECT_NEAR(perturbation, normal_error, 2e-15);
    EXPECT_LT(tangent_error, 2e-15);
    EXPECT_GT(normal_error, previous_normal_error);
    previous_normal_error = normal_error;
  }
  std::cout << " rad [input->(normal,tangent)]" << std::endl;
}

TEST(EpipolarGeometryTest, GreatCircleSamplingIsStableAndDoesNotRepeatEnd) {
  const std::array<Eigen::Vector3d, 5> normals{{
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d(1.0, 1e-14, 0.0),
      Eigen::Vector3d(0.2, -0.7, 1.4)}};
  for (const Eigen::Vector3d& input_normal : normals) {
    std::vector<Eigen::Vector3d> samples;
    ASSERT_TRUE(sampleGreatCircle(input_normal, 257U, &samples));
    ASSERT_EQ(257U, samples.size());
    const Eigen::Vector3d normal = input_normal.normalized();
    for (const Eigen::Vector3d& sample : samples) {
      EXPECT_NEAR(1.0, sample.norm(), 2e-15);
      EXPECT_LT(std::abs(sample.dot(normal)), 2e-15);
    }
    EXPECT_GT(robustAngle(samples.front(), samples.back()), 1e-3);
  }
}

TEST(EpipolarGeometryTest, DegenerateAndInvalidInputsFailSafely) {
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d valid_bearing(0.1, 0.2, 1.0);
  const Eigen::Vector3d valid_translation(0.2, 0.0, 0.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  double scalar = 0.0;
  Eigen::Vector3d vector;
  std::vector<Eigen::Vector3d> samples;

  EXPECT_FALSE(epipolarPlaneNormal(valid_bearing, identity,
                                   Eigen::Vector3d::Zero(), &vector));
  EXPECT_FALSE(epipolarPlaneNormal(valid_bearing, identity,
                                   Eigen::Vector3d(1e-12, 0.0, 0.0),
                                   &vector));
  EXPECT_FALSE(epipolarPlaneNormal(Eigen::Vector3d::Zero(), identity,
                                   valid_translation, &vector));
  EXPECT_FALSE(epipolarPlaneNormal(valid_translation, identity,
                                   valid_translation, &vector));
  EXPECT_FALSE(epipolarPlaneNormal(Eigen::Vector3d(nan, 0.0, 1.0), identity,
                                   valid_translation, &vector));
  Eigen::Matrix3d invalid_rotation = identity;
  invalid_rotation(0, 0) = 2.0;
  EXPECT_FALSE(epipolarPlaneNormal(valid_bearing, invalid_rotation,
                                   valid_translation, &vector));
  invalid_rotation = identity;
  invalid_rotation(0, 0) = nan;
  EXPECT_FALSE(epipolarPlaneNormal(valid_bearing, invalid_rotation,
                                   valid_translation, &vector));
  EXPECT_FALSE(epipolarPlaneNormal(valid_bearing, identity,
                                   valid_translation, nullptr));
  EXPECT_FALSE(epipolarAlgebraicResidual(
      valid_bearing, valid_bearing, identity, valid_translation, nullptr));
  EXPECT_FALSE(epipolarAngularError(valid_bearing, Eigen::Vector3d::Zero(),
                                    identity, valid_translation, &scalar));
  EXPECT_FALSE(symmetricEpipolarAngularError(
      valid_bearing, valid_bearing, identity, valid_translation, nullptr));
  EXPECT_FALSE(sampleGreatCircle(Eigen::Vector3d::Zero(), 16U, &samples));
  EXPECT_FALSE(sampleGreatCircle(Eigen::Vector3d::UnitZ(), 2U, &samples));
  EXPECT_FALSE(sampleGreatCircle(Eigen::Vector3d::UnitZ(), 16U, nullptr));

  RigCamera camera;
  camera.R_b_c = identity;
  camera.t_b_c = Eigen::Vector3d::Zero();
  RelativePose relative_pose;
  EXPECT_FALSE(relativeCameraPose(camera, camera, nullptr));
  EXPECT_FALSE(relativeCameraPose(camera, camera, &relative_pose));
  camera.R_b_c(0, 0) = 2.0;
  camera.t_b_c = valid_translation;
  EXPECT_FALSE(relativeCameraPose(camera, RigCamera(), &relative_pose));
}

TEST(EpipolarGeometryTest, AllDirectedRealRigRelativePosesAreConsistent) {
  CameraRig rig;
  std::string load_error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &load_error))
      << load_error;
  const std::array<Eigen::Matrix4d, 4> T_cam_imu =
      readTCamImuByProjectCamera();
  const int expected_kalibr_ids[4] = {0, 1, 3, 2};
  double maximum_inverse_rotation_error = 0.0;
  double maximum_inverse_translation_error = 0.0;
  double maximum_point_round_trip_error = 0.0;
  double maximum_raw_transform_error = 0.0;
  double maximum_baseline_error = 0.0;
  std::size_t pair_count = 0U;

  for (CameraId source_id = 0U; source_id < 4U; ++source_id) {
    const RigCamera* source = rig.camera(source_id);
    ASSERT_NE(nullptr, source);
    ASSERT_EQ(expected_kalibr_ids[source_id], source->kalibr_camera_id);
    for (CameraId target_id = 0U; target_id < 4U; ++target_id) {
      if (source_id == target_id) continue;
      const RigCamera* target = rig.camera(target_id);
      ASSERT_NE(nullptr, target);
      RelativePose forward;
      RelativePose backward;
      ASSERT_TRUE(relativeCameraPose(*source, *target, &forward));
      ASSERT_TRUE(relativeCameraPose(*target, *source, &backward));
      const double inverse_rotation_error =
          (backward.R_target_source - forward.R_target_source.transpose())
              .norm();
      const double inverse_translation_error =
          (backward.t_target_source +
           backward.R_target_source * forward.t_target_source)
              .norm();
      const Eigen::Vector3d point_source(0.21, -0.34, 2.7);
      const Eigen::Vector3d point_target =
          forward.R_target_source * point_source +
          forward.t_target_source;
      const Eigen::Vector3d recovered_source =
          backward.R_target_source * point_target +
          backward.t_target_source;
      const double point_round_trip_error =
          (recovered_source - point_source).norm();

      const Eigen::Matrix4d expected_relative =
          T_cam_imu[target_id] * T_cam_imu[source_id].inverse();
      Eigen::Matrix4d actual_relative = Eigen::Matrix4d::Identity();
      actual_relative.block<3, 3>(0, 0) = forward.R_target_source;
      actual_relative.block<3, 1>(0, 3) = forward.t_target_source;
      const double raw_transform_error =
          (actual_relative - expected_relative).norm();
      const double baseline_error =
          std::abs(forward.t_target_source.norm() -
                   (source->t_b_c - target->t_b_c).norm());

      maximum_inverse_rotation_error = std::max(
          maximum_inverse_rotation_error, inverse_rotation_error);
      maximum_inverse_translation_error = std::max(
          maximum_inverse_translation_error, inverse_translation_error);
      maximum_point_round_trip_error = std::max(
          maximum_point_round_trip_error, point_round_trip_error);
      maximum_raw_transform_error =
          std::max(maximum_raw_transform_error, raw_transform_error);
      maximum_baseline_error =
          std::max(maximum_baseline_error, baseline_error);
      ++pair_count;
      std::cout << "Real relative pose C" << source_id << " -> C"
                << target_id << ": baseline="
                << forward.t_target_source.norm()
                << " m, inverse_R_error=" << inverse_rotation_error
                << ", inverse_t_error=" << inverse_translation_error
                << ", point_round_trip=" << point_round_trip_error
                << ", raw_T_error=" << raw_transform_error << std::endl;
    }
  }
  std::cout << "Real relative-pose maxima: pairs=" << pair_count
            << ", inverse_R=" << maximum_inverse_rotation_error
            << ", inverse_t=" << maximum_inverse_translation_error
            << ", point_round_trip=" << maximum_point_round_trip_error
            << ", raw_T=" << maximum_raw_transform_error
            << ", baseline=" << maximum_baseline_error << std::endl;
  EXPECT_EQ(12U, pair_count);
  EXPECT_LT(maximum_inverse_rotation_error, 2e-15);
  EXPECT_LT(maximum_inverse_translation_error, 2e-15);
  EXPECT_LT(maximum_point_round_trip_error, 1e-13);
  EXPECT_LT(maximum_raw_transform_error, 1e-13);
  EXPECT_LT(maximum_baseline_error, 1e-15);
}

TEST(EpipolarGeometryTest, RealRigSyntheticVisibleCorrespondencesAreExact) {
  CameraRig rig;
  std::string load_error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &load_error))
      << load_error;
  const std::array<double, 5> depths{{0.5, 1.0, 2.0, 5.0, 10.0}};
  std::size_t total_visible = 0U;
  double global_maximum_algebraic = 0.0;
  double global_maximum_forward = 0.0;
  double global_maximum_backward = 0.0;
  double global_maximum_symmetric = 0.0;

  for (CameraId source_id = 0U; source_id < 4U; ++source_id) {
    const RigCamera* source = rig.camera(source_id);
    ASSERT_NE(nullptr, source);
    for (CameraId target_id = 0U; target_id < 4U; ++target_id) {
      if (source_id == target_id) continue;
      const RigCamera* target = rig.camera(target_id);
      ASSERT_NE(nullptr, target);
      RelativePose pose;
      ASSERT_TRUE(relativeCameraPose(*source, *target, &pose));
      std::size_t attempted = 0U;
      std::size_t both_visible = 0U;
      std::size_t degeneracies = 0U;
      double sum_error = 0.0;
      double maximum_algebraic = 0.0;
      double maximum_forward = 0.0;
      double maximum_backward = 0.0;
      double maximum_symmetric = 0.0;

      for (int v = 0; v < source->model->height(); v += 110) {
        for (int u = 0; u < source->model->width(); u += 136) {
          Eigen::Vector3d bearing_source;
          if (!source->model->unproject(Eigen::Vector2d(u, v),
                                        &bearing_source)) {
            attempted += depths.size();
            continue;
          }
          for (const double depth : depths) {
            ++attempted;
            const Eigen::Vector3d point_target =
                pose.R_target_source * (depth * bearing_source) +
                pose.t_target_source;
            Eigen::Vector2d target_pixel;
            if (!target->model->project(point_target, &target_pixel)) continue;
            Eigen::Vector3d bearing_target;
            ASSERT_TRUE(target->model->unproject(target_pixel,
                                                 &bearing_target));
            double algebraic = 0.0;
            EpipolarError error;
            if (!epipolarAlgebraicResidual(
                    bearing_source, bearing_target, pose.R_target_source,
                    pose.t_target_source, &algebraic) ||
                !symmetricEpipolarAngularError(
                    bearing_source, bearing_target, pose.R_target_source,
                    pose.t_target_source, &error)) {
              ++degeneracies;
              continue;
            }
            ++both_visible;
            sum_error += error.average;
            maximum_algebraic =
                std::max(maximum_algebraic, std::abs(algebraic));
            maximum_forward = std::max(maximum_forward, error.forward);
            maximum_backward = std::max(maximum_backward, error.backward);
            maximum_symmetric = std::max(maximum_symmetric, error.maximum);
          }
        }
      }
      ASSERT_GT(both_visible, 20U) << "C" << source_id << " -> C"
                                   << target_id;
      const double average_error = sum_error / both_visible;
      total_visible += both_visible;
      global_maximum_algebraic =
          std::max(global_maximum_algebraic, maximum_algebraic);
      global_maximum_forward =
          std::max(global_maximum_forward, maximum_forward);
      global_maximum_backward =
          std::max(global_maximum_backward, maximum_backward);
      global_maximum_symmetric =
          std::max(global_maximum_symmetric, maximum_symmetric);
      std::cout << "Real synthetic epipolar C" << source_id << " -> C"
                << target_id << ": attempted=" << attempted
                << ", both_visible=" << both_visible
                << ", degeneracies=" << degeneracies
                << ", max_algebraic=" << maximum_algebraic
                << ", max_forward=" << maximum_forward
                << ", max_backward=" << maximum_backward
                << ", max_symmetric=" << maximum_symmetric
                << ", average=" << average_error << " rad" << std::endl;
      EXPECT_LT(maximum_algebraic, 2e-13);
      EXPECT_LT(maximum_forward, 5e-12);
      EXPECT_LT(maximum_backward, 5e-12);
      EXPECT_LT(maximum_symmetric, 5e-12);
    }
  }
  std::cout << "Real synthetic epipolar global: visible=" << total_visible
            << ", max_algebraic=" << global_maximum_algebraic
            << ", max_forward=" << global_maximum_forward
            << ", max_backward=" << global_maximum_backward
            << ", max_symmetric=" << global_maximum_symmetric << " rad"
            << std::endl;
}

}  // namespace
}  // namespace sphere_vio
