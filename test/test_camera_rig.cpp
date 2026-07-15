#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <yaml-cpp/yaml.h>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/camera/kannala_brandt.hpp"
#include "sphere_vio/camera/omni_radtan.hpp"
#include "sphere_vio/common/camera_rig_loader.hpp"

namespace sphere_vio {
namespace {

std::string realCameraConfigPath() {
  return std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml";
}

OmniRadtan::Parameters syntheticOmniParameters(double fx = 420.0,
                                                double cx = 319.5) {
  // SYNTHETIC CAMERA PARAMETERS. These are not real calibration values.
  OmniRadtan::Parameters parameters;
  parameters.width = 640;
  parameters.height = 480;
  parameters.xi = 1.0;
  parameters.fx = fx;
  parameters.fy = 415.0;
  parameters.cx = cx;
  parameters.cy = 239.5;
  parameters.k1 = -0.01;
  parameters.k2 = 0.02;
  parameters.p1 = 0.001;
  parameters.p2 = -0.001;
  return parameters;
}

RigCamera syntheticCamera(
    CameraId camera_id, const Eigen::Matrix3d& R_b_c =
                            Eigen::Matrix3d::Identity(),
    const Eigen::Vector3d& t_b_c = Eigen::Vector3d::Zero(),
    double fx = 420.0, double cx = 319.5) {
  RigCamera camera;
  camera.camera_id = camera_id;
  camera.name = "synthetic_" + std::to_string(camera_id);
  camera.model = std::make_shared<OmniRadtan>(
      syntheticOmniParameters(fx, cx));
  camera.R_b_c = R_b_c;
  camera.t_b_c = t_b_c;
  return camera;
}

double bearingAngle(const Eigen::Vector3d& first,
                    const Eigen::Vector3d& second) {
  const double dot = std::max(-1.0, std::min(1.0, first.dot(second)));
  return std::atan2(first.cross(second).norm(), dot);
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

struct RealTransformRecord {
  CameraId project_camera_id = 0;
  int kalibr_camera_id = -1;
  std::string name;
  Eigen::Matrix4d T_cam_imu = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_cn_cnm1 = Eigen::Matrix4d::Identity();
};

std::vector<RealTransformRecord> readRealTransforms() {
  const YAML::Node cameras = YAML::LoadFile(realCameraConfigPath())["cameras"];
  std::vector<RealTransformRecord> records;
  for (const YAML::Node& camera : cameras) {
    RealTransformRecord record;
    record.project_camera_id = camera["camera_id"].as<CameraId>();
    record.kalibr_camera_id = camera["kalibr_camera_id"].as<int>();
    record.name = camera["name"].as<std::string>();
    record.T_cam_imu = parseTransform(camera["T_cam_imu"]);
    record.T_cn_cnm1 = parseTransform(camera["T_cn_cnm1"]);
    records.push_back(record);
  }
  return records;
}

TEST(CameraRigTest, IdentityExtrinsicManagesSingleCamera) {
  CameraRig rig;
  ASSERT_TRUE(rig.addCamera(syntheticCamera(0U)));
  EXPECT_EQ(1U, rig.size());
  EXPECT_TRUE(rig.hasCamera(0U));
  EXPECT_FALSE(rig.hasCamera(1U));
  ASSERT_NE(nullptr, rig.camera(0U));
  EXPECT_EQ("synthetic_0", rig.camera(0U)->name);
  EXPECT_EQ(nullptr, rig.camera(3U));

  const Eigen::Vector3d bearing_c(0.2, -0.3, 0.9);
  Eigen::Vector3d bearing_b;
  ASSERT_TRUE(rig.cameraBearingToBody(0U, bearing_c, &bearing_b));
  EXPECT_TRUE(bearing_b.isApprox(bearing_c.normalized(), 1e-15));
}

TEST(CameraRigTest, KnownRotationAffectsBearingButTranslationDoesNot) {
  const Eigen::Matrix3d R_b_c =
      Eigen::AngleAxisd(0.5 * 3.14159265358979323846,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  CameraRig rig;
  ASSERT_TRUE(rig.addCamera(
      syntheticCamera(0U, R_b_c, Eigen::Vector3d(10.0, -20.0, 30.0))));

  Eigen::Vector3d bearing_b;
  ASSERT_TRUE(rig.cameraBearingToBody(0U, Eigen::Vector3d::UnitX(),
                                     &bearing_b));
  EXPECT_TRUE(bearing_b.isApprox(Eigen::Vector3d::UnitY(), 1e-15));

  Eigen::Vector3d recovered_bearing_c;
  ASSERT_TRUE(rig.bodyBearingToCamera(0U, bearing_b, &recovered_bearing_c));
  EXPECT_TRUE(
      recovered_bearing_c.isApprox(Eigen::Vector3d::UnitX(), 1e-15));
}

TEST(CameraRigTest, PointTransformsUseRotationAndTranslationAndRoundTrip) {
  const Eigen::Matrix3d R_b_c =
      Eigen::AngleAxisd(0.5 * 3.14159265358979323846,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d t_b_c(1.0, 2.0, 3.0);
  CameraRig rig;
  ASSERT_TRUE(rig.addCamera(syntheticCamera(0U, R_b_c, t_b_c)));

  const Eigen::Vector3d point_c(2.0, 0.0, 1.0);
  Eigen::Vector3d point_b;
  ASSERT_TRUE(rig.cameraPointToBody(0U, point_c, &point_b));
  EXPECT_TRUE(point_b.isApprox(Eigen::Vector3d(1.0, 4.0, 4.0), 1e-15));

  Eigen::Vector3d recovered_point_c;
  ASSERT_TRUE(rig.bodyPointToCamera(0U, point_b, &recovered_point_c));
  EXPECT_TRUE(recovered_point_c.isApprox(point_c, 1e-15));
}

TEST(CameraRigTest, PixelAndBodyPointConvenienceFunctionsAreConsistent) {
  const Eigen::Vector3d t_b_c(1.0, 2.0, 3.0);
  CameraRig rig;
  ASSERT_TRUE(rig.addCamera(
      syntheticCamera(0U, Eigen::Matrix3d::Identity(), t_b_c)));

  Eigen::Vector3d bearing_b;
  ASSERT_TRUE(rig.pixelToBodyBearing(0U, Eigen::Vector2d(319.5, 239.5),
                                    &bearing_b));
  EXPECT_TRUE(bearing_b.isApprox(Eigen::Vector3d::UnitZ(), 1e-15));

  Eigen::Vector2d pixel;
  ASSERT_TRUE(rig.projectBodyPointToCamera(
      0U, t_b_c + 5.0 * Eigen::Vector3d::UnitZ(), &pixel));
  EXPECT_TRUE(pixel.isApprox(Eigen::Vector2d(319.5, 239.5), 1e-15));
}

TEST(CameraRigTest, InvalidIdsInputsAndOutputPointersFailSafely) {
  CameraRig rig;
  ASSERT_TRUE(rig.addCamera(syntheticCamera(0U)));
  Eigen::Vector2d pixel;
  Eigen::Vector3d vector;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(rig.pixelToCameraBearing(3U, Eigen::Vector2d::Zero(), &vector));
  EXPECT_FALSE(rig.cameraBearingToBody(3U, Eigen::Vector3d::UnitZ(), &vector));
  EXPECT_FALSE(rig.bodyBearingToCamera(3U, Eigen::Vector3d::UnitZ(), &vector));
  EXPECT_FALSE(rig.cameraPointToBody(3U, Eigen::Vector3d::Zero(), &vector));
  EXPECT_FALSE(rig.bodyPointToCamera(3U, Eigen::Vector3d::Zero(), &vector));
  EXPECT_FALSE(rig.projectBodyPointToCamera(
      3U, Eigen::Vector3d::UnitZ(), &pixel));
  EXPECT_FALSE(rig.cameraBearingToBody(0U, Eigen::Vector3d::Zero(), &vector));
  EXPECT_FALSE(rig.bodyBearingToCamera(0U, Eigen::Vector3d::Zero(), &vector));
  EXPECT_FALSE(rig.cameraBearingToBody(
      0U, Eigen::Vector3d(nan, 0.0, 1.0), &vector));
  EXPECT_FALSE(rig.pixelToCameraBearing(0U, Eigen::Vector2d::Zero(), nullptr));
  EXPECT_FALSE(rig.cameraBearingToBody(
      0U, Eigen::Vector3d::UnitZ(), nullptr));
  EXPECT_FALSE(rig.bodyBearingToCamera(
      0U, Eigen::Vector3d::UnitZ(), nullptr));
  EXPECT_FALSE(rig.pixelToBodyBearing(0U, Eigen::Vector2d::Zero(), nullptr));
  EXPECT_FALSE(rig.cameraPointToBody(0U, Eigen::Vector3d::Zero(), nullptr));
  EXPECT_FALSE(rig.bodyPointToCamera(0U, Eigen::Vector3d::Zero(), nullptr));
  EXPECT_FALSE(rig.projectBodyPointToCamera(
      0U, Eigen::Vector3d::UnitZ(), nullptr));
}

TEST(CameraRigTest, RejectsInvalidCameraDefinitionsAndDuplicateIds) {
  const auto expect_rejected = [](const RigCamera& camera) {
    CameraRig rig;
    EXPECT_FALSE(rig.addCamera(camera));
    EXPECT_EQ(0U, rig.size());
  };

  RigCamera invalid = syntheticCamera(0U);
  invalid.model.reset();
  expect_rejected(invalid);
  invalid = syntheticCamera(0U);
  invalid.name.clear();
  expect_rejected(invalid);
  invalid = syntheticCamera(4U);
  expect_rejected(invalid);
  invalid = syntheticCamera(0U);
  invalid.R_b_c(0, 0) = 2.0;
  expect_rejected(invalid);
  invalid = syntheticCamera(0U);
  invalid.R_b_c = Eigen::Vector3d(-1.0, 1.0, 1.0).asDiagonal();
  expect_rejected(invalid);
  invalid = syntheticCamera(0U);
  invalid.R_b_c(0, 0) = std::numeric_limits<double>::quiet_NaN();
  expect_rejected(invalid);
  invalid = syntheticCamera(0U);
  invalid.t_b_c.x() = std::numeric_limits<double>::infinity();
  expect_rejected(invalid);
  invalid = syntheticCamera(0U);
  invalid.timeshift_available = true;
  expect_rejected(invalid);
  invalid = syntheticCamera(0U);
  invalid.timeshift_cam_imu = 0.0;
  expect_rejected(invalid);

  CameraRig rig;
  ASSERT_TRUE(rig.addCamera(syntheticCamera(0U)));
  EXPECT_FALSE(rig.addCamera(syntheticCamera(0U)));
  EXPECT_EQ(1U, rig.size());
}

TEST(CameraRigTest, SupportsDifferentCameraModelsAndIntrinsics) {
  CameraRig rig;
  ASSERT_TRUE(rig.addCamera(syntheticCamera(0U, Eigen::Matrix3d::Identity(),
                                            Eigen::Vector3d::Zero(), 400.0,
                                            300.0)));

  KannalaBrandt::Parameters kb4_parameters;
  kb4_parameters.width = 800;
  kb4_parameters.height = 600;
  kb4_parameters.fx = 500.0;
  kb4_parameters.fy = 490.0;
  kb4_parameters.cx = 410.0;
  kb4_parameters.cy = 295.0;
  RigCamera second = syntheticCamera(1U);
  second.model = std::make_shared<KannalaBrandt>(kb4_parameters);
  ASSERT_TRUE(rig.addCamera(second));

  ASSERT_NE(nullptr, rig.camera(0U));
  ASSERT_NE(nullptr, rig.camera(1U));
  EXPECT_EQ("omni_radtan", rig.camera(0U)->model->modelName());
  EXPECT_EQ("KannalaBrandtKB4", rig.camera(1U)->model->modelName());
  EXPECT_NE(rig.camera(0U)->model->width(), rig.camera(1U)->model->width());

  Eigen::Vector2d first_pixel;
  Eigen::Vector2d second_pixel;
  ASSERT_TRUE(rig.projectBodyPointToCamera(
      0U, Eigen::Vector3d::UnitZ(), &first_pixel));
  ASSERT_TRUE(rig.projectBodyPointToCamera(
      1U, Eigen::Vector3d::UnitZ(), &second_pixel));
  EXPECT_TRUE(first_pixel.isApprox(Eigen::Vector2d(300.0, 239.5), 1e-15));
  EXPECT_TRUE(second_pixel.isApprox(Eigen::Vector2d(410.0, 295.0), 1e-15));
}

TEST(CameraRigTest, LoadsRealRigWithExplicitProjectToKalibrMapping) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &error))
      << error;
  ASSERT_EQ(4U, rig.size());

  const int expected_kalibr_ids[4] = {0, 1, 3, 2};
  const char* expected_names[4] = {"left", "right", "bleft", "bright"};
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    const RigCamera* camera = rig.camera(camera_id);
    ASSERT_NE(nullptr, camera);
    EXPECT_EQ(expected_kalibr_ids[camera_id], camera->kalibr_camera_id);
    EXPECT_EQ(expected_names[camera_id], camera->name);
    EXPECT_EQ("omni_radtan", camera->model->modelName());
    EXPECT_EQ(1088, camera->model->width());
    EXPECT_EQ(880, camera->model->height());
  }
  EXPECT_TRUE(rig.camera(0U)->timeshift_available);
  EXPECT_TRUE(rig.camera(1U)->timeshift_available);
  EXPECT_FALSE(rig.camera(2U)->timeshift_available);
  EXPECT_FALSE(rig.camera(3U)->timeshift_available);
  EXPECT_TRUE(std::isnan(rig.camera(2U)->timeshift_cam_imu));
  EXPECT_TRUE(std::isnan(rig.camera(3U)->timeshift_cam_imu));
}

TEST(CameraRigTest, KalibrTCamImuIsInvertedForBodyToCameraConvention) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &error))
      << error;
  const std::vector<RealTransformRecord> records = readRealTransforms();
  const Eigen::Vector4d point_b_h(0.31, -0.27, 1.42, 1.0);

  for (const RealTransformRecord& record : records) {
    const Eigen::Vector3d expected_point_c =
        (record.T_cam_imu * point_b_h).head<3>();
    Eigen::Vector3d point_c;
    ASSERT_TRUE(rig.bodyPointToCamera(record.project_camera_id,
                                      point_b_h.head<3>(), &point_c));
    EXPECT_TRUE(point_c.isApprox(expected_point_c, 1e-14));

    Eigen::Vector3d recovered_point_b;
    ASSERT_TRUE(rig.cameraPointToBody(record.project_camera_id, point_c,
                                      &recovered_point_b));
    EXPECT_LT((recovered_point_b - point_b_h.head<3>()).norm(), 5e-14);
  }
}

TEST(CameraRigTest, ModelDomainFailuresPropagateWithoutFabricatingBearing) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &error))
      << error;
  Eigen::Vector3d bearing_b = Eigen::Vector3d::Constant(123.0);
  EXPECT_TRUE(rig.camera(0U)->model->isPixelValid(Eigen::Vector2d(0.0, 0.0)));
  EXPECT_FALSE(
      rig.pixelToBodyBearing(0U, Eigen::Vector2d(0.0, 0.0), &bearing_b));
  EXPECT_TRUE(bearing_b.isApprox(Eigen::Vector3d::Constant(123.0)));
}

TEST(CameraRigTest, RealExtrinsicsAndCameraChainAreConsistent) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(realCameraConfigPath(), &rig, &error))
      << error;
  const std::vector<RealTransformRecord> records = readRealTransforms();
  ASSERT_EQ(4U, records.size());

  std::map<int, RealTransformRecord> by_kalibr_id;
  for (const RealTransformRecord& record : records) {
    by_kalibr_id.emplace(record.kalibr_camera_id, record);
  }

  double maximum_chain_error = 0.0;
  double maximum_bearing_angle = 0.0;
  std::mt19937 generator(20260716U);
  std::uniform_real_distribution<double> component(-1.0, 1.0);

  for (const RealTransformRecord& record : records) {
    const RigCamera* camera = rig.camera(record.project_camera_id);
    ASSERT_NE(nullptr, camera);
    const double determinant = camera->R_b_c.determinant();
    const double orthogonality_error =
        (camera->R_b_c.transpose() * camera->R_b_c -
         Eigen::Matrix3d::Identity())
            .norm();
    const double body_distance = camera->t_b_c.norm();

    Eigen::Matrix4d T_b_c = Eigen::Matrix4d::Identity();
    T_b_c.block<3, 3>(0, 0) = camera->R_b_c;
    T_b_c.block<3, 1>(0, 3) = camera->t_b_c;
    const double inverse_recovery_error =
        (T_b_c.inverse() - record.T_cam_imu).norm();
    const double double_inverse_error =
        (record.T_cam_imu.inverse().inverse() - record.T_cam_imu).norm();

    std::cout << "Real rig C" << record.project_camera_id
              << " <- Kalibr cam" << record.kalibr_camera_id << " ("
              << record.name << "): det(R_b_c)=" << determinant
              << ", orthogonality_error=" << orthogonality_error
              << ", t_b_c=[" << camera->t_b_c.transpose()
              << "] m, body_distance=" << body_distance
              << " m, inverse_recovery_error=" << inverse_recovery_error
              << std::endl;

    EXPECT_NEAR(1.0, determinant, 1e-12);
    EXPECT_LT(orthogonality_error, 1e-12);
    EXPECT_TRUE(camera->t_b_c.allFinite());
    EXPECT_GT(body_distance, 0.04);
    EXPECT_LT(body_distance, 0.06);
    EXPECT_LT(inverse_recovery_error, 1e-12);
    EXPECT_LT(double_inverse_error, 1e-12);

    for (int sample = 0; sample < 250; ++sample) {
      Eigen::Vector3d bearing_b(component(generator), component(generator),
                                component(generator));
      if (bearing_b.norm() < 1e-6) {
        --sample;
        continue;
      }
      bearing_b.normalize();
      Eigen::Vector3d bearing_c;
      Eigen::Vector3d recovered_bearing_b;
      ASSERT_TRUE(rig.bodyBearingToCamera(record.project_camera_id,
                                          bearing_b, &bearing_c));
      ASSERT_TRUE(rig.cameraBearingToBody(record.project_camera_id,
                                          bearing_c,
                                          &recovered_bearing_b));
      maximum_bearing_angle =
          std::max(maximum_bearing_angle,
                   bearingAngle(bearing_b, recovered_bearing_b));
    }
  }

  for (int kalibr_camera_id = 0; kalibr_camera_id < 4;
       ++kalibr_camera_id) {
    const int predecessor = (kalibr_camera_id + 3) % 4;
    const RealTransformRecord& current = by_kalibr_id.at(kalibr_camera_id);
    const RealTransformRecord& previous = by_kalibr_id.at(predecessor);
    const Eigen::Matrix4d expected =
        current.T_cam_imu * previous.T_cam_imu.inverse();
    const double chain_error = (current.T_cn_cnm1 - expected).norm();
    maximum_chain_error = std::max(maximum_chain_error, chain_error);

    const RigCamera* current_camera = rig.camera(current.project_camera_id);
    const RigCamera* previous_camera = rig.camera(previous.project_camera_id);
    ASSERT_NE(nullptr, current_camera);
    ASSERT_NE(nullptr, previous_camera);
    const double baseline =
        (current_camera->t_b_c - previous_camera->t_b_c).norm();
    const double relative_rotation_angle = std::acos(std::max(
        -1.0, std::min(1.0, 0.5 *
                                (current.T_cn_cnm1.block<3, 3>(0, 0).trace() -
                                 1.0))));
    std::cout << "Kalibr cam" << kalibr_camera_id << " <- cam"
              << predecessor << ": baseline=" << baseline
              << " m, relative_rotation=" << relative_rotation_angle
              << " rad, T_cn_cnm1_error=" << chain_error << std::endl;
    EXPECT_GT(baseline, 0.06);
    EXPECT_LT(baseline, 0.075);
    EXPECT_GT(relative_rotation_angle, 1.4);
    EXPECT_LT(relative_rotation_angle, 1.75);
    EXPECT_LT(chain_error, 1e-12);
  }

  std::cout << "Real rig maxima: T_cn_cnm1_error=" << maximum_chain_error
            << ", bearing_round_trip_angle=" << maximum_bearing_angle
            << " rad" << std::endl;
  EXPECT_LT(maximum_chain_error, 1e-12);
  EXPECT_LT(maximum_bearing_angle, 1e-12);
}

TEST(CameraRigTest, LoaderFailuresAreExplicit) {
  CameraRig rig;
  std::string error;
  EXPECT_FALSE(loadCameraRigFromYaml(realCameraConfigPath(), nullptr, &error));
  EXPECT_FALSE(error.empty());
  error.clear();
  EXPECT_FALSE(loadCameraRigFromYaml(
      std::string(SPHERE_VIO_SOURCE_DIR) + "/config/not_present.yaml", &rig,
      &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace sphere_vio
