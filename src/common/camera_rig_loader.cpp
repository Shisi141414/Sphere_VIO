#include "sphere_vio/common/camera_rig_loader.hpp"

#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/LU>
#include <yaml-cpp/yaml.h>

#include "sphere_vio/camera/omni_radtan.hpp"

namespace sphere_vio {
namespace {

constexpr int kExpectedCameraCount = 4;
constexpr double kHomogeneousRowTolerance = 1e-12;
constexpr double kCameraChainConsistencyTolerance = 1e-9;

void setError(const std::string& message, std::string* error_message) {
  if (error_message) *error_message = message;
}

Eigen::Matrix4d parseTransform(const YAML::Node& node,
                               const std::string& field_name) {
  if (!node || !node.IsMap() || !node["rows"] || !node["cols"] ||
      !node["data"] || node["rows"].as<int>() != 4 ||
      node["cols"].as<int>() != 4 || !node["data"].IsSequence() ||
      node["data"].size() != 16U) {
    throw std::runtime_error(field_name + " must be a 4x4 matrix");
  }

  Eigen::Matrix4d transform;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      transform(row, column) =
          node["data"][static_cast<std::size_t>(row * 4 + column)]
              .as<double>();
    }
  }
  if (!transform.allFinite() ||
      transform.row(3).head<3>().norm() > kHomogeneousRowTolerance ||
      std::abs(transform(3, 3) - 1.0) > kHomogeneousRowTolerance) {
    throw std::runtime_error(field_name + " is not a finite homogeneous transform");
  }
  return transform;
}

std::vector<double> parseVector(const YAML::Node& node,
                                std::size_t expected_size,
                                const std::string& field_name) {
  if (!node || !node.IsSequence() || node.size() != expected_size) {
    throw std::runtime_error(field_name + " has an invalid size");
  }
  std::vector<double> values;
  values.reserve(expected_size);
  for (std::size_t index = 0; index < expected_size; ++index) {
    const double value = node[index].as<double>();
    if (!std::isfinite(value)) {
      throw std::runtime_error(field_name + " contains a non-finite value");
    }
    values.push_back(value);
  }
  return values;
}

RigCamera parseCamera(const YAML::Node& node, Eigen::Matrix4d* T_cam_imu,
                      Eigen::Matrix4d* T_cn_cnm1) {
  if (!node.IsMap() || !T_cam_imu || !T_cn_cnm1) {
    throw std::runtime_error("Invalid camera configuration entry");
  }

  const int camera_id = node["camera_id"].as<int>();
  if (camera_id < 0 || camera_id >= kExpectedCameraCount) {
    throw std::runtime_error("camera_id is outside [0, 3]");
  }
  const int kalibr_camera_id = node["kalibr_camera_id"].as<int>();
  if (kalibr_camera_id < 0 || kalibr_camera_id >= kExpectedCameraCount) {
    throw std::runtime_error("kalibr_camera_id is outside [0, 3]");
  }
  if (node["model"].as<std::string>() != "omni" ||
      node["distortion_model"].as<std::string>() != "radtan") {
    throw std::runtime_error("Only Kalibr omni+radtan is supported here");
  }

  const std::vector<double> resolution =
      parseVector(node["resolution"], 2U, "resolution");
  const std::vector<double> intrinsics =
      parseVector(node["intrinsics"], 5U, "intrinsics");
  const std::vector<double> distortion =
      parseVector(node["distortion_coeffs"], 4U, "distortion_coeffs");

  OmniRadtan::Parameters parameters;
  parameters.width = static_cast<int>(resolution[0]);
  parameters.height = static_cast<int>(resolution[1]);
  if (static_cast<double>(parameters.width) != resolution[0] ||
      static_cast<double>(parameters.height) != resolution[1]) {
    throw std::runtime_error("resolution must contain integer dimensions");
  }
  parameters.xi = intrinsics[0];
  parameters.fx = intrinsics[1];
  parameters.fy = intrinsics[2];
  parameters.cx = intrinsics[3];
  parameters.cy = intrinsics[4];
  parameters.k1 = distortion[0];
  parameters.k2 = distortion[1];
  parameters.p1 = distortion[2];
  parameters.p2 = distortion[3];

  *T_cam_imu = parseTransform(node["T_cam_imu"], "T_cam_imu");
  *T_cn_cnm1 = parseTransform(node["T_cn_cnm1"], "T_cn_cnm1");
  const Eigen::Matrix3d R_c_b = T_cam_imu->block<3, 3>(0, 0);
  const Eigen::Vector3d t_c_b = T_cam_imu->block<3, 1>(0, 3);

  RigCamera camera;
  camera.camera_id = static_cast<CameraId>(camera_id);
  camera.name = node["name"].as<std::string>();
  camera.model = std::make_shared<OmniRadtan>(parameters);
  camera.R_b_c = R_c_b.transpose();
  camera.t_b_c = -camera.R_b_c * t_c_b;
  camera.kalibr_camera_id = kalibr_camera_id;
  camera.timeshift_available = node["timeshift_available"].as<bool>();

  const YAML::Node timeshift = node["timeshift_cam_imu"];
  if (camera.timeshift_available) {
    if (!timeshift || timeshift.IsNull()) {
      throw std::runtime_error("Available timeshift_cam_imu has no value");
    }
    camera.timeshift_cam_imu = timeshift.as<double>();
    if (!std::isfinite(camera.timeshift_cam_imu)) {
      throw std::runtime_error("timeshift_cam_imu is not finite");
    }
  } else {
    if (!timeshift || !timeshift.IsNull()) {
      throw std::runtime_error("Unavailable timeshift_cam_imu must be null");
    }
    camera.timeshift_cam_imu = std::numeric_limits<double>::quiet_NaN();
  }
  return camera;
}

void validateCameraChain(
    const std::map<int, Eigen::Matrix4d>& T_cam_imu_by_kalibr_id,
    const std::map<int, Eigen::Matrix4d>& T_cn_cnm1_by_kalibr_id) {
  for (int kalibr_camera_id = 0; kalibr_camera_id < kExpectedCameraCount;
       ++kalibr_camera_id) {
    const int predecessor =
        (kalibr_camera_id + kExpectedCameraCount - 1) % kExpectedCameraCount;
    const Eigen::Matrix4d expected =
        T_cam_imu_by_kalibr_id.at(kalibr_camera_id) *
        T_cam_imu_by_kalibr_id.at(predecessor).inverse();
    const double consistency_error =
        (T_cn_cnm1_by_kalibr_id.at(kalibr_camera_id) - expected).norm();
    if (!std::isfinite(consistency_error) ||
        consistency_error > kCameraChainConsistencyTolerance) {
      throw std::runtime_error("T_cn_cnm1 is inconsistent with T_cam_imu");
    }
  }
}

}  // namespace

bool loadCameraRigFromYaml(const std::string& yaml_path, CameraRig* rig,
                           std::string* error_message) {
  if (error_message) error_message->clear();
  if (!rig) {
    setError("CameraRig output pointer is null", error_message);
    return false;
  }

  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const YAML::Node cameras = root["cameras"];
    if (!cameras || !cameras.IsSequence() ||
        cameras.size() != static_cast<std::size_t>(kExpectedCameraCount)) {
      throw std::runtime_error("cameras must contain exactly four entries");
    }

    CameraRig loaded_rig;
    std::set<int> kalibr_camera_ids;
    std::map<int, Eigen::Matrix4d> T_cam_imu_by_kalibr_id;
    std::map<int, Eigen::Matrix4d> T_cn_cnm1_by_kalibr_id;
    for (const YAML::Node& node : cameras) {
      Eigen::Matrix4d T_cam_imu;
      Eigen::Matrix4d T_cn_cnm1;
      RigCamera camera = parseCamera(node, &T_cam_imu, &T_cn_cnm1);
      if (!kalibr_camera_ids.insert(camera.kalibr_camera_id).second) {
        throw std::runtime_error("Duplicate kalibr_camera_id");
      }
      T_cam_imu_by_kalibr_id.emplace(camera.kalibr_camera_id, T_cam_imu);
      T_cn_cnm1_by_kalibr_id.emplace(camera.kalibr_camera_id, T_cn_cnm1);
      if (!loaded_rig.addCamera(camera)) {
        throw std::runtime_error("CameraRig rejected a camera entry");
      }
    }
    if (loaded_rig.size() != static_cast<std::size_t>(kExpectedCameraCount)) {
      throw std::runtime_error("CameraRig did not load exactly four cameras");
    }
    validateCameraChain(T_cam_imu_by_kalibr_id,
                        T_cn_cnm1_by_kalibr_id);
    *rig = std::move(loaded_rig);
    return true;
  } catch (const std::exception& exception) {
    setError(exception.what(), error_message);
    return false;
  }
}

}  // namespace sphere_vio
