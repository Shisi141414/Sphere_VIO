#pragma once

#include <string>

#include "sphere_vio/camera/camera_rig.hpp"

namespace sphere_vio {

// Loads raw Kalibr T_cam_imu matrices and explicitly inverts them to obtain
// the T_b_c convention stored by CameraRig. The loader has no ROS dependency.
bool loadCameraRigFromYaml(const std::string& yaml_path, CameraRig* rig,
                           std::string* error_message = nullptr);

}  // namespace sphere_vio
