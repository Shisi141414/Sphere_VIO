#pragma once

#include <Eigen/Core>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/panorama/panorama_spec.hpp"

namespace sphere_vio {

bool validatePanoramaSpec(const PanoramaSpec& panorama);
bool validatePanoramaForCamera(const RigCamera& camera,
                               const PanoramaSpec& panorama);
bool validatePanoramaForRig(const CameraRig& rig,
                            const PanoramaSpec& panorama);

// USPM axes: theta=atan2(n_P.x,n_P.z), phi=asin(n_P.y). Coordinates are
// continuous geometry, not integer image indices. Full-360 u is periodic.
bool sphereBearingToPanorama(const Eigen::Vector3d& sphere_bearing_p,
                             const PanoramaSpec& panorama,
                             Eigen::Vector2d* panorama_coordinate);
bool panoramaToSphereBearing(const Eigen::Vector2d& panorama_coordinate,
                             const PanoramaSpec& panorama,
                             Eigen::Vector3d* sphere_bearing_p);

bool cameraBearingToSphere(const RigCamera& camera,
                           const Eigen::Vector3d& bearing_c,
                           const PanoramaSpec& panorama,
                           Eigen::Vector3d* sphere_point_p,
                           Eigen::Vector3d* sphere_bearing_p);
bool cameraBearingToPanorama(const RigCamera& camera,
                             const Eigen::Vector3d& bearing_c,
                             const PanoramaSpec& panorama,
                             Eigen::Vector2d* panorama_coordinate,
                             Eigen::Vector3d* sphere_bearing_p = nullptr);
bool cameraPixelToPanorama(const RigCamera& camera,
                           const Eigen::Vector2d& pixel,
                           const PanoramaSpec& panorama,
                           Eigen::Vector2d* panorama_coordinate,
                           Eigen::Vector3d* sphere_bearing_p = nullptr);
bool panoramaToCameraBearing(const RigCamera& camera,
                             const Eigen::Vector2d& panorama_coordinate,
                             const PanoramaSpec& panorama,
                             Eigen::Vector3d* bearing_c);
bool panoramaToCameraPixel(const RigCamera& camera,
                           const Eigen::Vector2d& panorama_coordinate,
                           const PanoramaSpec& panorama,
                           Eigen::Vector2d* pixel);

}  // namespace sphere_vio
