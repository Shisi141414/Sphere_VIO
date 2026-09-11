#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/common/types.hpp"

namespace sphere_vio {

struct OmniRectifierOptions {
  int width = 800;
  int height = 400;
  double horizontal_fov = 200.0 * M_PI / 180.0;
  double vertical_fov = 100.0 * M_PI / 180.0;
};

// Precomputed rectification for Omni-Radtan fisheye cameras.
//
// The rectified view is a local equirectangular canvas: pixel u maps linearly
// to azimuth and pixel v maps linearly to elevation around the camera optical
// axis (+z, with +x right and +y down in OpenCV convention). All geometric
// consumers (bearings, epipolar checks, triangulation, MSCKF) keep using the
// original fisheye pixel through the camera model; the rectified image is only
// a more natural input for corner detection and optical flow.
class OmniRectifier {
 public:
  OmniRectifier(const CameraRig& rig, OmniRectifierOptions options = {});

  int width() const { return options_.width; }
  int height() const { return options_.height; }
  const OmniRectifierOptions& options() const { return options_; }

  bool rectifiedImage(CameraId camera_id, const cv::Mat& raw_image,
                      cv::Mat* rectified) const;
  bool rectifiedMask(CameraId camera_id, cv::Mat* mask) const;

  // Rectified virtual pixel -> original fisheye pixel.
  bool rawPixelFromRectified(CameraId camera_id,
                             const Eigen::Vector2d& rectified,
                             Eigen::Vector2d* raw) const;
  // Original fisheye pixel -> rectified virtual pixel.
  bool rectifiedPixelFromRaw(CameraId camera_id,
                             const Eigen::Vector2d& raw,
                             Eigen::Vector2d* rectified) const;

 private:
  struct CameraMap {
    cv::Mat map_x;
    cv::Mat map_y;
    cv::Mat mask;
  };

  CameraRig rig_;
  OmniRectifierOptions options_;
  std::array<CameraMap, 4> camera_maps_;
};

}  // namespace sphere_vio
