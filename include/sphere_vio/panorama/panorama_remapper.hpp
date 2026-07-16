#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/panorama/panorama_spec.hpp"

namespace sphere_vio {

struct PanoramaRemapOptions {
  int interpolation = cv::INTER_LINEAR;
  int border_mode = cv::BORDER_CONSTANT;
  double sampling_margin = 1.0;
  bool parallel_cameras = true;
};

struct CameraPanoramaRemap {
  CameraId camera_id = 0U;
  int source_width = 0;
  int source_height = 0;
  cv::Mat map_x;       // CV_32FC1, -1 at invalid destinations.
  cv::Mat map_y;       // CV_32FC1, -1 at invalid destinations.
  cv::Mat valid_mask;  // CV_8UC1, 0 or 255.
  cv::Mat owner_score; // CV_32FC1, bearing_c.z() or -infinity.
  std::size_t valid_pixel_count = 0U;
};

struct PanoramaLayer {
  CameraId camera_id = 0U;
  cv::Mat image;
  cv::Mat valid_mask;
};

struct PairOverlapMask {
  CameraId camera_id_1 = 0U;
  CameraId camera_id_2 = 0U;
  cv::Mat mask;
  std::size_t pixel_count = 0U;
};

struct PanoramaRemapResult {
  std::array<PanoramaLayer, 4> layers;
  cv::Mat coverage_count;
  cv::Mat owner_camera_id;  // CV_8SC1: -1 or CameraId 0..3.
  cv::Mat owner_selected_composite;
  std::vector<PairOverlapMask> configured_overlaps;
  double processing_time_ms = 0.0;
  double composite_time_ms = 0.0;
};

class PanoramaRemapper {
 public:
  bool initialize(const CameraRig& rig, const PanoramaSpec& panorama,
                  const PanoramaRemapOptions& options,
                  std::string* error = nullptr);
  bool remap(const std::array<cv::Mat, 4>& source_images,
             PanoramaRemapResult* result,
             std::string* error = nullptr) const;

  bool initialized() const { return initialized_; }
  const PanoramaSpec& panorama() const { return panorama_; }
  const PanoramaRemapOptions& options() const { return options_; }
  const std::array<CameraPanoramaRemap, 4>& cameraRemaps() const {
    return camera_remaps_;
  }
  const cv::Mat& coverageCount() const { return coverage_count_; }
  const cv::Mat& ownerCameraId() const { return owner_camera_id_; }
  const std::vector<PairOverlapMask>& configuredOverlaps() const {
    return configured_overlaps_;
  }
  double precomputeTimeMs() const { return precompute_time_ms_; }
  std::size_t staticMapBytes() const;

 private:
  PanoramaSpec panorama_;
  PanoramaRemapOptions options_;
  std::array<RigCamera, 4> cameras_;
  std::array<CameraPanoramaRemap, 4> camera_remaps_;
  cv::Mat coverage_count_;
  cv::Mat owner_camera_id_;
  std::vector<PairOverlapMask> configured_overlaps_;
  double precompute_time_ms_ = 0.0;
  bool initialized_ = false;
};

}  // namespace sphere_vio
