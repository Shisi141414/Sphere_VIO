#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/frontend/feature.hpp"

namespace sphere_vio {

struct FeatureDetectorOptions {
  std::size_t maximum_features = 250U;
  int grid_rows = 6;
  int grid_columns = 8;
  int fast_threshold = 20;
  bool fast_nonmax_suppression = true;
  double minimum_feature_distance = 15.0;
  int border_margin = 12;
};

class FeatureDetector {
 public:
  explicit FeatureDetector(FeatureDetectorOptions options = {});

  const FeatureDetectorOptions& options() const { return options_; }
  void setMaximumFeatures(std::size_t maximum_features);

  bool detect(const cv::Mat& grayscale_image, Timestamp timestamp,
              CameraId camera_id, const CameraRig& camera_rig,
              const std::vector<Eigen::Vector2d>& occupied_pixels,
              std::size_t maximum_new_features,
              std::vector<FeatureObservation>* observations,
              FeatureDetectionStatistics* statistics) const;

 private:
  FeatureDetectorOptions options_;
};

}  // namespace sphere_vio
