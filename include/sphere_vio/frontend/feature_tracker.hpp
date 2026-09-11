#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/frontend/feature.hpp"

namespace sphere_vio {

class OmniRectifier;

struct FeatureTrackerOptions {
  cv::Size window_size = cv::Size(21, 21);
  int pyramid_levels = 4;
  int maximum_iterations = 30;
  double termination_epsilon = 0.01;
  double maximum_forward_backward_error = 1.0;
  double maximum_lk_error = 30.0;
  int border_margin = 12;
};

class FeatureTracker {
 public:
  explicit FeatureTracker(FeatureTrackerOptions options = {});

  const FeatureTrackerOptions& options() const { return options_; }
  void setPyramidLevels(int pyramid_levels);

  bool track(const cv::Mat& previous_image, const cv::Mat& current_image,
             Timestamp current_timestamp, CameraId camera_id,
             const CameraRig& camera_rig,
             const std::vector<FeatureTrack>& input_tracks,
             std::vector<FeatureTrack>* output_tracks,
             FeatureTrackingStatistics* statistics,
             const OmniRectifier* rectifier = nullptr) const;

 private:
  FeatureTrackerOptions options_;
};

}  // namespace sphere_vio
