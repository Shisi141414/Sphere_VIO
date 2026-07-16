#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/common/types.hpp"
#include "sphere_vio/frontend/feature_detector.hpp"
#include "sphere_vio/frontend/feature_tracker.hpp"

namespace sphere_vio {

struct TemporalFrontendOptions {
  FeatureDetectorOptions detector;
  FeatureTrackerOptions tracker;
  double redetection_ratio = 0.75;
};

struct CameraTrackingResult {
  CameraId camera_id = 0U;
  Timestamp timestamp = 0.0;
  std::vector<FeatureTrack> tracks;
  std::size_t input_tracks = 0U;
  std::size_t successfully_tracked = 0U;
  std::size_t newly_detected = 0U;
  std::size_t rejected_tracks = 0U;
  std::size_t model_domain_rejections = 0U;
  double average_forward_backward_error = 0.0;
  double maximum_forward_backward_error = 0.0;
  double average_track_age = 0.0;
  std::uint32_t maximum_track_age = 0U;
  double processing_time_seconds = 0.0;
  bool initialized_this_frame = false;
  bool reset_due_to_image_size = false;
  FeatureTrackingStatistics tracking_statistics;
  FeatureDetectionStatistics detection_statistics;
};

struct MultiCameraTrackingResult {
  Timestamp timestamp = 0.0;
  std::array<CameraTrackingResult, 4> cameras;
};

class TemporalFrontend {
 public:
  explicit TemporalFrontend(TemporalFrontendOptions options = {});

  bool processFrame(const MultiCameraFrame& frame,
                    const CameraRig& camera_rig,
                    MultiCameraTrackingResult* result);

  // Resetting state does not reuse FeatureIds during this object's lifetime.
  void reset();
  void resetCamera(CameraId camera_id);

 private:
  struct CameraTrackingState {
    bool initialized = false;
    Timestamp previous_timestamp = 0.0;
    cv::Mat previous_image;
    std::vector<FeatureTrack> tracks;
    std::uint64_t frame_count = 0U;
  };

  TemporalFrontendOptions options_;
  FeatureDetector detector_;
  FeatureTracker tracker_;
  std::array<CameraTrackingState, 4> states_;
  FeatureId next_feature_id_ = 1U;
};

}  // namespace sphere_vio
