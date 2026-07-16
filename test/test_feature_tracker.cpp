#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "frontend_test_utils.hpp"
#include "sphere_vio/frontend/feature_detector.hpp"
#include "sphere_vio/frontend/feature_tracker.hpp"

namespace sphere_vio {
namespace {

std::vector<FeatureTrack> detectTracks(const cv::Mat& image,
                                       const CameraRig& rig,
                                       CameraId camera_id = 0U) {
  FeatureDetectorOptions detector_options;
  detector_options.maximum_features = 120U;
  detector_options.minimum_feature_distance = 12.0;
  FeatureDetector detector(detector_options);
  std::vector<FeatureObservation> observations;
  FeatureDetectionStatistics statistics;
  if (!detector.detect(image, 1.0, camera_id, rig, {}, 120U, &observations,
                       &statistics)) {
    return {};
  }
  std::vector<FeatureTrack> tracks;
  FeatureId id = 10U;
  for (const FeatureObservation& observation : observations) {
    FeatureTrack track;
    track.id = id++;
    track.camera_id = camera_id;
    track.current = observation;
    track.age = 1U;
    track.total_observation_count = 1U;
    track.newly_detected = true;
    tracks.push_back(track);
  }
  return tracks;
}

TEST(FeatureTrackerTest, RecoversKnownTranslationAndPreservesIdentity) {
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat previous = frontend_test::makeCheckerTexture();
  const cv::Mat current = frontend_test::translate(previous, 3.0, 2.0);
  const std::vector<FeatureTrack> input = detectTracks(previous, rig);
  ASSERT_GT(input.size(), 30U);

  FeatureTracker tracker;
  std::vector<FeatureTrack> output;
  FeatureTrackingStatistics statistics;
  ASSERT_TRUE(tracker.track(previous, current, 1.05, 0U, rig, input, &output,
                            &statistics));
  ASSERT_GT(output.size(), input.size() / 2U);
  double dx_sum = 0.0;
  double dy_sum = 0.0;
  for (const FeatureTrack& track : output) {
    dx_sum += track.current.pixel.x() - track.previous.pixel.x();
    dy_sum += track.current.pixel.y() - track.previous.pixel.y();
    EXPECT_EQ(track.previous.camera_id, track.current.camera_id);
    EXPECT_EQ(0U, track.camera_id);
    EXPECT_EQ(2U, track.age);
    EXPECT_EQ(2U, track.total_observation_count);
    EXPECT_TRUE(track.has_previous_observation);
    EXPECT_FALSE(track.newly_detected);
    EXPECT_NEAR(1.0, track.current.bearing_c.norm(), 1e-12);
    EXPECT_NEAR(1.0, track.current.bearing_b.norm(), 1e-12);
    const auto found = std::find_if(
        input.begin(), input.end(), [&](const FeatureTrack& candidate) {
          return candidate.id == track.id;
        });
    EXPECT_NE(input.end(), found);
  }
  const double mean_dx = dx_sum / output.size();
  const double mean_dy = dy_sum / output.size();
  std::cout << "Synthetic LK displacement: dx=" << mean_dx
            << " dy=" << mean_dy << " FB avg/max="
            << statistics.average_forward_backward_error << "/"
            << statistics.maximum_forward_backward_error << std::endl;
  EXPECT_NEAR(3.0, mean_dx, 0.15);
  EXPECT_NEAR(2.0, mean_dy, 0.15);
  EXPECT_LT(statistics.average_forward_backward_error, 0.2);
  EXPECT_EQ(input.size(), statistics.successfully_tracked +
                              statistics.rejectedCount());
}

TEST(FeatureTrackerTest, OcclusionAndIncorrectTextureProduceRejections) {
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat previous = frontend_test::makeCheckerTexture();
  cv::Mat current = frontend_test::translate(previous, 3.0, 2.0);
  cv::rectangle(current, cv::Rect(80, 50, 150, 130), cv::Scalar(128),
                cv::FILLED);
  const std::vector<FeatureTrack> input = detectTracks(previous, rig);
  FeatureTracker tracker;
  std::vector<FeatureTrack> output;
  FeatureTrackingStatistics statistics;
  ASSERT_TRUE(tracker.track(previous, current, 1.05, 0U, rig, input, &output,
                            &statistics));
  std::cout << "Occlusion rejection counts: forward="
            << statistics.forward_flow_failures << " backward="
            << statistics.backward_flow_failures << " FB="
            << statistics.forward_backward_rejections << " LK="
            << statistics.lk_error_rejections << " outside="
            << statistics.outside_image_rejections << std::endl;
  EXPECT_GT(statistics.rejectedCount(), 0U);
  EXPECT_LT(output.size(), input.size());
  EXPECT_GT(statistics.forward_flow_failures +
                statistics.backward_flow_failures +
                statistics.forward_backward_rejections +
                statistics.lk_error_rejections,
            0U);
  EXPECT_GT(statistics.backward_flow_failures +
                statistics.forward_backward_rejections,
            0U);
  for (const FeatureTrack& track : output) {
    EXPECT_TRUE(track.current.pixel.allFinite());
    EXPECT_TRUE(track.current.bearing_c.allFinite());
    EXPECT_TRUE(track.current.bearing_b.allFinite());
  }
}

TEST(FeatureTrackerTest, BorderExitAndLargeMotionFailSafely) {
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat previous = frontend_test::makeCheckerTexture();
  const std::vector<FeatureTrack> input = detectTracks(previous, rig);
  ASSERT_FALSE(input.empty());
  const cv::Mat large_motion = frontend_test::translate(previous, 140.0, 0.0);
  FeatureTracker tracker;
  std::vector<FeatureTrack> output;
  FeatureTrackingStatistics statistics;
  ASSERT_TRUE(tracker.track(previous, large_motion, 1.05, 0U, rig, input,
                            &output, &statistics));
  EXPECT_LT(output.size(), input.size());
  EXPECT_GT(statistics.rejectedCount(), 0U);
  EXPECT_EQ(input.size(), output.size() + statistics.rejectedCount());
}

TEST(FeatureTrackerTest, InvalidImageAndCrossCameraTrackAreRejected) {
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat image = frontend_test::makeCheckerTexture();
  std::vector<FeatureTrack> input = detectTracks(image, rig);
  ASSERT_FALSE(input.empty());
  input.front().camera_id = 1U;
  FeatureTracker tracker;
  std::vector<FeatureTrack> output;
  FeatureTrackingStatistics statistics;
  EXPECT_FALSE(tracker.track(image, image, 2.0, 0U, rig, input, &output,
                             &statistics));
  EXPECT_FALSE(tracker.track(cv::Mat(), image, 2.0, 0U, rig, {}, &output,
                             &statistics));
  EXPECT_FALSE(tracker.track(image, cv::Mat(120, 160, CV_8UC1), 2.0, 0U,
                             rig, {}, &output, &statistics));
}

}  // namespace
}  // namespace sphere_vio
