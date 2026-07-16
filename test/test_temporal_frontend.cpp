#include <algorithm>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "frontend_test_utils.hpp"
#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/frontend/temporal_frontend.hpp"

namespace sphere_vio {
namespace {

TemporalFrontendOptions testOptions() {
  TemporalFrontendOptions options;
  options.detector.maximum_features = 80U;
  options.detector.minimum_feature_distance = 13.0;
  options.redetection_ratio = 0.75;
  return options;
}

TEST(TemporalFrontendTest, FourCamerasTrackIndependentlyWithGlobalIds) {
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat image = frontend_test::makeCheckerTexture();
  TemporalFrontend frontend(testOptions());
  MultiCameraTrackingResult first;
  ASSERT_TRUE(frontend.processFrame(
      frontend_test::makeFrame(image, 1.0, 1.0), rig, &first));

  std::set<FeatureId> all_ids;
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    const CameraTrackingResult& result = first.cameras[camera_id];
    EXPECT_TRUE(result.initialized_this_frame);
    EXPECT_EQ(result.tracks.size(), result.newly_detected);
    ASSERT_GT(result.tracks.size(), 20U);
    for (const FeatureTrack& track : result.tracks) {
      EXPECT_EQ(camera_id, track.camera_id);
      EXPECT_EQ(camera_id, track.current.camera_id);
      EXPECT_EQ(1U, track.age);
      EXPECT_TRUE(track.newly_detected);
      EXPECT_TRUE(all_ids.insert(track.id).second);
    }
  }

  MultiCameraTrackingResult second;
  ASSERT_TRUE(frontend.processFrame(
      frontend_test::makeFrame(frontend_test::translate(image, 3.0, 2.0),
                               1.05, 1.0),
      rig, &second));
  std::set<FeatureId> second_ids;
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    const CameraTrackingResult& result = second.cameras[camera_id];
    EXPECT_FALSE(result.initialized_this_frame);
    EXPECT_GT(result.successfully_tracked, 20U);
    EXPECT_EQ(result.input_tracks,
              result.successfully_tracked + result.rejected_tracks);
    for (const FeatureTrack& track : result.tracks) {
      EXPECT_EQ(camera_id, track.camera_id);
      EXPECT_TRUE(second_ids.insert(track.id).second);
      if (!track.newly_detected) {
        EXPECT_EQ(2U, track.age);
        EXPECT_TRUE(all_ids.count(track.id) == 1U);
      }
    }
  }
}

TEST(TemporalFrontendTest, NonIncreasingTimeFailsWithoutCorruptingState) {
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat image = frontend_test::makeCheckerTexture();
  TemporalFrontend frontend(testOptions());
  MultiCameraTrackingResult result;
  ASSERT_TRUE(
      frontend.processFrame(frontend_test::makeFrame(image, 1.0), rig,
                            &result));
  EXPECT_FALSE(
      frontend.processFrame(frontend_test::makeFrame(image, 1.0), rig,
                            &result));
  ASSERT_TRUE(frontend.processFrame(
      frontend_test::makeFrame(frontend_test::translate(image, 2.0, 0.0),
                               1.05),
      rig, &result));
  EXPECT_GT(result.cameras[0].successfully_tracked, 0U);
}

TEST(TemporalFrontendTest, ImageSizeChangeResetsOnlyAffectedCameraAndIds) {
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat image = frontend_test::makeCheckerTexture();
  TemporalFrontend frontend(testOptions());
  MultiCameraTrackingResult first;
  ASSERT_TRUE(frontend.processFrame(frontend_test::makeFrame(image, 1.0), rig,
                                    &first));
  FeatureId maximum_first_id = 0U;
  for (const CameraTrackingResult& result : first.cameras)
    for (const FeatureTrack& track : result.tracks)
      maximum_first_id = std::max(maximum_first_id, track.id);

  MultiCameraFrame changed =
      frontend_test::makeFrame(frontend_test::translate(image, 2.0, 1.0),
                               1.05);
  changed.images[2].image = frontend_test::makeCheckerTexture(300, 220);
  MultiCameraTrackingResult second;
  ASSERT_TRUE(frontend.processFrame(changed, rig, &second));
  EXPECT_TRUE(second.cameras[2].reset_due_to_image_size);
  EXPECT_TRUE(second.cameras[2].initialized_this_frame);
  EXPECT_EQ(0U, second.cameras[2].successfully_tracked);
  for (const FeatureTrack& track : second.cameras[2].tracks) {
    EXPECT_GT(track.id, maximum_first_id);
    EXPECT_EQ(1U, track.age);
  }
  EXPECT_FALSE(second.cameras[0].reset_due_to_image_size);
  EXPECT_GT(second.cameras[0].successfully_tracked, 0U);
}

TEST(TemporalFrontendTest, RealRigProducesUnitBearingsWithCorrectMapping) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(
      std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml", &rig,
      &error))
      << error;
  ASSERT_EQ(4U, rig.size());
  ASSERT_EQ(3, rig.camera(2U)->kalibr_camera_id);
  ASSERT_EQ(2, rig.camera(3U)->kalibr_camera_id);

  TemporalFrontendOptions options = testOptions();
  options.detector.maximum_features = 40U;
  TemporalFrontend frontend(options);
  const cv::Mat image = frontend_test::makeCheckerTexture(1088, 880, 32);
  MultiCameraTrackingResult result;
  ASSERT_TRUE(frontend.processFrame(frontend_test::makeFrame(image, 2.0), rig,
                                    &result));
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    ASSERT_FALSE(result.cameras[camera_id].tracks.empty());
    for (const FeatureTrack& track : result.cameras[camera_id].tracks) {
      EXPECT_EQ(camera_id, track.camera_id);
      EXPECT_NEAR(1.0, track.current.bearing_c.norm(), 1e-10);
      EXPECT_NEAR(1.0, track.current.bearing_b.norm(), 1e-10);
      EXPECT_TRUE(track.current.bearing_c.allFinite());
      EXPECT_TRUE(track.current.bearing_b.allFinite());
    }
  }
}

TEST(TemporalFrontendTest, ResetDoesNotReuseFeatureIds) {
  const CameraRig rig = frontend_test::makeRig();
  const cv::Mat image = frontend_test::makeCheckerTexture();
  TemporalFrontend frontend(testOptions());
  MultiCameraTrackingResult first;
  ASSERT_TRUE(frontend.processFrame(frontend_test::makeFrame(image, 1.0), rig,
                                    &first));
  FeatureId maximum_first_id = 0U;
  for (const CameraTrackingResult& result : first.cameras)
    for (const FeatureTrack& track : result.tracks)
      maximum_first_id = std::max(maximum_first_id, track.id);
  frontend.reset();
  MultiCameraTrackingResult second;
  ASSERT_TRUE(frontend.processFrame(frontend_test::makeFrame(image, 1.0), rig,
                                    &second));
  for (const CameraTrackingResult& result : second.cameras)
    for (const FeatureTrack& track : result.tracks)
      EXPECT_GT(track.id, maximum_first_id);
}

}  // namespace
}  // namespace sphere_vio
