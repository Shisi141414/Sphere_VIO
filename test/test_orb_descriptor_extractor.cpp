#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "frontend_test_utils.hpp"
#include "sphere_vio/frontend/orb_descriptor_extractor.hpp"

namespace sphere_vio {
namespace {

CameraTrackingResult makeTrackingResult(
    CameraId camera_id, const std::vector<Eigen::Vector2d>& pixels,
    FeatureId first_id = (FeatureId{1} << 40U)) {
  const CameraRig rig = frontend_test::makeRig();
  CameraTrackingResult result;
  result.camera_id = camera_id;
  result.timestamp = 10.0;
  for (std::size_t index = 0U; index < pixels.size(); ++index) {
    FeatureTrack track;
    track.id = first_id + index;
    track.camera_id = camera_id;
    track.current.camera_id = camera_id;
    track.current.timestamp = result.timestamp;
    track.current.pixel = pixels[index];
    if (rig.hasCamera(camera_id)) {
      rig.pixelToCameraBearing(camera_id, pixels[index],
                               &track.current.bearing_c);
      rig.cameraBearingToBody(camera_id, track.current.bearing_c,
                              &track.current.bearing_b);
    }
    track.age = 1U;
    track.total_observation_count = 1U;
    result.tracks.push_back(track);
  }
  return result;
}

TEST(OrbDescriptorExtractorTest, Computes32ByteDescriptorsAtExistingTracks) {
  const cv::Mat image = frontend_test::makeCheckerTexture();
  const CameraTrackingResult tracking = makeTrackingResult(
      0U, {{60.0, 60.0}, {120.0, 80.0}, {180.0, 130.0}, {250.0, 180.0}});
  OrbDescriptorExtractor extractor;
  CameraDescriptorSet descriptors;
  DescriptorExtractionStatistics statistics;
  ASSERT_TRUE(extractor.extract(image, tracking, &descriptors, &statistics));
  ASSERT_EQ(4U, descriptors.feature_ids.size());
  EXPECT_EQ(CV_8UC1, descriptors.descriptors.type());
  EXPECT_EQ(32, descriptors.descriptors.cols);
  EXPECT_EQ(4, descriptors.descriptors.rows);
  EXPECT_EQ(descriptors.feature_ids.size(), descriptors.pixels.size());
  EXPECT_EQ(descriptors.pixels.size(), descriptors.bearings_c.size());
  EXPECT_EQ(descriptors.bearings_c.size(), descriptors.bearings_b.size());
  for (std::size_t index = 0U; index < descriptors.feature_ids.size(); ++index) {
    EXPECT_EQ(tracking.tracks[index].id, descriptors.feature_ids[index]);
    EXPECT_TRUE(tracking.tracks[index].current.pixel.isApprox(
        descriptors.pixels[index], 0.0));
  }
}

TEST(OrbDescriptorExtractorTest, OrbDeletionPreservesFeatureIdMapping) {
  const cv::Mat image = frontend_test::makeCheckerTexture();
  const CameraTrackingResult tracking = makeTrackingResult(
      1U, {{-1.0, 80.0}, {8.0, 8.0}, {20.0, 90.0}, {90.0, 90.0},
           {220.0, 150.0}},
      (FeatureId{1} << 48U));
  OrbDescriptorExtractor extractor;
  CameraDescriptorSet descriptors;
  DescriptorExtractionStatistics statistics;
  ASSERT_TRUE(extractor.extract(image, tracking, &descriptors, &statistics));
  EXPECT_EQ(1U, statistics.outside_image_rejections);
  EXPECT_EQ(1U, statistics.patch_boundary_rejections);
  EXPECT_GE(statistics.orb_discarded, 1U);
  ASSERT_EQ(2U, descriptors.feature_ids.size());
  EXPECT_EQ(tracking.tracks[3].id, descriptors.feature_ids[0]);
  EXPECT_EQ(tracking.tracks[4].id, descriptors.feature_ids[1]);
  EXPECT_GT(descriptors.feature_ids[0],
            static_cast<FeatureId>(std::numeric_limits<std::uint32_t>::max()));
}

TEST(OrbDescriptorExtractorTest, EmptyTracksProduceEmptySet) {
  CameraTrackingResult tracking;
  tracking.camera_id = 2U;
  tracking.timestamp = 3.0;
  OrbDescriptorExtractor extractor;
  CameraDescriptorSet descriptors;
  DescriptorExtractionStatistics statistics;
  ASSERT_TRUE(extractor.extract(frontend_test::makeCheckerTexture(), tracking,
                                &descriptors, &statistics));
  EXPECT_TRUE(descriptors.feature_ids.empty());
  EXPECT_TRUE(descriptors.descriptors.empty());
  EXPECT_EQ(0U, statistics.input_tracks);
}

TEST(OrbDescriptorExtractorTest, IdenticalInputIsDeterministic) {
  const cv::Mat image = frontend_test::makeCheckerTexture();
  const CameraTrackingResult tracking = makeTrackingResult(
      3U, {{60.0, 60.0}, {120.0, 80.0}, {180.0, 130.0}});
  OrbDescriptorExtractor extractor;
  CameraDescriptorSet first;
  CameraDescriptorSet second;
  DescriptorExtractionStatistics first_statistics;
  DescriptorExtractionStatistics second_statistics;
  ASSERT_TRUE(extractor.extract(image, tracking, &first, &first_statistics));
  ASSERT_TRUE(extractor.extract(image, tracking, &second, &second_statistics));
  EXPECT_EQ(first.feature_ids, second.feature_ids);
  ASSERT_EQ(first.descriptors.size(), second.descriptors.size());
  EXPECT_EQ(0, cv::countNonZero(first.descriptors != second.descriptors));
}

TEST(OrbDescriptorExtractorTest, AllCameraIdsPreserve64BitFeatureIds) {
  const cv::Mat image = frontend_test::makeCheckerTexture();
  OrbDescriptorExtractor extractor;
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    const FeatureId expected = (FeatureId{1} << 40U) + camera_id * 100U;
    const CameraTrackingResult tracking =
        makeTrackingResult(camera_id, {{80.0, 80.0}}, expected);
    CameraDescriptorSet descriptors;
    DescriptorExtractionStatistics statistics;
    ASSERT_TRUE(
        extractor.extract(image, tracking, &descriptors, &statistics));
    ASSERT_EQ(1U, descriptors.feature_ids.size());
    EXPECT_EQ(expected, descriptors.feature_ids.front());
  }
}

TEST(OrbDescriptorExtractorTest, InvalidInputsFailSafely) {
  const CameraTrackingResult tracking =
      makeTrackingResult(0U, {{80.0, 80.0}});
  OrbDescriptorExtractor extractor;
  CameraDescriptorSet descriptors;
  DescriptorExtractionStatistics statistics;
  EXPECT_FALSE(
      extractor.extract(cv::Mat(), tracking, &descriptors, &statistics));
  EXPECT_FALSE(extractor.extract(cv::Mat(240, 320, CV_8UC3), tracking,
                                 &descriptors, &statistics));
  EXPECT_FALSE(extractor.extract(frontend_test::makeCheckerTexture(), tracking,
                                 nullptr, &statistics));

  CameraTrackingResult duplicate = tracking;
  duplicate.tracks.push_back(duplicate.tracks.front());
  EXPECT_FALSE(extractor.extract(frontend_test::makeCheckerTexture(), duplicate,
                                 &descriptors, &statistics));
}

}  // namespace
}  // namespace sphere_vio
