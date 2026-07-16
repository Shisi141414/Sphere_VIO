#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "sphere_vio/camera/omni_radtan.hpp"
#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/panorama/panorama_remapper.hpp"
#include "sphere_vio/panorama/uspm.hpp"

namespace sphere_vio {
namespace {

CameraRig makeIdenticalRig() {
  OmniRadtan::Parameters p;
  p.width = 64; p.height = 48; p.xi = 1.0;
  p.fx = 24; p.fy = 24; p.cx = 31.5; p.cy = 23.5;
  CameraRig rig;
  for (CameraId id = 0; id < 4; ++id) {
    RigCamera camera;
    camera.camera_id = id;
    camera.name = "C" + std::to_string(id);
    camera.model = std::make_shared<OmniRadtan>(p);
    EXPECT_TRUE(rig.addCamera(camera));
  }
  return rig;
}

std::array<cv::Mat, 4> makeImages(int width, int height) {
  std::array<cv::Mat, 4> images;
  for (int id = 0; id < 4; ++id) {
    images[id] = cv::Mat(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        images[id].at<std::uint8_t>(y, x) =
            static_cast<std::uint8_t>((id * 53 + x * 3 + y * 5) % 255 + 1);
      }
    }
  }
  return images;
}

TEST(PanoramaRemapper, StaticMapsUsePixelCentersMasksCoverageAndTieBreak) {
  const CameraRig rig = makeIdenticalRig();
  PanoramaSpec panorama;
  panorama.width = 80; panorama.height = 40;
  PanoramaRemapper remapper;
  ASSERT_TRUE(remapper.initialize(rig, panorama, PanoramaRemapOptions()));
  ASSERT_TRUE(remapper.initialized());
  for (const CameraPanoramaRemap& map : remapper.cameraRemaps()) {
    EXPECT_EQ(map.map_x.type(), CV_32FC1);
    EXPECT_EQ(map.map_y.type(), CV_32FC1);
    EXPECT_EQ(map.valid_mask.type(), CV_8UC1);
    EXPECT_EQ(map.owner_score.type(), CV_32FC1);
    EXPECT_EQ(map.map_x.size(), cv::Size(80, 40));
    EXPECT_GT(map.valid_pixel_count, 0U);
    EXPECT_EQ(cv::countNonZero(map.valid_mask),
              static_cast<int>(map.valid_pixel_count));
  }
  double minimum = 0, maximum = 0;
  cv::minMaxLoc(remapper.coverageCount(), &minimum, &maximum);
  EXPECT_GE(minimum, 0); EXPECT_LE(maximum, 4);
  EXPECT_EQ(remapper.configuredOverlaps().size(), 4U);
  for (const PairOverlapMask& overlap : remapper.configuredOverlaps()) {
    EXPECT_EQ(overlap.pixel_count,
              remapper.cameraRemaps()[0].valid_pixel_count);
  }
  for (int y = 0; y < panorama.height; ++y) {
    for (int x = 0; x < panorama.width; ++x) {
      const int owner = remapper.ownerCameraId().at<std::int8_t>(y, x);
      EXPECT_TRUE(owner == -1 || owner == 0);  // exact score tie -> lower id
    }
  }
  // Center convention: the stored map equals explicit Phase 5A evaluation at
  // panorama coordinate (x+0.5,y+0.5), not integer corner coordinates.
  const int x = 40, y = 20;
  if (remapper.cameraRemaps()[0].valid_mask.at<std::uint8_t>(y, x)) {
    Eigen::Vector2d expected;
    ASSERT_TRUE(panoramaToCameraPixel(*rig.camera(0), {x + 0.5, y + 0.5},
                                     panorama, &expected));
    EXPECT_NEAR(remapper.cameraRemaps()[0].map_x.at<float>(y, x), expected.x(),
                1e-5);
    EXPECT_NEAR(remapper.cameraRemaps()[0].map_y.at<float>(y, x), expected.y(),
                1e-5);
  }
}

TEST(PanoramaRemapper, ValidatesInputsMarginAndDoesNotModifySources) {
  const CameraRig rig = makeIdenticalRig();
  PanoramaSpec panorama; panorama.width = 80; panorama.height = 40;
  PanoramaRemapOptions options;
  PanoramaRemapper remapper;
  options.sampling_margin = -1;
  EXPECT_FALSE(remapper.initialize(rig, panorama, options));
  options.sampling_margin = 1;
  ASSERT_TRUE(remapper.initialize(rig, panorama, options));
  auto images = makeImages(64, 48);
  const cv::Mat before = images[0].clone();
  PanoramaRemapResult result;
  EXPECT_FALSE(remapper.remap(images, nullptr));
  auto invalid = images; invalid[0] = cv::Mat();
  EXPECT_FALSE(remapper.remap(invalid, &result));
  invalid = images; invalid[0] = cv::Mat(48, 64, CV_8UC3);
  EXPECT_FALSE(remapper.remap(invalid, &result));
  invalid = images; invalid[0] = cv::Mat(47, 64, CV_8UC1);
  EXPECT_FALSE(remapper.remap(invalid, &result));
  ASSERT_TRUE(remapper.remap(images, &result));
  EXPECT_EQ(cv::countNonZero(before != images[0]), 0);
  for (const PanoramaLayer& layer : result.layers) {
    EXPECT_EQ(layer.image.size(), cv::Size(80, 40));
    cv::Mat outside, nonzero;
    cv::bitwise_not(layer.valid_mask, outside);
    cv::bitwise_and(layer.image, outside, nonzero);
    EXPECT_EQ(cv::countNonZero(nonzero), 0);
  }
}

TEST(PanoramaRemapper, SequentialAndParallelArePixelIdentical) {
  const CameraRig rig = makeIdenticalRig();
  PanoramaSpec panorama; panorama.width = 80; panorama.height = 40;
  PanoramaRemapOptions sequential_options;
  sequential_options.parallel_cameras = false;
  PanoramaRemapOptions parallel_options = sequential_options;
  parallel_options.parallel_cameras = true;
  PanoramaRemapper sequential, parallel;
  ASSERT_TRUE(sequential.initialize(rig, panorama, sequential_options));
  ASSERT_TRUE(parallel.initialize(rig, panorama, parallel_options));
  const auto images = makeImages(64, 48);
  PanoramaRemapResult a, b;
  ASSERT_TRUE(sequential.remap(images, &a));
  ASSERT_TRUE(parallel.remap(images, &b));
  for (int id = 0; id < 4; ++id) {
    EXPECT_EQ(cv::countNonZero(a.layers[id].image != b.layers[id].image), 0);
  }
  EXPECT_EQ(cv::countNonZero(a.owner_selected_composite !=
                             b.owner_selected_composite), 0);
  for (int y = 0; y < panorama.height; ++y) {
    for (int x = 0; x < panorama.width; ++x) {
      const int owner = a.owner_camera_id.at<std::int8_t>(y, x);
      const std::uint8_t composite =
          a.owner_selected_composite.at<std::uint8_t>(y, x);
      EXPECT_EQ(composite, owner < 0 ? 0U :
                a.layers[owner].image.at<std::uint8_t>(y, x));
    }
  }
}

TEST(PanoramaRemapper, RealRigDenseMapsAreFiniteOrExplicitlyInvalid) {
  CameraRig rig; std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(
      std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml", &rig, &error));
  PanoramaSpec panorama; panorama.width = 128; panorama.height = 64;
  PanoramaRemapper remapper;
  ASSERT_TRUE(remapper.initialize(rig, panorama, PanoramaRemapOptions(), &error))
      << error;
  for (const CameraPanoramaRemap& map : remapper.cameraRemaps()) {
    for (int y = 0; y < panorama.height; ++y) {
      for (int x = 0; x < panorama.width; ++x) {
        const bool valid = map.valid_mask.at<std::uint8_t>(y, x) != 0;
        const float mx = map.map_x.at<float>(y, x);
        const float my = map.map_y.at<float>(y, x);
        EXPECT_TRUE(valid ? (std::isfinite(mx) && std::isfinite(my))
                          : (mx == -1.0f && my == -1.0f));
      }
    }
  }
  EXPECT_GT(remapper.staticMapBytes(), 0U);
}

}  // namespace
}  // namespace sphere_vio
