#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "sphere_vio/camera/omni_radtan.hpp"
#include "sphere_vio/frontend/omni_rectifier.hpp"

namespace sphere_vio {
namespace {

CameraRig makeOmniRig() {
  CameraRig rig;
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    OmniRadtan::Parameters parameters;
    parameters.width = 640;
    parameters.height = 480;
    parameters.xi = 1.2;
    parameters.fx = 620.0;
    parameters.fy = 615.0;
    parameters.cx = 319.5;
    parameters.cy = 239.5;
    parameters.k1 = -0.05;
    parameters.k2 = 0.08;
    parameters.p1 = 0.002;
    parameters.p2 = -0.0015;

    RigCamera camera;
    camera.camera_id = camera_id;
    camera.name = "rectifier_test_" + std::to_string(camera_id);
    camera.model = std::make_shared<OmniRadtan>(parameters);
    camera.R_b_c =
        Eigen::AngleAxisd(0.05 * camera_id, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
    camera.t_b_c =
        Eigen::Vector3d(0.03 * static_cast<double>(camera_id), 0.0, 0.0);
    if (!rig.addCamera(camera)) return CameraRig{};
  }
  return rig;
}

TEST(OmniRectifierTest, RectifiedRawPixelRoundTrip) {
  const CameraRig rig = makeOmniRig();
  OmniRectifierOptions options;
  options.width = 800;
  options.height = 400;
  options.horizontal_fov = 200.0 * M_PI / 180.0;
  options.vertical_fov = 100.0 * M_PI / 180.0;
  const OmniRectifier rectifier(rig, options);

  std::mt19937 generator(20260911U);
  std::uniform_real_distribution<double> horizontal(0.0, options.width - 1.0);
  std::uniform_real_distribution<double> vertical(0.0, options.height - 1.0);
  std::size_t checked = 0U;
  for (std::size_t sample = 0U; sample < 2000U && checked < 200U; ++sample) {
    const Eigen::Vector2d rectified(horizontal(generator), vertical(generator));
    Eigen::Vector2d raw;
    if (!rectifier.rawPixelFromRectified(0U, rectified, &raw)) continue;
    Eigen::Vector2d back;
    ASSERT_TRUE(rectifier.rectifiedPixelFromRaw(0U, raw, &back));
    EXPECT_LT((back - rectified).norm(), 0.5);
    ++checked;
  }
  // The synthetic camera has a wide visible region; at least some samples
  // must land in the valid mask or the test rig is broken.
  EXPECT_GT(checked, 0U);
}

TEST(OmniRectifierTest, RemapProducesExpectedSizeAndMask) {
  const CameraRig rig = makeOmniRig();
  OmniRectifierOptions options;
  options.width = 320;
  options.height = 160;
  const OmniRectifier rectifier(rig, options);

  cv::Mat raw(480, 640, CV_8UC1, cv::Scalar(128));
  cv::Mat rectified;
  ASSERT_TRUE(rectifier.rectifiedImage(0U, raw, &rectified));
  EXPECT_EQ(rectified.size(), cv::Size(320, 160));
  EXPECT_EQ(rectified.type(), CV_8UC1);

  cv::Mat mask;
  ASSERT_TRUE(rectifier.rectifiedMask(0U, &mask));
  EXPECT_EQ(mask.size(), cv::Size(320, 160));
  EXPECT_EQ(mask.type(), CV_8UC1);
  EXPECT_GT(cv::countNonZero(mask), 0);
}

}  // namespace
}  // namespace sphere_vio
