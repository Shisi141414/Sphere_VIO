#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "sphere_vio/camera/omni_radtan.hpp"
#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/geometry/spherical_geometry.hpp"
#include "sphere_vio/panorama/uspm.hpp"

namespace sphere_vio {
namespace {

constexpr double kPi = 3.14159265358979323846;

RigCamera makeCamera(const Eigen::Vector3d& center = Eigen::Vector3d::Zero()) {
  OmniRadtan::Parameters p;
  p.width = 640; p.height = 480; p.xi = 1.0;
  p.fx = 300.0; p.fy = 300.0; p.cx = 320.0; p.cy = 240.0;
  RigCamera camera;
  camera.name = "synthetic";
  camera.model = std::make_shared<OmniRadtan>(p);
  camera.t_b_c = center;
  return camera;
}

double wrappedPixelDifference(double a, double b, double width) {
  double d = std::abs(a - b);
  return std::min(d, width - d);
}

TEST(UspmSpec, RejectsInvalidInputs) {
  PanoramaSpec p;
  EXPECT_TRUE(validatePanoramaSpec(p));
  p.width = 0; EXPECT_FALSE(validatePanoramaSpec(p)); p.width = 2048;
  p.sphere_radius = 0.0; EXPECT_FALSE(validatePanoramaSpec(p)); p.sphere_radius = 1.0;
  p.horizontal_fov = 2.0 * kPi + 0.01; EXPECT_FALSE(validatePanoramaSpec(p));
  p.horizontal_fov = 2.0 * kPi;
  p.frame.R_p_b(0, 0) = 2.0; EXPECT_FALSE(validatePanoramaSpec(p));
  p.frame.R_p_b.setIdentity();
  p.frame.t_p_b.x() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validatePanoramaSpec(p));
}

TEST(UspmGeometry, AnalyticIntersectionAndNonUnitInput) {
  const RigCamera camera = makeCamera(Eigen::Vector3d(0.2, 0.0, 0.0));
  PanoramaSpec p;
  Eigen::Vector3d point, bearing;
  ASSERT_TRUE(cameraBearingToSphere(camera, Eigen::Vector3d(0.0, 0.0, 5.0),
                                    p, &point, &bearing));
  EXPECT_NEAR(point.x(), 0.2, 1e-14);
  EXPECT_NEAR(point.z(), std::sqrt(0.96), 1e-14);
  EXPECT_NEAR(point.norm(), 1.0, 1e-14);
  EXPECT_FALSE(cameraBearingToSphere(camera, Eigen::Vector3d::Zero(), p,
                                     &point, &bearing));
  EXPECT_FALSE(cameraBearingToSphere(camera,
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 1), p,
      &point, &bearing));
  EXPECT_FALSE(cameraBearingToSphere(camera, Eigen::Vector3d::UnitZ(), p,
                                     nullptr, &bearing));
}

TEST(UspmGeometry, CameraOutsideSphereIsRejected) {
  PanoramaSpec p;
  const RigCamera outside = makeCamera(Eigen::Vector3d(1.1, 0, 0));
  EXPECT_FALSE(validatePanoramaForCamera(outside, p));
  Eigen::Vector3d point, bearing;
  EXPECT_FALSE(cameraBearingToSphere(outside, -Eigen::Vector3d::UnitX(), p,
                                     &point, &bearing));
}

TEST(UspmAngles, RoundTripSeamPolesAndPartialFov) {
  PanoramaSpec p;
  const std::vector<Eigen::Vector3d> bearings = {
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(),
      -Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(),
      -Eigen::Vector3d::UnitY(), Eigen::Vector3d(1e-10, 0, -1)};
  for (const Eigen::Vector3d& input : bearings) {
    Eigen::Vector2d uv;
    Eigen::Vector3d output;
    ASSERT_TRUE(sphereBearingToPanorama(input, p, &uv));
    ASSERT_TRUE(panoramaToSphereBearing(uv, p, &output));
    EXPECT_NEAR(input.normalized().dot(output), 1.0, 2e-15);
  }
  Eigen::Vector3d a, b;
  ASSERT_TRUE(panoramaToSphereBearing(Eigen::Vector2d(0, 512), p, &a));
  ASSERT_TRUE(panoramaToSphereBearing(Eigen::Vector2d(2048, 512), p, &b));
  EXPECT_NEAR((a - b).norm(), 0.0, 1e-15);
  p.horizontal_fov = kPi;
  Eigen::Vector2d uv;
  EXPECT_FALSE(sphereBearingToPanorama(-Eigen::Vector3d::UnitZ(), p, &uv));
}

TEST(UspmGeometry, SameCameraBearingAndPixelRoundTrip) {
  const RigCamera camera = makeCamera(Eigen::Vector3d(0.08, -0.03, 0.02));
  PanoramaSpec p;
  const std::vector<Eigen::Vector2d> pixels = {
      {320, 240}, {100, 100}, {540, 100}, {100, 380}, {540, 380}};
  for (const Eigen::Vector2d& pixel : pixels) {
    Eigen::Vector3d original;
    Eigen::Vector2d uv, recovered_pixel;
    ASSERT_TRUE(camera.model->unproject(pixel, &original));
    ASSERT_TRUE(cameraPixelToPanorama(camera, pixel, p, &uv));
    Eigen::Vector3d recovered;
    ASSERT_TRUE(panoramaToCameraBearing(camera, uv, p, &recovered));
    EXPECT_NEAR(original.dot(recovered), 1.0, 2e-14);
    ASSERT_TRUE(panoramaToCameraPixel(camera, uv, p, &recovered_pixel));
    EXPECT_LT((pixel - recovered_pixel).norm(), 1e-9);
  }
  Eigen::Vector2d pixel;
  EXPECT_FALSE(panoramaToCameraPixel(camera, Eigen::Vector2d(1024, 1024), p,
                                     &pixel));
}

TEST(UspmRealRig, FourCameraRoundTripCoverageAndRadiusSweep) {
  CameraRig rig;
  std::string error;
  ASSERT_TRUE(loadCameraRigFromYaml(
      std::string(SPHERE_VIO_SOURCE_DIR) + "/config/cameras.yaml", &rig, &error))
      << error;
  PanoramaSpec p;
  ASSERT_TRUE(validatePanoramaForRig(rig, p));
  const std::vector<double> radii = {0.10, 0.25, 0.50, 1.0, 2.0, 10.0, 100.0};
  for (CameraId id = 0; id < 4; ++id) {
    const RigCamera* camera = rig.camera(id);
    ASSERT_NE(camera, nullptr);
    int valid = 0;
    double max_pixel_error = 0.0, sum_pixel_error = 0.0;
    double min_u = p.width, max_u = 0, min_v = p.height, max_v = 0;
    for (int y = 20; y < camera->model->height(); y += 70) {
      for (int x = 20; x < camera->model->width(); x += 70) {
        const Eigen::Vector2d input(x, y);
        Eigen::Vector2d uv, output;
        if (!cameraPixelToPanorama(*camera, input, p, &uv)) continue;
        ASSERT_TRUE(panoramaToCameraPixel(*camera, uv, p, &output));
        const double error_px = (input - output).norm();
        max_pixel_error = std::max(max_pixel_error, error_px);
        sum_pixel_error += error_px; ++valid;
        min_u = std::min(min_u, uv.x()); max_u = std::max(max_u, uv.x());
        min_v = std::min(min_v, uv.y()); max_v = std::max(max_v, uv.y());
      }
    }
    ASSERT_GT(valid, 0);
    EXPECT_LT(max_pixel_error, 1e-7);
    Eigen::Vector2d center(camera->model->width() * 0.5,
                           camera->model->height() * 0.5), center_uv;
    Eigen::Vector3d center_n;
    ASSERT_TRUE(cameraPixelToPanorama(*camera, center, p, &center_uv, &center_n));
    std::cout << "USPM C" << static_cast<int>(id) << " " << camera->name
              << " samples=" << valid << " pixel_error(max/mean)="
              << max_pixel_error << "/" << sum_pixel_error / valid
              << " coverage_u=[" << min_u << ',' << max_u << "] v=["
              << min_v << ',' << max_v << "] center_n="
              << center_n.transpose() << " center_uv=" << center_uv.transpose()
              << '\n';

    Eigen::Vector3d bearing_c, bearing_b;
    ASSERT_TRUE(camera->model->unproject(center, &bearing_c));
    ASSERT_TRUE(rig.cameraBearingToBody(id, bearing_c, &bearing_b));
    Eigen::Vector2d old_erp;
    ASSERT_TRUE(bearingToEquirectangular(bearing_b, p.width, p.height, &old_erp));
    double previous = std::numeric_limits<double>::infinity();
    for (double radius : radii) {
      p.sphere_radius = radius;
      ASSERT_TRUE(validatePanoramaForCamera(*camera, p));
      Eigen::Vector2d uspm;
      ASSERT_TRUE(cameraBearingToPanorama(*camera, bearing_c, p, &uspm));
      // Compare finite sphere direction to its zero-translation USPM limit;
      // old ERP uses a deliberately different documented axis convention.
      RigCamera centered = *camera; centered.t_b_c.setZero();
      Eigen::Vector2d limit;
      ASSERT_TRUE(cameraBearingToPanorama(centered, bearing_c, p, &limit));
      const double difference = std::hypot(
          wrappedPixelDifference(uspm.x(), limit.x(), p.width),
          uspm.y() - limit.y());
      EXPECT_LE(difference, previous + 1e-9);
      previous = difference;
      std::cout << "USPM radius C" << static_cast<int>(id) << " r=" << radius
                << " finite_to_direction_px=" << difference
                << " legacy_erp=" << old_erp.transpose() << '\n';
    }
    p.sphere_radius = 1.0;
  }

  const std::vector<std::pair<CameraId, CameraId>> pairs =
      {{0, 1}, {0, 2}, {1, 3}, {2, 3}};
  for (const auto& pair : pairs) {
    int common = 0, total = 0, seam = 0;
    for (int v = 0; v <= p.height; v += 16) {
      for (int u = 0; u < p.width; u += 16) {
        ++total; Eigen::Vector2d a, b;
        if (panoramaToCameraPixel(*rig.camera(pair.first), {double(u), double(v)}, p, &a) &&
            panoramaToCameraPixel(*rig.camera(pair.second), {double(u), double(v)}, p, &b)) {
          ++common; if (u < 64 || u >= p.width - 64) ++seam;
        }
      }
    }
    EXPECT_GT(common, 0);
    std::cout << "USPM overlap C" << static_cast<int>(pair.first) << "-C"
              << static_cast<int>(pair.second) << " common=" << common << '/'
              << total << " ratio=" << double(common) / total
              << " seam=" << seam << '\n';
  }
}

}  // namespace
}  // namespace sphere_vio
