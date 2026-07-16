#pragma once

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/camera/kannala_brandt.hpp"

namespace sphere_vio {
namespace frontend_test {

inline CameraRig makeRig(int width = 320, int height = 240) {
  CameraRig rig;
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    KannalaBrandt::Parameters parameters;
    parameters.width = width;
    parameters.height = height;
    parameters.fx = 0.5 * width;
    parameters.fy = 0.5 * width;
    parameters.cx = 0.5 * (width - 1);
    parameters.cy = 0.5 * (height - 1);
    RigCamera camera;
    camera.camera_id = camera_id;
    camera.name = "synthetic_frontend_" + std::to_string(camera_id);
    camera.model = std::make_shared<KannalaBrandt>(parameters);
    camera.R_b_c =
        Eigen::AngleAxisd(0.1 * camera_id, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
    const std::array<Eigen::Vector3d, 4> positions{{
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.10, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.10, 0.0),
        Eigen::Vector3d(0.10, 0.10, 0.0)}};
    camera.t_b_c = positions[camera_id];
    if (!rig.addCamera(camera)) return CameraRig{};
  }
  return rig;
}

inline cv::Mat makeCheckerTexture(int width = 320, int height = 240,
                                  int square_size = 16) {
  cv::Mat image(height, width, CV_8UC1);
  cv::RNG random(0x5A17C9U + static_cast<std::uint64_t>(width) * 31U +
                 static_cast<std::uint64_t>(height));
  random.fill(image, cv::RNG::UNIFORM, 15, 241);
  cv::GaussianBlur(image, image, cv::Size(3, 3), 0.65);
  for (int y = 0; y < height; y += square_size) {
    for (int x = 0; x < width; x += square_size) {
      const int parity = (x / square_size + y / square_size) % 2;
      const cv::Point center(
          std::min(width - 1, x + square_size / 2),
          std::min(height - 1, y + square_size / 2));
      cv::circle(image, center, std::max(2, square_size / 6),
                 cv::Scalar(parity ? 235 : 20), cv::FILLED, cv::LINE_8);
    }
  }
  return image;
}

inline cv::Mat translate(const cv::Mat& image, double dx, double dy) {
  cv::Mat translated;
  const cv::Mat transform =
      (cv::Mat_<double>(2, 3) << 1.0, 0.0, dx, 0.0, 1.0, dy);
  cv::warpAffine(image, translated, transform, image.size(), cv::INTER_LINEAR,
                 cv::BORDER_CONSTANT, cv::Scalar(25));
  return translated;
}

inline MultiCameraFrame makeFrame(const cv::Mat& image, Timestamp timestamp,
                                  double per_camera_shift = 0.0) {
  MultiCameraFrame frame;
  frame.timestamp = timestamp;
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    ImageFrame camera_frame;
    camera_frame.timestamp = timestamp;
    camera_frame.camera_id = camera_id;
    camera_frame.image = translate(image, per_camera_shift * camera_id, 0.0);
    frame.images.push_back(std::move(camera_frame));
  }
  return frame;
}

}  // namespace frontend_test
}  // namespace sphere_vio
