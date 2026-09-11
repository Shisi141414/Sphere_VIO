#include "sphere_vio/frontend/omni_rectifier.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace sphere_vio {
namespace {

bool validRectifiedPixel(const OmniRectifierOptions& options,
                         const Eigen::Vector2d& pixel) {
  return pixel.allFinite() && pixel.x() >= 0.0 && pixel.y() >= 0.0 &&
         pixel.x() < static_cast<double>(options.width) &&
         pixel.y() < static_cast<double>(options.height);
}

// Equirectangular pixel-to-direction mapping. u is azimuth in [-HFOV/2,
// +HFOV/2], v is elevation in [-VFOV/2, +VFOV/2]. Forward is the camera +z
// axis; +x points right and +y points down, matching OpenCV's pixel axes.
Eigen::Vector3d bearingFromRectified(const OmniRectifierOptions& options,
                                     const Eigen::Vector2d& rectified) {
  const double azimuth =
      ((rectified.x() + 0.5) / static_cast<double>(options.width) - 0.5) *
      options.horizontal_fov;
  const double elevation =
      (0.5 - (rectified.y() + 0.5) /
                 static_cast<double>(options.height)) *
      options.vertical_fov;
  const double cos_elevation = std::cos(elevation);
  return Eigen::Vector3d(cos_elevation * std::sin(azimuth),
                         std::sin(elevation),
                         cos_elevation * std::cos(azimuth));
}

bool rectifiedFromBearing(const OmniRectifierOptions& options,
                          const Eigen::Vector3d& bearing,
                          Eigen::Vector2d* rectified) {
  const double norm = bearing.norm();
  if (!rectified || !bearing.allFinite() || norm <= 1e-12) return false;
  const double azimuth = std::atan2(bearing.x(), bearing.z());
  const double elevation = std::asin(
      std::max(-1.0, std::min(1.0, bearing.y() / norm)));
  const double u =
      (azimuth / options.horizontal_fov + 0.5) * options.width - 0.5;
  const double v =
      (0.5 - elevation / options.vertical_fov) * options.height - 0.5;
  *rectified = Eigen::Vector2d(u, v);
  return validRectifiedPixel(options, *rectified);
}

}  // namespace

OmniRectifier::OmniRectifier(const CameraRig& rig, OmniRectifierOptions options)
    : rig_(rig), options_(options) {
  if (options_.width <= 0 || options_.height <= 0 ||
      options_.horizontal_fov <= 0.0 || options_.vertical_fov <= 0.0 ||
      options_.horizontal_fov > 2.0 * M_PI ||
      options_.vertical_fov > M_PI) {
    throw std::invalid_argument("invalid omni rectifier options");
  }

  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    const RigCamera* camera = rig_.camera(camera_id);
    if (!camera || !camera->model) continue;

    CameraMap& map = camera_maps_[camera_id];
    map.map_x.create(options_.height, options_.width, CV_32FC1);
    map.map_y.create(options_.height, options_.width, CV_32FC1);
    map.mask.create(options_.height, options_.width, CV_8UC1);
    map.mask.setTo(0);

    for (int row = 0; row < options_.height; ++row) {
      float* map_x = map.map_x.ptr<float>(row);
      float* map_y = map.map_y.ptr<float>(row);
      std::uint8_t* mask = map.mask.ptr<std::uint8_t>(row);
      for (int column = 0; column < options_.width; ++column) {
        const Eigen::Vector2d rectified(column, row);
        const Eigen::Vector3d bearing =
            bearingFromRectified(options_, rectified);
        Eigen::Vector2d raw;
        if (camera->model->projectBearing(bearing, &raw)) {
          map_x[column] = static_cast<float>(raw.x());
          map_y[column] = static_cast<float>(raw.y());
          mask[column] = 255U;
        } else {
          // Invalid texels deliberately read a harmless in-bounds pixel while
          // the mask marks them, keeping the two maps finite everywhere.
          map_x[column] = 0.0F;
          map_y[column] = 0.0F;
        }
      }
    }
  }
}

bool OmniRectifier::rectifiedImage(CameraId camera_id,
                                   const cv::Mat& raw_image,
                                   cv::Mat* rectified) const {
  if (!rectified || camera_id >= 4U || raw_image.empty() ||
      raw_image.type() != CV_8UC1 || camera_maps_[camera_id].map_x.empty()) {
    return false;
  }
  cv::remap(raw_image, *rectified, camera_maps_[camera_id].map_x,
            camera_maps_[camera_id].map_y, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0));
  return !rectified->empty() && rectified->size() ==
      cv::Size(options_.width, options_.height);
}

bool OmniRectifier::rectifiedMask(CameraId camera_id, cv::Mat* mask) const {
  if (!mask || camera_id >= 4U || camera_maps_[camera_id].mask.empty()) {
    return false;
  }
  *mask = camera_maps_[camera_id].mask.clone();
  return true;
}

bool OmniRectifier::rawPixelFromRectified(CameraId camera_id,
                                          const Eigen::Vector2d& rectified,
                                          Eigen::Vector2d* raw) const {
  if (!raw || camera_id >= 4U ||
      !validRectifiedPixel(options_, rectified)) {
    return false;
  }
  const int column = static_cast<int>(std::floor(rectified.x()));
  const int row = static_cast<int>(std::floor(rectified.y()));
  if (row < 0 || row >= options_.height || column < 0 ||
      column >= options_.width ||
      camera_maps_[camera_id].mask.at<std::uint8_t>(row, column) == 0U) {
    return false;
  }
  // Use bilinear interpolation of the precomputed map so sub-pixel LK results
  // map back smoothly rather than snapping to the nearest texel.
  const float x0 = std::floor(rectified.x());
  const float y0 = std::floor(rectified.y());
  const float tx = static_cast<float>(rectified.x()) - x0;
  const float ty = static_cast<float>(rectified.y()) - y0;
  const int column_1 = std::min(column + 1, options_.width - 1);
  const int row_1 = std::min(row + 1, options_.height - 1);
  const auto sample = [&](int sample_row, int sample_column) {
    return Eigen::Vector2d(
        camera_maps_[camera_id].map_x.at<float>(sample_row, sample_column),
        camera_maps_[camera_id].map_y.at<float>(sample_row, sample_column));
  };
  const Eigen::Vector2d top =
      (1.0F - tx) * sample(row, column) + tx * sample(row, column_1);
  const Eigen::Vector2d bottom =
      (1.0F - tx) * sample(row_1, column) + tx * sample(row_1, column_1);
  *raw = (1.0F - ty) * top + ty * bottom;
  return raw->allFinite();
}

bool OmniRectifier::rectifiedPixelFromRaw(CameraId camera_id,
                                          const Eigen::Vector2d& raw,
                                          Eigen::Vector2d* rectified) const {
  if (!rectified || camera_id >= 4U) return false;
  const RigCamera* camera = rig_.camera(camera_id);
  if (!camera || !camera->model) return false;
  Eigen::Vector3d bearing;
  if (!camera->model->unprojectToBearing(raw, &bearing)) return false;
  return rectifiedFromBearing(options_, bearing, rectified);
}

}  // namespace sphere_vio
