#include "sphere_vio/panorama/panorama_remapper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "sphere_vio/panorama/uspm.hpp"

namespace sphere_vio {
namespace {

void setError(const std::string& value, std::string* error) {
  if (error) *error = value;
}

bool validOptions(const PanoramaRemapOptions& options) {
  return (options.interpolation == cv::INTER_LINEAR ||
          options.interpolation == cv::INTER_NEAREST) &&
         options.border_mode == cv::BORDER_CONSTANT &&
         std::isfinite(options.sampling_margin) &&
         options.sampling_margin >= 0.0;
}

std::size_t matBytes(const cv::Mat& mat) {
  return mat.empty() ? 0U : mat.total() * mat.elemSize();
}

}  // namespace

bool PanoramaRemapper::initialize(const CameraRig& rig,
                                  const PanoramaSpec& panorama,
                                  const PanoramaRemapOptions& options,
                                  std::string* error) {
  if (error) error->clear();
  initialized_ = false;
  if (!validatePanoramaForRig(rig, panorama)) {
    setError("invalid PanoramaSpec or CameraRig for finite-radius USPM", error);
    return false;
  }
  if (!validOptions(options)) {
    setError("invalid panorama remap options", error);
    return false;
  }
  const auto start = std::chrono::steady_clock::now();
  panorama_ = panorama;
  options_ = options;
  const cv::Size size(panorama.width, panorama.height);
  coverage_count_ = cv::Mat(size, CV_8UC1, cv::Scalar(0));
  owner_camera_id_ = cv::Mat(size, CV_8SC1, cv::Scalar(-1));
  cv::Mat best_score(size, CV_32FC1,
                     cv::Scalar(-std::numeric_limits<float>::infinity()));

  for (CameraId id = 0U; id < 4U; ++id) {
    const RigCamera* camera = rig.camera(id);
    if (!camera) {
      setError("CameraRig must contain CameraId 0 through 3", error);
      return false;
    }
    cameras_[id] = *camera;
    CameraPanoramaRemap& map = camera_remaps_[id];
    map = CameraPanoramaRemap();
    map.camera_id = id;
    map.source_width = camera->model->width();
    map.source_height = camera->model->height();
    map.map_x = cv::Mat(size, CV_32FC1, cv::Scalar(-1.0f));
    map.map_y = cv::Mat(size, CV_32FC1, cv::Scalar(-1.0f));
    map.valid_mask = cv::Mat(size, CV_8UC1, cv::Scalar(0));
    map.owner_score = cv::Mat(
        size, CV_32FC1,
        cv::Scalar(-std::numeric_limits<float>::infinity()));
    for (int y = 0; y < panorama.height; ++y) {
      for (int x = 0; x < panorama.width; ++x) {
        const Eigen::Vector2d panorama_pixel(x + 0.5, y + 0.5);
        Eigen::Vector3d bearing_c;
        Eigen::Vector2d source_pixel;
        if (!panoramaToCameraBearing(*camera, panorama_pixel, panorama,
                                     &bearing_c) ||
            !camera->model->project(bearing_c, &source_pixel) ||
            !source_pixel.allFinite()) continue;
        const double margin = options.sampling_margin;
        if (source_pixel.x() < margin || source_pixel.y() < margin ||
            source_pixel.x() >= map.source_width - margin ||
            source_pixel.y() >= map.source_height - margin ||
            !std::isfinite(bearing_c.z())) continue;
        map.map_x.at<float>(y, x) = static_cast<float>(source_pixel.x());
        map.map_y.at<float>(y, x) = static_cast<float>(source_pixel.y());
        map.valid_mask.at<std::uint8_t>(y, x) = 255U;
        map.owner_score.at<float>(y, x) = static_cast<float>(bearing_c.z());
        ++map.valid_pixel_count;
        ++coverage_count_.at<std::uint8_t>(y, x);
        float& best = best_score.at<float>(y, x);
        std::int8_t& owner = owner_camera_id_.at<std::int8_t>(y, x);
        const float score = static_cast<float>(bearing_c.z());
        if (score > best || (score == best &&
                            (owner < 0 || id < static_cast<CameraId>(owner)))) {
          best = score;
          owner = static_cast<std::int8_t>(id);
        }
      }
    }
  }

  configured_overlaps_.clear();
  const std::array<std::pair<CameraId, CameraId>, 4> pairs{{
      {0U, 1U}, {0U, 2U}, {1U, 3U}, {2U, 3U}}};
  for (const auto& pair : pairs) {
    PairOverlapMask overlap;
    overlap.camera_id_1 = pair.first;
    overlap.camera_id_2 = pair.second;
    cv::bitwise_and(camera_remaps_[pair.first].valid_mask,
                    camera_remaps_[pair.second].valid_mask, overlap.mask);
    overlap.pixel_count = static_cast<std::size_t>(cv::countNonZero(overlap.mask));
    configured_overlaps_.push_back(std::move(overlap));
  }
  precompute_time_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  initialized_ = true;
  return true;
}

bool PanoramaRemapper::remap(const std::array<cv::Mat, 4>& sources,
                             PanoramaRemapResult* result,
                             std::string* error) const {
  if (error) error->clear();
  if (!result || !initialized_) {
    setError(result ? "PanoramaRemapper is not initialized" :
                      "PanoramaRemapResult output is null", error);
    return false;
  }
  for (CameraId id = 0U; id < 4U; ++id) {
    if (sources[id].empty() || sources[id].type() != CV_8UC1 ||
        sources[id].cols != camera_remaps_[id].source_width ||
        sources[id].rows != camera_remaps_[id].source_height) {
      setError("camera C" + std::to_string(id) +
                   " must be a non-empty mono8 image with calibrated size", error);
      return false;
    }
  }
  const auto start = std::chrono::steady_clock::now();
  PanoramaRemapResult output;
  const auto remap_one = [&](CameraId id) {
    PanoramaLayer& layer = output.layers[id];
    layer.camera_id = id;
    layer.valid_mask = camera_remaps_[id].valid_mask;
    cv::remap(sources[id], layer.image, camera_remaps_[id].map_x,
              camera_remaps_[id].map_y, options_.interpolation,
              options_.border_mode, cv::Scalar(0));
    cv::bitwise_and(layer.image, layer.valid_mask, layer.image);
  };
  try {
    if (options_.parallel_cameras) {
      std::array<std::future<void>, 4> futures;
      for (CameraId id = 0U; id < 4U; ++id) {
        futures[id] = std::async(std::launch::async, remap_one, id);
      }
      for (auto& future : futures) future.get();
    } else {
      for (CameraId id = 0U; id < 4U; ++id) remap_one(id);
    }
  } catch (const std::exception& exception) {
    setError(std::string("cv::remap failed: ") + exception.what(), error);
    return false;
  }
  const auto composite_start = std::chrono::steady_clock::now();
  output.owner_selected_composite =
      cv::Mat(panorama_.height, panorama_.width, CV_8UC1, cv::Scalar(0));
  for (CameraId id = 0U; id < 4U; ++id) {
    cv::Mat owner_mask;
    cv::compare(owner_camera_id_, static_cast<int>(id), owner_mask, cv::CMP_EQ);
    output.layers[id].image.copyTo(output.owner_selected_composite, owner_mask);
  }
  output.composite_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - composite_start).count();
  output.coverage_count = coverage_count_;
  output.owner_camera_id = owner_camera_id_;
  output.configured_overlaps = configured_overlaps_;
  output.processing_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  *result = std::move(output);
  return true;
}

std::size_t PanoramaRemapper::staticMapBytes() const {
  std::size_t bytes = matBytes(coverage_count_) + matBytes(owner_camera_id_);
  for (const CameraPanoramaRemap& map : camera_remaps_) {
    bytes += matBytes(map.map_x) + matBytes(map.map_y) +
             matBytes(map.valid_mask) + matBytes(map.owner_score);
  }
  for (const PairOverlapMask& overlap : configured_overlaps_) {
    bytes += matBytes(overlap.mask);
  }
  return bytes;
}

}  // namespace sphere_vio
