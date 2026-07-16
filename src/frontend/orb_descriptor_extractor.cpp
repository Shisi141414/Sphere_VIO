#include "sphere_vio/frontend/orb_descriptor_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace sphere_vio {
namespace {

constexpr int kOrbDescriptorBytes = 32;

bool validOptions(const OrbDescriptorExtractorOptions& options) {
  return options.patch_size > 0 && options.patch_size % 2 == 1 &&
         options.edge_threshold >= 0 && options.levels > 0 &&
         std::isfinite(options.scale_factor) && options.scale_factor > 1.0 &&
         options.fast_threshold >= 0;
}

bool insideImage(const Eigen::Vector2d& pixel, int width, int height) {
  return pixel.allFinite() && pixel.x() >= 0.0 && pixel.y() >= 0.0 &&
         pixel.x() < static_cast<double>(width) &&
         pixel.y() < static_cast<double>(height);
}

bool patchInsideImage(const Eigen::Vector2d& pixel, int width, int height,
                      int half_patch) {
  return pixel.x() >= half_patch && pixel.y() >= half_patch &&
         pixel.x() < width - half_patch && pixel.y() < height - half_patch;
}

}  // namespace

OrbDescriptorExtractor::OrbDescriptorExtractor(
    OrbDescriptorExtractorOptions options)
    : options_(std::move(options)) {
  if (validOptions(options_)) {
    orb_ = cv::ORB::create(1000, static_cast<float>(options_.scale_factor),
                           options_.levels, options_.edge_threshold, 0, 2,
                           cv::ORB::HARRIS_SCORE, options_.patch_size,
                           options_.fast_threshold);
  }
}

bool OrbDescriptorExtractor::extract(
    const cv::Mat& grayscale_image, const CameraTrackingResult& tracking,
    CameraDescriptorSet* descriptors,
    DescriptorExtractionStatistics* statistics) {
  if (!descriptors || !statistics) return false;
  *descriptors = CameraDescriptorSet{};
  *statistics = DescriptorExtractionStatistics{};
  descriptors->camera_id = tracking.camera_id;
  descriptors->timestamp = tracking.timestamp;
  statistics->input_tracks = tracking.tracks.size();

  if (!validOptions(options_) || !orb_ || grayscale_image.empty() ||
      grayscale_image.type() != CV_8UC1 || !std::isfinite(tracking.timestamp)) {
    return false;
  }

  std::set<FeatureId> unique_feature_ids;
  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(tracking.tracks.size());
  const int half_patch = options_.patch_size / 2;
  for (std::size_t index = 0U; index < tracking.tracks.size(); ++index) {
    const FeatureTrack& track = tracking.tracks[index];
    if (track.camera_id != tracking.camera_id ||
        track.current.camera_id != tracking.camera_id ||
        !track.current.bearing_c.allFinite() ||
        !track.current.bearing_b.allFinite() || track.id == 0U ||
        !unique_feature_ids.insert(track.id).second ||
        index > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      return false;
    }
    if (!insideImage(track.current.pixel, grayscale_image.cols,
                     grayscale_image.rows)) {
      ++statistics->outside_image_rejections;
      continue;
    }
    if (!patchInsideImage(track.current.pixel, grayscale_image.cols,
                          grayscale_image.rows, half_patch)) {
      ++statistics->patch_boundary_rejections;
      continue;
    }
    cv::KeyPoint keypoint(
        cv::Point2f(static_cast<float>(track.current.pixel.x()),
                    static_cast<float>(track.current.pixel.y())),
        static_cast<float>(options_.patch_size));
    keypoint.class_id = static_cast<int>(index);
    keypoints.push_back(keypoint);
  }

  const std::size_t submitted_count = keypoints.size();
  cv::Mat computed;
  if (!keypoints.empty()) {
    try {
      orb_->compute(grayscale_image, keypoints, computed);
    } catch (const cv::Exception&) {
      return false;
    }
  }
  statistics->orb_discarded = submitted_count - keypoints.size();
  if (keypoints.empty()) {
    if (!computed.empty()) {
      ++statistics->empty_descriptor_rejections;
      return false;
    }
    return true;
  }
  if (computed.empty() || computed.rows != static_cast<int>(keypoints.size()) ||
      computed.cols != kOrbDescriptorBytes) {
    ++statistics->empty_descriptor_rejections;
    return false;
  }
  if (computed.type() != CV_8UC1) {
    ++statistics->invalid_descriptor_type_rejections;
    return false;
  }

  descriptors->feature_ids.reserve(keypoints.size());
  descriptors->pixels.reserve(keypoints.size());
  descriptors->bearings_c.reserve(keypoints.size());
  descriptors->bearings_b.reserve(keypoints.size());
  descriptors->descriptors = computed.clone();
  std::set<int> returned_indices;
  for (const cv::KeyPoint& keypoint : keypoints) {
    if (keypoint.class_id < 0 ||
        static_cast<std::size_t>(keypoint.class_id) >= tracking.tracks.size() ||
        !returned_indices.insert(keypoint.class_id).second) {
      return false;
    }
    const FeatureTrack& track =
        tracking.tracks[static_cast<std::size_t>(keypoint.class_id)];
    descriptors->feature_ids.push_back(track.id);
    descriptors->pixels.push_back(track.current.pixel);
    descriptors->bearings_c.push_back(track.current.bearing_c);
    descriptors->bearings_b.push_back(track.current.bearing_b);
  }
  statistics->accepted = descriptors->feature_ids.size();
  return descriptors->descriptors.rows ==
             static_cast<int>(descriptors->feature_ids.size()) &&
         descriptors->feature_ids.size() == descriptors->pixels.size() &&
         descriptors->pixels.size() == descriptors->bearings_c.size() &&
         descriptors->bearings_c.size() == descriptors->bearings_b.size();
}

}  // namespace sphere_vio
