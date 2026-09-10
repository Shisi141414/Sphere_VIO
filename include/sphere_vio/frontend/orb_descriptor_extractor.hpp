#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "sphere_vio/frontend/temporal_frontend.hpp"

namespace sphere_vio {

struct OrbDescriptorExtractorOptions {
  int patch_size = 31;
  int edge_threshold = 31;
  int levels = 4;
  double scale_factor = 1.2;
  int fast_threshold = 20;
  std::size_t maximum_descriptors = 180U;
};

struct DescriptorExtractionStatistics {
  std::size_t input_tracks = 0U;
  std::size_t outside_image_rejections = 0U;
  std::size_t patch_boundary_rejections = 0U;
  std::size_t orb_discarded = 0U;
  std::size_t empty_descriptor_rejections = 0U;
  std::size_t invalid_descriptor_type_rejections = 0U;
  std::size_t descriptor_cap_rejections = 0U;
  std::size_t accepted = 0U;
};

struct CameraDescriptorSet {
  CameraId camera_id = 0U;
  Timestamp timestamp = 0.0;
  std::vector<FeatureId> feature_ids;
  std::vector<Eigen::Vector2d> pixels;
  std::vector<Eigen::Vector3d> bearings_c;
  std::vector<Eigen::Vector3d> bearings_b;
  cv::Mat descriptors;
};

class OrbDescriptorExtractor {
 public:
  explicit OrbDescriptorExtractor(OrbDescriptorExtractorOptions options = {});

  const OrbDescriptorExtractorOptions& options() const { return options_; }

  bool extract(const cv::Mat& grayscale_image,
               const CameraTrackingResult& tracking,
               CameraDescriptorSet* descriptors,
               DescriptorExtractionStatistics* statistics);

 private:
  OrbDescriptorExtractorOptions options_;
  cv::Ptr<cv::ORB> orb_;
};

}  // namespace sphere_vio
