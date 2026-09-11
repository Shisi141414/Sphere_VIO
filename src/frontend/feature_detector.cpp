#include "sphere_vio/frontend/feature_detector.hpp"

#include "sphere_vio/frontend/omni_rectifier.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <opencv2/features2d.hpp>

namespace sphere_vio {
namespace {

bool validOptions(const FeatureDetectorOptions& options) {
  return options.maximum_features > 0U && options.grid_rows > 0 &&
         options.grid_columns > 0 && options.fast_threshold >= 0 &&
         std::isfinite(options.minimum_feature_distance) &&
         options.minimum_feature_distance >= 0.0 &&
         options.border_margin >= 0;
}

bool insideBorder(const cv::Point2f& point, int width, int height,
                  int margin) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         point.x >= static_cast<float>(margin) &&
         point.y >= static_cast<float>(margin) &&
         point.x < static_cast<float>(width - margin) &&
         point.y < static_cast<float>(height - margin);
}

bool sufficientlyFar(const Eigen::Vector2d& pixel,
                     const std::vector<Eigen::Vector2d>& occupied,
                     double minimum_squared_distance) {
  for (const Eigen::Vector2d& other : occupied) {
    if (!other.allFinite() ||
        (pixel - other).squaredNorm() < minimum_squared_distance) {
      return false;
    }
  }
  return true;
}

}  // namespace

FeatureDetector::FeatureDetector(FeatureDetectorOptions options)
    : options_(std::move(options)) {}

void FeatureDetector::setMaximumFeatures(std::size_t maximum_features) {
  if (maximum_features > 0U) options_.maximum_features = maximum_features;
}

bool FeatureDetector::detect(
    const cv::Mat& grayscale_image, Timestamp timestamp, CameraId camera_id,
    const CameraRig& camera_rig,
    const std::vector<Eigen::Vector2d>& occupied_pixels,
    std::size_t maximum_new_features,
    std::vector<FeatureObservation>* observations,
    FeatureDetectionStatistics* statistics,
    const OmniRectifier* rectifier) const {
  if (!observations || !statistics) return false;
  observations->clear();
  *statistics = FeatureDetectionStatistics{};

  const RigCamera* rig_camera = camera_rig.camera(camera_id);
  const bool rectified_mode = rectifier != nullptr;
  if (!validOptions(options_) || grayscale_image.empty() ||
      grayscale_image.type() != CV_8UC1 || !std::isfinite(timestamp) ||
      !rig_camera || grayscale_image.cols <= 2 * options_.border_margin ||
      grayscale_image.rows <= 2 * options_.border_margin) {
    return false;
  }
  if (rectified_mode &&
      (grayscale_image.cols != rectifier->width() ||
       grayscale_image.rows != rectifier->height())) {
    return false;
  }
  // In rectified mode the occupied list is expressed in the rectified canvas
  // because the grid, distance checks, and FAST all operate there. In raw
  // mode it is the ordinary fisheye pixel list.
  for (const Eigen::Vector2d& occupied : occupied_pixels) {
    if (!occupied.allFinite()) return false;
  }

  // 校正模式里镜头视野之外的区域在校正图中是纯黑填充。让 FAST 带上有效掩膜，
  // 避免在“有效像素/黑边”交界处检出一堆高分假角点；这些点即使后续被
  // rawPixelFromRectified 拦下，也会白白占用网格配额。
  cv::Mat rectified_mask;
  if (rectified_mode &&
      !rectifier->rectifiedMask(camera_id, &rectified_mask)) {
    return false;
  }

  const std::size_t feature_limit =
      std::min(maximum_new_features, options_.maximum_features);
  if (feature_limit == 0U) return true;

  const int cell_count = options_.grid_rows * options_.grid_columns;
  std::vector<std::vector<cv::KeyPoint>> cells(
      static_cast<std::size_t>(cell_count));
  for (int row = 0; row < options_.grid_rows; ++row) {
    const int y0 = row * grayscale_image.rows / options_.grid_rows;
    const int y1 = (row + 1) * grayscale_image.rows / options_.grid_rows;
    for (int column = 0; column < options_.grid_columns; ++column) {
      const int x0 = column * grayscale_image.cols / options_.grid_columns;
      const int x1 =
          (column + 1) * grayscale_image.cols / options_.grid_columns;
      if (x1 <= x0 || y1 <= y0) continue;

      std::vector<cv::KeyPoint>& keypoints =
          cells[static_cast<std::size_t>(row * options_.grid_columns +
                                         column)];
      cv::FAST(grayscale_image(cv::Rect(x0, y0, x1 - x0, y1 - y0)),
               keypoints, options_.fast_threshold,
               options_.fast_nonmax_suppression);
      for (cv::KeyPoint& keypoint : keypoints) {
        keypoint.pt.x += static_cast<float>(x0);
        keypoint.pt.y += static_cast<float>(y0);
      }
      // OpenCV 的自由函数 cv::FAST 不接受掩膜参数，因此先检测再按掩膜过滤：
      // 校正模式下丢掉落在有效视野外（黑边）的角点，避免占用网格配额。
      if (rectified_mode) {
        keypoints.erase(
            std::remove_if(
                keypoints.begin(), keypoints.end(),
                [&rectified_mask](const cv::KeyPoint& keypoint) {
                  const int column = static_cast<int>(keypoint.pt.x);
                  const int row = static_cast<int>(keypoint.pt.y);
                  if (row < 0 || row >= rectified_mask.rows || column < 0 ||
                      column >= rectified_mask.cols) {
                    return true;
                  }
                  return rectified_mask.at<std::uint8_t>(row, column) == 0U;
                }),
            keypoints.end());
      }
      statistics->candidate_count += keypoints.size();
      std::stable_sort(
          keypoints.begin(), keypoints.end(),
          [](const cv::KeyPoint& lhs, const cv::KeyPoint& rhs) {
            if (lhs.response != rhs.response)
              return lhs.response > rhs.response;
            if (lhs.pt.y != rhs.pt.y) return lhs.pt.y < rhs.pt.y;
            return lhs.pt.x < rhs.pt.x;
          });
    }
  }

  std::vector<Eigen::Vector2d> occupied = occupied_pixels;
  occupied.reserve(occupied.size() + feature_limit);
  const double minimum_squared_distance =
      options_.minimum_feature_distance * options_.minimum_feature_distance;
  const std::size_t maximum_per_cell =
      (options_.maximum_features + cells.size() - 1U) / cells.size();
  std::vector<std::size_t> accepted_per_cell(cells.size(), 0U);

  std::size_t rank = 0U;
  bool had_candidate = true;
  while (observations->size() < feature_limit && had_candidate) {
    had_candidate = false;
    for (std::size_t cell_index = 0U; cell_index < cells.size();
         ++cell_index) {
      const std::vector<cv::KeyPoint>& cell = cells[cell_index];
      if (accepted_per_cell[cell_index] >= maximum_per_cell ||
          rank >= cell.size()) {
        continue;
      }
      had_candidate = true;
      const cv::Point2f point = cell[rank].pt;
      if (!insideBorder(point, grayscale_image.cols, grayscale_image.rows,
                        options_.border_margin)) {
        ++statistics->border_rejections;
        continue;
      }

      const Eigen::Vector2d working_pixel(static_cast<double>(point.x),
                                          static_cast<double>(point.y));
      Eigen::Vector2d raw_pixel = working_pixel;
      if (rectified_mode &&
          !rectifier->rawPixelFromRectified(camera_id, working_pixel,
                                            &raw_pixel)) {
        continue;
      }
      if (!sufficientlyFar(working_pixel, occupied,
                           minimum_squared_distance)) {
        ++statistics->distance_rejections;
        continue;
      }

      FeatureObservation observation;
      observation.timestamp = timestamp;
      observation.camera_id = camera_id;
      observation.pixel = raw_pixel;
      observation.rectified_pixel =
          rectified_mode ? working_pixel : Eigen::Vector2d::Zero();
      if (!rig_camera->model->unproject(raw_pixel, &observation.bearing_c) ||
          !camera_rig.cameraBearingToBody(camera_id, observation.bearing_c,
                                         &observation.bearing_b)) {
        ++statistics->model_domain_rejections;
        continue;
      }
      observations->push_back(observation);
      occupied.push_back(working_pixel);
      ++accepted_per_cell[cell_index];
      if (observations->size() >= feature_limit) break;
    }
    ++rank;
  }
  statistics->accepted_count = observations->size();
  return true;
}

}  // namespace sphere_vio
