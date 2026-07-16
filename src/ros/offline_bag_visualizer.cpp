#include "sphere_vio/ros/offline_bag_visualizer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "sphere_vio/imu_interval_buffer.hpp"
#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/frontend/orb_descriptor_extractor.hpp"
#include "sphere_vio/geometry/epipolar_geometry.hpp"
#include "sphere_vio/geometry/spherical_geometry.hpp"
#include "sphere_vio/frontend/temporal_frontend.hpp"
#include "sphere_vio/ros/frame_assembler.hpp"
#include "sphere_vio/ros/ros_conversions.hpp"

namespace sphere_vio {
namespace {

constexpr char kWindowName[] = "Sphere-VIO Offline Bag Visualizer";
constexpr int kMaximumCanvasWidth = 1600;
constexpr int kMaximumCanvasHeight = 1000;
constexpr int kStatusHeight = 125;
constexpr int kTimelineHeight = 95;
constexpr int kCoverageMapWidth = 768;
constexpr int kCoverageMapHeight = 384;
constexpr int kCoverageMapTop = 48;
constexpr int kCoveragePanelHeight = 482;
constexpr std::size_t kMaximumPendingFrames = 4;

const std::array<const char*, 4> kCameraNames{{"LEFT", "RIGHT", "BLEFT",
                                               "BRIGHT"}};
const std::array<cv::Scalar, 4> kCoverageColors{
    {cv::Scalar(80, 255, 255), cv::Scalar(80, 255, 80),
     cv::Scalar(255, 80, 255), cv::Scalar(80, 160, 255)}};

struct IntervalDisplayStatistics {
  bool first_frame = true;
  std::size_t imu_count = 0;
  bool has_intervals = false;
  double minimum_dt = 0.0;
  double maximum_dt = 0.0;
  double average_dt = 0.0;
  bool gap_warning = false;
};

struct VisualizerStatistics {
  std::uint64_t processed_frames = 0;
  std::uint64_t imu_messages = 0;
  std::uint64_t frames_with_gap_warning = 0;
  bool has_frame = false;
  Timestamp first_frame_time = 0.0;
  Timestamp last_frame_time = 0.0;
};

struct EpipolarCurveOverlay {
  CameraId source_camera = 0U;
  CameraId target_camera = 1U;
  Eigen::Vector2d source_pixel = Eigen::Vector2d::Zero();
  std::vector<std::vector<cv::Point>> target_segments;
  std::size_t projected_sample_count = 0U;
};

bool topicMatches(const std::string& recorded_topic,
                  const std::string& configured_topic) {
  return recorded_topic == configured_topic ||
         ("/" + recorded_topic) == configured_topic;
}

std::string formatTimestamp(double value) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(9) << value;
  return output.str();
}

std::string formatDuration(double value) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(6) << value;
  return output.str();
}

std::string imageEncoding(const cv::Mat& image) {
  if (image.type() == CV_8UC1) return "mono8";
  if (image.type() == CV_8UC3) return "bgr8";
  if (image.type() == CV_8UC4) return "bgra8";
  std::ostringstream output;
  output << "cv_type=" << image.type();
  return output.str();
}

bool makeBgrImage(const cv::Mat& source, cv::Mat* bgr) {
  if (!bgr || source.empty()) return false;
  if (source.type() == CV_8UC1) {
    cv::cvtColor(source, *bgr, cv::COLOR_GRAY2BGR);
    return true;
  }
  if (source.type() == CV_8UC3) {
    *bgr = source.clone();
    return true;
  }
  if (source.type() == CV_8UC4) {
    cv::cvtColor(source, *bgr, cv::COLOR_BGRA2BGR);
    return true;
  }
  return false;
}

void drawTextWithShadow(cv::Mat* image, const std::string& text,
                        const cv::Point& origin, double scale,
                        const cv::Scalar& color) {
  cv::putText(*image, text, origin + cv::Point(1, 1),
              cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 3,
              cv::LINE_AA);
  cv::putText(*image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1,
              cv::LINE_AA);
}

std::vector<int> sparseCoordinates(int size, int target_count) {
  std::vector<int> coordinates;
  if (size <= 0 || target_count <= 0) return coordinates;
  const int step = std::max(1, size / target_count);
  for (int coordinate = 0; coordinate < size; coordinate += step) {
    coordinates.push_back(coordinate);
  }
  if (coordinates.empty() || coordinates.back() != size - 1) {
    coordinates.push_back(size - 1);
  }
  return coordinates;
}

cv::Point coveragePoint(const Eigen::Vector2d& erp_coordinate) {
  const int x = std::max(
      0, std::min(kCoverageMapWidth - 1,
                  static_cast<int>(std::lround(erp_coordinate.x()))));
  const int y = std::max(
      0, std::min(kCoverageMapHeight - 1,
                  static_cast<int>(std::lround(erp_coordinate.y()))));
  return cv::Point(x, kCoverageMapTop + y);
}

bool buildSphericalCoveragePanel(const std::string& camera_config_file,
                                 cv::Mat* panel, std::string* error) {
  if (!panel) return false;
  CameraRig rig;
  if (!loadCameraRigFromYaml(camera_config_file, &rig, error)) return false;

  *panel = cv::Mat(kCoveragePanelHeight, kCoverageMapWidth, CV_8UC3,
                   cv::Scalar(14, 14, 18));
  const cv::Rect map_area(0, kCoverageMapTop, kCoverageMapWidth,
                          kCoverageMapHeight);
  cv::rectangle(*panel, map_area, cv::Scalar(28, 28, 34), cv::FILLED);
  cv::line(*panel, cv::Point(0, kCoverageMapTop),
           cv::Point(0, kCoverageMapTop + kCoverageMapHeight - 1),
           cv::Scalar(70, 70, 255), 2, cv::LINE_AA);
  cv::line(*panel, cv::Point(kCoverageMapWidth - 1, kCoverageMapTop),
           cv::Point(kCoverageMapWidth - 1,
                     kCoverageMapTop + kCoverageMapHeight - 1),
           cv::Scalar(70, 70, 255), 2, cv::LINE_AA);
  cv::line(*panel,
           cv::Point(0, kCoverageMapTop + kCoverageMapHeight / 2),
           cv::Point(kCoverageMapWidth - 1,
                     kCoverageMapTop + kCoverageMapHeight / 2),
           cv::Scalar(130, 130, 130), 1, cv::LINE_AA);
  drawTextWithShadow(panel, "BODY SPHERICAL COVERAGE",
                     cv::Point(10, 20), 0.52, cv::Scalar(255, 255, 255));

  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    const RigCamera* camera = rig.camera(camera_id);
    if (!camera) {
      if (error) *error = "spherical coverage rig is missing a camera";
      return false;
    }
    const std::vector<int> horizontal =
        sparseCoordinates(camera->model->width(), 12);
    const std::vector<int> vertical =
        sparseCoordinates(camera->model->height(), 10);
    int valid_count = 0;
    int model_domain_invalid = 0;
    for (const int v : vertical) {
      for (const int u : horizontal) {
        Eigen::Vector3d bearing_b;
        if (!rig.pixelToBodyBearing(camera_id, Eigen::Vector2d(u, v),
                                    &bearing_b)) {
          ++model_domain_invalid;
          continue;
        }
        Eigen::Vector2d erp_coordinate;
        if (!bearingToEquirectangular(bearing_b, kCoverageMapWidth,
                                      kCoverageMapHeight, &erp_coordinate)) {
          if (error) *error = "cannot map a valid Body bearing to ERP";
          return false;
        }
        ++valid_count;
        cv::circle(*panel, coveragePoint(erp_coordinate), 2,
                   kCoverageColors[camera_id], cv::FILLED, cv::LINE_AA);
      }
    }

    const Eigen::Vector2d center_pixel(
        0.5 * static_cast<double>(camera->model->width() - 1),
        0.5 * static_cast<double>(camera->model->height() - 1));
    Eigen::Vector3d center_bearing_b;
    Eigen::Vector2d center_erp;
    if (!rig.pixelToBodyBearing(camera_id, center_pixel,
                                &center_bearing_b) ||
        !bearingToEquirectangular(center_bearing_b, kCoverageMapWidth,
                                  kCoverageMapHeight, &center_erp)) {
      if (error) *error = "cannot map a camera center bearing to ERP";
      return false;
    }
    const cv::Point center_point = coveragePoint(center_erp);
    cv::circle(*panel, center_point, 6, kCoverageColors[camera_id], 2,
               cv::LINE_AA);
    drawTextWithShadow(panel, "C" + std::to_string(camera_id),
                       center_point + cv::Point(7, -7), 0.4,
                       kCoverageColors[camera_id]);

    std::ostringstream legend;
    legend << "C" << camera_id << " " << camera->name << " valid "
           << valid_count << " invalid " << model_domain_invalid;
    drawTextWithShadow(panel, legend.str(),
                       cv::Point(10 + static_cast<int>(camera_id) * 190, 42),
                       0.34, kCoverageColors[camera_id]);
    std::cout << "  spherical coverage C" << camera_id << " "
              << camera->name << ": valid samples=" << valid_count
              << ", model-domain invalid=" << model_domain_invalid
              << ", center bearing_b=[" << center_bearing_b.transpose()
              << "]" << std::endl;
  }

  drawTextWithShadow(panel, "SPARSE BEARING SAMPLES | seam | equator",
                     cv::Point(10, kCoverageMapTop + kCoverageMapHeight + 20),
                     0.4, cv::Scalar(210, 210, 210));
  drawTextWithShadow(panel, "NO IMAGE STITCHING | NO DEPTH",
                     cv::Point(430,
                               kCoverageMapTop + kCoverageMapHeight + 20),
                     0.4, cv::Scalar(80, 190, 255));
  return true;
}

bool buildEpipolarCurveOverlay(
    const OfflineBagVisualizerOptions& options, EpipolarCurveOverlay* overlay,
    std::string* error) {
  if (!overlay || options.epipolar_source_camera < 0 ||
      options.epipolar_source_camera > 3 ||
      options.epipolar_target_camera < 0 ||
      options.epipolar_target_camera > 3 ||
      options.epipolar_source_camera == options.epipolar_target_camera) {
    if (error) *error = "invalid epipolar camera selection";
    return false;
  }
  CameraRig rig;
  if (!loadCameraRigFromYaml(options.camera_config_file, &rig, error)) {
    return false;
  }
  const CameraId source_id =
      static_cast<CameraId>(options.epipolar_source_camera);
  const CameraId target_id =
      static_cast<CameraId>(options.epipolar_target_camera);
  const RigCamera* source = rig.camera(source_id);
  const RigCamera* target = rig.camera(target_id);
  if (!source || !target) {
    if (error) *error = "epipolar camera is missing from the rig";
    return false;
  }

  EpipolarCurveOverlay result;
  result.source_camera = source_id;
  result.target_camera = target_id;
  result.source_pixel.x() =
      options.epipolar_source_u >= 0.0
          ? options.epipolar_source_u
          : 0.5 * static_cast<double>(source->model->width() - 1);
  result.source_pixel.y() =
      options.epipolar_source_v >= 0.0
          ? options.epipolar_source_v
          : 0.5 * static_cast<double>(source->model->height() - 1);

  Eigen::Vector3d bearing_source;
  if (!source->model->unproject(result.source_pixel, &bearing_source)) {
    if (error) *error = "source pixel is outside the calibrated model domain";
    return false;
  }
  RelativePose relative_pose;
  if (!relativeCameraPose(*source, *target, &relative_pose)) {
    if (error) *error = "cannot compute non-degenerate relative camera pose";
    return false;
  }
  Eigen::Vector3d plane_normal_target;
  if (!epipolarPlaneNormal(bearing_source, relative_pose.R_target_source,
                           relative_pose.t_target_source,
                           &plane_normal_target)) {
    if (error) *error = "source ray produces a degenerate epipolar plane";
    return false;
  }
  std::vector<Eigen::Vector3d> great_circle;
  if (!sampleGreatCircle(plane_normal_target, 1440U, &great_circle)) {
    if (error) *error = "cannot sample the target epipolar great circle";
    return false;
  }

  std::vector<cv::Point> current_segment;
  cv::Point previous_point;
  bool has_previous = false;
  const double maximum_pixel_step = 40.0;
  const auto finish_segment = [&]() {
    if (current_segment.size() >= 2U) {
      result.target_segments.push_back(current_segment);
    }
    current_segment.clear();
    has_previous = false;
  };
  for (const Eigen::Vector3d& bearing_target : great_circle) {
    Eigen::Vector2d target_pixel;
    if (!target->model->project(bearing_target, &target_pixel)) {
      finish_segment();
      continue;
    }
    const cv::Point point(static_cast<int>(std::lround(target_pixel.x())),
                          static_cast<int>(std::lround(target_pixel.y())));
    if (has_previous && cv::norm(point - previous_point) > maximum_pixel_step) {
      finish_segment();
    }
    current_segment.push_back(point);
    previous_point = point;
    has_previous = true;
    ++result.projected_sample_count;
  }
  finish_segment();
  if (result.target_segments.empty()) {
    if (error) *error = "epipolar great circle is not visible in target model";
    return false;
  }
  *overlay = std::move(result);
  return true;
}

void drawEpipolarOverlay(CameraId camera_id,
                         const EpipolarCurveOverlay& overlay,
                         cv::Mat* image) {
  if (!image || image->empty()) return;
  const cv::Scalar curve_color(60, 80, 255);
  if (camera_id == overlay.source_camera) {
    const cv::Point source_point(
        static_cast<int>(std::lround(overlay.source_pixel.x())),
        static_cast<int>(std::lround(overlay.source_pixel.y())));
    cv::circle(*image, source_point, 9, curve_color, 2, cv::LINE_AA);
    cv::line(*image, source_point + cv::Point(-13, 0),
             source_point + cv::Point(13, 0), curve_color, 1, cv::LINE_AA);
    cv::line(*image, source_point + cv::Point(0, -13),
             source_point + cv::Point(0, 13), curve_color, 1, cv::LINE_AA);
  } else if (camera_id == overlay.target_camera) {
    for (const std::vector<cv::Point>& segment : overlay.target_segments) {
      cv::polylines(*image, segment, false, curve_color, 2, cv::LINE_AA);
    }
  } else {
    return;
  }

  const int label_y = std::max(25, image->rows - 90);
  drawTextWithShadow(image, "SPHERICAL EPIPOLAR CURVE",
                     cv::Point(12, label_y),
                     0.55, curve_color);
  drawTextWithShadow(
      image,
      "SOURCE C" + std::to_string(overlay.source_camera) + " -> TARGET C" +
          std::to_string(overlay.target_camera),
      cv::Point(12, label_y + 28), 0.48, cv::Scalar(255, 255, 255));
  drawTextWithShadow(image, "NO FEATURE MATCHING | NO DEPTH ESTIMATION",
                     cv::Point(12, label_y + 56), 0.45,
                     cv::Scalar(80, 190, 255));
}

void drawTemporalFeatureOverlay(const CameraTrackingResult& tracking,
                                cv::Mat* image) {
  if (!image || image->empty()) return;
  const cv::Scalar new_color(80, 220, 255);
  const cv::Scalar tracked_color(80, 255, 80);
  const cv::Scalar motion_color(255, 180, 50);
  for (const FeatureTrack& track : tracking.tracks) {
    const cv::Point current(
        static_cast<int>(std::lround(track.current.pixel.x())),
        static_cast<int>(std::lround(track.current.pixel.y())));
    if (!track.newly_detected && track.has_previous_observation) {
      const cv::Point previous(
          static_cast<int>(std::lround(track.previous.pixel.x())),
          static_cast<int>(std::lround(track.previous.pixel.y())));
      cv::line(*image, previous, current, motion_color, 1, cv::LINE_AA);
    }
    cv::circle(*image, current, track.newly_detected ? 4 : 3,
               track.newly_detected ? new_color : tracked_color,
               track.newly_detected ? 1 : cv::FILLED, cv::LINE_AA);
    cv::putText(*image, std::to_string(track.id % 10000U),
                current + cv::Point(4, -3), cv::FONT_HERSHEY_PLAIN, 0.65,
                track.newly_detected ? new_color : tracked_color, 1,
                cv::LINE_AA);
  }

  const int first_line = std::max(24, image->rows - 105);
  std::ostringstream counts;
  counts << "ACTIVE " << tracking.tracks.size() << " | NEW "
         << tracking.newly_detected << " | TRACKED "
         << tracking.successfully_tracked << " | REJECTED "
         << tracking.rejected_tracks;
  std::ostringstream quality;
  quality << std::fixed << std::setprecision(2) << "AGE AVG "
          << tracking.average_track_age << " | FB AVG "
          << tracking.average_forward_backward_error;
  drawTextWithShadow(image, counts.str(), cv::Point(12, first_line), 0.45,
                     cv::Scalar(255, 255, 255));
  drawTextWithShadow(image, quality.str(), cv::Point(12, first_line + 23),
                     0.45, cv::Scalar(255, 255, 255));
  drawTextWithShadow(image, "SAME-CAMERA TEMPORAL TRACKING",
                     cv::Point(12, first_line + 46), 0.43, tracked_color);
  drawTextWithShadow(image, "NO CROSS-CAMERA MATCHING",
                     cv::Point(12, first_line + 69), 0.43,
                     cv::Scalar(80, 190, 255));
  drawTextWithShadow(image, "NO DEPTH ESTIMATION",
                     cv::Point(12, first_line + 92), 0.43,
                     cv::Scalar(80, 190, 255));
}

void drawCrossCameraMatchOverlay(
    const CrossCameraPairResult& matching, std::size_t maximum_matches,
    const std::array<cv::Point2d, 4>& image_offsets,
    const std::array<cv::Point2d, 4>& image_scales, cv::Mat* canvas) {
  if (!canvas || canvas->empty()) return;
  const std::size_t displayed =
      std::min(maximum_matches, matching.matches.size());
  const cv::Scalar line_color(255, 120, 40);
  const cv::Scalar first_color(80, 255, 255);
  const cv::Scalar second_color(255, 80, 255);
  const auto canvas_point = [&](CameraId camera_id,
                                const Eigen::Vector2d& pixel) {
    return cv::Point(
        static_cast<int>(std::lround(
            image_offsets[camera_id].x +
            pixel.x() * image_scales[camera_id].x)),
        static_cast<int>(std::lround(
            image_offsets[camera_id].y +
            pixel.y() * image_scales[camera_id].y)));
  };
  for (std::size_t index = 0U; index < displayed; ++index) {
    const CrossCameraMatch& match = matching.matches[index];
    const cv::Point first = canvas_point(match.camera_id_1, match.pixel_1);
    const cv::Point second = canvas_point(match.camera_id_2, match.pixel_2);
    cv::line(*canvas, first, second, line_color, 1, cv::LINE_AA);
    cv::circle(*canvas, first, 5, first_color, 2, cv::LINE_AA);
    cv::circle(*canvas, second, 5, second_color, 2, cv::LINE_AA);
    if (index < 12U) {
      std::ostringstream label;
      label << std::fixed << std::setprecision(3) << match.descriptor_distance
            << "/" << match.epipolar_error_maximum;
      const cv::Point midpoint((first.x + second.x) / 2,
                               (first.y + second.y) / 2);
      drawTextWithShadow(canvas, label.str(), midpoint, 0.32,
                         cv::Scalar(255, 255, 255));
    }
  }

  const int box_width = std::min(700, canvas->cols);
  const int box_x = std::max(
      0, std::min(canvas->cols - box_width,
                  static_cast<int>(image_offsets[matching.camera_id_1].x)));
  const int requested_y =
      static_cast<int>(image_offsets[matching.camera_id_1].y) + 70;
  const int box_y = std::max(0, std::min(canvas->rows - 122, requested_y));
  const cv::Rect box(box_x, box_y, box_width, std::min(122, canvas->rows));
  cv::rectangle(*canvas, box, cv::Scalar(12, 12, 16), cv::FILLED);
  drawTextWithShadow(canvas, "CROSS-CAMERA CANDIDATE MATCHES",
                     box.tl() + cv::Point(10, 22), 0.48, line_color);
  drawTextWithShadow(canvas,
                     "DESCRIPTOR + SPHERICAL EPIPOLAR FILTER",
                     box.tl() + cv::Point(10, 45), 0.43,
                     cv::Scalar(255, 255, 255));
  std::ostringstream pair_text;
  pair_text << "C" << matching.camera_id_1 << "-C" << matching.camera_id_2
            << " FINAL " << matching.matches.size() << " DISPLAYED "
            << displayed;
  drawTextWithShadow(canvas, pair_text.str(),
                     box.tl() + cv::Point(10, 68), 0.43,
                     cv::Scalar(255, 255, 255));
  drawTextWithShadow(canvas, "NO TRIANGULATION | NO DEPTH",
                     box.tl() + cv::Point(10, 91), 0.43,
                     cv::Scalar(80, 190, 255));
  drawTextWithShadow(canvas, "NO LANDMARK ASSOCIATION",
                     box.tl() + cv::Point(10, 114), 0.43,
                     cv::Scalar(80, 190, 255));
}

void appendSphericalCoveragePanel(const cv::Mat& panel, cv::Mat* canvas) {
  if (!canvas || canvas->empty() || panel.empty()) return;
  cv::Mat displayed_panel = panel;
  if (panel.cols > canvas->cols) {
    const double scale =
        static_cast<double>(canvas->cols) / static_cast<double>(panel.cols);
    cv::resize(panel, displayed_panel,
               cv::Size(canvas->cols,
                        std::max(1, static_cast<int>(std::lround(
                                        panel.rows * scale)))),
               0.0, 0.0, cv::INTER_AREA);
  }
  cv::Mat combined(canvas->rows + displayed_panel.rows, canvas->cols,
                   CV_8UC3, cv::Scalar(14, 14, 18));
  canvas->copyTo(combined(cv::Rect(0, 0, canvas->cols, canvas->rows)));
  const int offset_x = (canvas->cols - displayed_panel.cols) / 2;
  displayed_panel.copyTo(combined(cv::Rect(
      offset_x, canvas->rows, displayed_panel.cols, displayed_panel.rows)));
  *canvas = std::move(combined);
}

IntervalDisplayStatistics calculateIntervalStatistics(
    bool first_frame, const std::vector<ImuMeasurement>& measurements,
    double gap_warning) {
  IntervalDisplayStatistics statistics;
  statistics.first_frame = first_frame;
  statistics.imu_count = measurements.size();
  if (first_frame || measurements.size() < 2) return statistics;

  double sum = 0.0;
  std::size_t interval_count = 0;
  for (std::size_t index = 1; index < measurements.size(); ++index) {
    const double dt =
        measurements[index].timestamp - measurements[index - 1].timestamp;
    if (!std::isfinite(dt) || dt <= 0.0) continue;
    if (!statistics.has_intervals) {
      statistics.minimum_dt = dt;
      statistics.maximum_dt = dt;
      statistics.has_intervals = true;
    } else {
      statistics.minimum_dt = std::min(statistics.minimum_dt, dt);
      statistics.maximum_dt = std::max(statistics.maximum_dt, dt);
    }
    sum += dt;
    ++interval_count;
  }
  if (statistics.has_intervals) {
    statistics.average_dt = sum / interval_count;
    statistics.gap_warning = statistics.maximum_dt > gap_warning;
  }
  return statistics;
}

void drawPlaybackState(cv::Mat* canvas, bool paused) {
  if (!canvas || canvas->empty()) return;
  const int box_width = 125;
  const cv::Rect area(std::max(0, canvas->cols - box_width), 0,
                      std::min(box_width, canvas->cols), 36);
  cv::rectangle(*canvas, area, cv::Scalar(18, 18, 18), cv::FILLED);
  cv::putText(*canvas, paused ? "PAUSED" : "PLAYING",
              cv::Point(area.x + 8, 25), cv::FONT_HERSHEY_SIMPLEX, 0.55,
              paused ? cv::Scalar(0, 220, 255) : cv::Scalar(80, 255, 80), 1,
              cv::LINE_AA);
}

bool renderFrame(const MultiCameraFrame& frame, std::uint64_t frame_index,
                 bool first_frame, Timestamp previous_frame_time,
                 const std::vector<ImuMeasurement>& imu_measurements,
                 double imu_gap_warning, bool paused, cv::Mat* canvas,
                 IntervalDisplayStatistics* interval_statistics,
                 const cv::Mat* spherical_coverage_panel,
                 const EpipolarCurveOverlay* epipolar_curve_overlay,
                 const MultiCameraTrackingResult* temporal_tracking,
                 const CrossCameraPairResult* cross_camera_matching,
                 std::size_t maximum_displayed_matches,
                 std::string* error) {
  if (!canvas || !interval_statistics) return false;
  if (frame.images.size() != FrameAssembler::kCameraCount) {
    if (error) *error = "completed frame does not contain four images";
    return false;
  }

  std::array<const ImageFrame*, FrameAssembler::kCameraCount> images{};
  int maximum_width = 0;
  int maximum_height = 0;
  for (const ImageFrame& image : frame.images) {
    if (image.camera_id >= FrameAssembler::kCameraCount || image.image.empty()) {
      if (error) *error = "completed frame contains an invalid camera image";
      return false;
    }
    images[image.camera_id] = &image;
    maximum_width = std::max(maximum_width, image.image.cols);
    maximum_height = std::max(maximum_height, image.image.rows);
  }
  if (std::any_of(images.begin(), images.end(),
                  [](const ImageFrame* image) { return image == nullptr; })) {
    if (error) *error = "completed frame contains duplicate camera ids";
    return false;
  }

  const double horizontal_scale =
      static_cast<double>(kMaximumCanvasWidth) / (2.0 * maximum_width);
  const double vertical_scale =
      static_cast<double>(kMaximumCanvasHeight - kStatusHeight -
                          kTimelineHeight) /
      (2.0 * maximum_height);
  const double scale = std::min(1.0, std::min(horizontal_scale, vertical_scale));
  const int cell_width =
      std::max(1, static_cast<int>(std::lround(maximum_width * scale)));
  const int cell_height =
      std::max(1, static_cast<int>(std::lround(maximum_height * scale)));
  const int canvas_width = 2 * cell_width;
  const int canvas_height = kStatusHeight + 2 * cell_height + kTimelineHeight;
  *canvas = cv::Mat(canvas_height, canvas_width, CV_8UC3,
                    cv::Scalar(18, 18, 18));
  std::array<cv::Point2d, 4> image_offsets;
  std::array<cv::Point2d, 4> image_scales;

  for (std::size_t camera_id = 0; camera_id < images.size(); ++camera_id) {
    cv::Mat bgr;
    if (!makeBgrImage(images[camera_id]->image, &bgr)) {
      if (error) {
        *error = "unsupported image encoding for camera " +
                 std::to_string(camera_id);
      }
      return false;
    }
    if (epipolar_curve_overlay) {
      drawEpipolarOverlay(static_cast<CameraId>(camera_id),
                          *epipolar_curve_overlay, &bgr);
    }
    if (temporal_tracking) {
      const CameraTrackingResult& tracking =
          temporal_tracking->cameras[camera_id];
      if (tracking.camera_id != camera_id) {
        if (error) *error = "temporal tracking camera id mismatch";
        return false;
      }
      drawTemporalFeatureOverlay(tracking, &bgr);
    }
    cv::Mat resized;
    const cv::Size scaled_size(
        std::max(1, static_cast<int>(std::lround(bgr.cols * scale))),
        std::max(1, static_cast<int>(std::lround(bgr.rows * scale))));
    cv::resize(bgr, resized, scaled_size, 0.0, 0.0,
               scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);

    const int column = static_cast<int>(camera_id % 2);
    const int row = static_cast<int>(camera_id / 2);
    const int cell_x = column * cell_width;
    const int cell_y = kStatusHeight + row * cell_height;
    const int offset_x = cell_x + (cell_width - resized.cols) / 2;
    const int offset_y = cell_y + (cell_height - resized.rows) / 2;
    image_offsets[camera_id] = cv::Point2d(offset_x, offset_y);
    image_scales[camera_id] = cv::Point2d(
        static_cast<double>(resized.cols) / bgr.cols,
        static_cast<double>(resized.rows) / bgr.rows);
    resized.copyTo((*canvas)(cv::Rect(offset_x, offset_y, resized.cols,
                                      resized.rows)));

    const cv::Point text_origin(offset_x + 8, offset_y + 21);
    drawTextWithShadow(canvas,
                       "C" + std::to_string(camera_id) + " " +
                           kCameraNames[camera_id],
                       text_origin, 0.52, cv::Scalar(80, 255, 255));
    drawTextWithShadow(
        canvas, "t = " + formatTimestamp(images[camera_id]->timestamp),
        text_origin + cv::Point(0, 21), 0.43, cv::Scalar(255, 255, 255));
    drawTextWithShadow(
        canvas,
        std::to_string(images[camera_id]->image.cols) + "x" +
            std::to_string(images[camera_id]->image.rows) + " " +
            imageEncoding(images[camera_id]->image),
        text_origin + cv::Point(0, 41), 0.43, cv::Scalar(255, 255, 255));
  }
  *interval_statistics = calculateIntervalStatistics(
      first_frame, imu_measurements, imu_gap_warning);
  const IntervalDisplayStatistics& imu = *interval_statistics;
  const double image_interval =
      first_frame ? 0.0 : frame.timestamp - previous_frame_time;
  const std::string previous_text =
      first_frame ? "N/A" : formatTimestamp(previous_frame_time);
  const std::string interval_text =
      first_frame ? "N/A" : formatDuration(image_interval) + " s";
  const std::string imu_count_text =
      first_frame ? "N/A" : std::to_string(imu.imu_count);
  const std::string minimum_dt_text =
      imu.has_intervals ? formatDuration(imu.minimum_dt) + " s" : "N/A";
  const std::string maximum_dt_text =
      imu.has_intervals ? formatDuration(imu.maximum_dt) + " s" : "N/A";
  const std::string average_dt_text =
      imu.has_intervals ? formatDuration(imu.average_dt) + " s" : "N/A";

  const cv::Scalar label_color(190, 190, 190);
  const cv::Scalar value_color(240, 240, 240);
  const int right_column = canvas_width / 2;
  cv::putText(*canvas, "frame", cv::Point(12, 23),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, label_color, 1, cv::LINE_AA);
  cv::putText(*canvas, std::to_string(frame_index), cv::Point(125, 23),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, value_color, 1, cv::LINE_AA);
  cv::putText(*canvas, "current t", cv::Point(12, 46),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, label_color, 1, cv::LINE_AA);
  cv::putText(*canvas, formatTimestamp(frame.timestamp), cv::Point(125, 46),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, value_color, 1, cv::LINE_AA);
  cv::putText(*canvas, "previous t", cv::Point(12, 69),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, label_color, 1, cv::LINE_AA);
  cv::putText(*canvas, previous_text, cv::Point(125, 69),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, value_color, 1, cv::LINE_AA);
  cv::putText(*canvas, "image dt", cv::Point(12, 92),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, label_color, 1, cv::LINE_AA);
  cv::putText(*canvas, interval_text, cv::Point(125, 92),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, value_color, 1, cv::LINE_AA);

  cv::putText(*canvas, "IMU count", cv::Point(right_column, 46),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, label_color, 1, cv::LINE_AA);
  cv::putText(*canvas, imu_count_text,
              cv::Point(right_column + 105, 46), cv::FONT_HERSHEY_SIMPLEX,
              0.45, value_color, 1, cv::LINE_AA);
  cv::putText(*canvas, "IMU dt min", cv::Point(right_column, 69),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, label_color, 1, cv::LINE_AA);
  cv::putText(*canvas, minimum_dt_text,
              cv::Point(right_column + 105, 69), cv::FONT_HERSHEY_SIMPLEX,
              0.45, value_color, 1, cv::LINE_AA);
  cv::putText(*canvas, "max", cv::Point(right_column + 245, 69),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, label_color, 1, cv::LINE_AA);
  cv::putText(*canvas, maximum_dt_text,
              cv::Point(right_column + 285, 69), cv::FONT_HERSHEY_SIMPLEX,
              0.45,
              imu.gap_warning ? cv::Scalar(40, 40, 255) : value_color, 1,
              cv::LINE_AA);
  cv::putText(*canvas, "IMU dt avg", cv::Point(right_column, 92),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, label_color, 1, cv::LINE_AA);
  cv::putText(*canvas, average_dt_text,
              cv::Point(right_column + 105, 92), cv::FONT_HERSHEY_SIMPLEX,
              0.45, value_color, 1, cv::LINE_AA);

  const std::string interval_state =
      first_frame ? "FIRST FRAME"
                  : (imu.gap_warning ? "IMU GAP" : "IMU OK");
  cv::putText(*canvas, "SYNC OK | " + interval_state, cv::Point(12, 116),
              cv::FONT_HERSHEY_SIMPLEX, 0.48,
              imu.gap_warning ? cv::Scalar(40, 40, 255)
                              : cv::Scalar(80, 255, 80),
              1, cv::LINE_AA);
  cv::putText(*canvas,
              "RAW SENSOR VISUALIZATION - NO CAMERA PROJECTION APPLIED",
              cv::Point(std::max(12, canvas_width / 2 - 210), 116),
              cv::FONT_HERSHEY_SIMPLEX, 0.39, cv::Scalar(80, 190, 255), 1,
              cv::LINE_AA);
  drawPlaybackState(canvas, paused);

  const int timeline_top = kStatusHeight + 2 * cell_height;
  const int timeline_y = timeline_top + 48;
  const int timeline_left = 38;
  const int timeline_right = canvas_width - 38;
  cv::line(*canvas, cv::Point(timeline_left, timeline_y),
           cv::Point(timeline_right, timeline_y), cv::Scalar(210, 210, 210),
           1, cv::LINE_AA);
  cv::line(*canvas, cv::Point(timeline_left, timeline_y - 15),
           cv::Point(timeline_left, timeline_y + 15),
           cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  cv::line(*canvas, cv::Point(timeline_right, timeline_y - 15),
           cv::Point(timeline_right, timeline_y + 15),
           cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  cv::putText(*canvas, first_frame ? "NO PREVIOUS FRAME" : "previous image",
              cv::Point(timeline_left, timeline_top + 22),
              cv::FONT_HERSHEY_SIMPLEX, 0.42, value_color, 1, cv::LINE_AA);
  cv::putText(*canvas, "current image",
              cv::Point(std::max(timeline_left, timeline_right - 105),
                        timeline_top + 22),
              cv::FONT_HERSHEY_SIMPLEX, 0.42, value_color, 1, cv::LINE_AA);
  cv::putText(*canvas, "Space pause/resume   N next frame   Q/Esc quit",
              cv::Point(timeline_left, timeline_top + 84),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, label_color, 1, cv::LINE_AA);

  if (!first_frame && frame.timestamp > previous_frame_time) {
    const double interval = frame.timestamp - previous_frame_time;
    for (std::size_t index = 0; index < imu_measurements.size(); ++index) {
      const double fraction = std::max(
          0.0, std::min(1.0, (imu_measurements[index].timestamp -
                              previous_frame_time) /
                                 interval));
      const int x = timeline_left + static_cast<int>(std::lround(
                                        fraction *
                                        (timeline_right - timeline_left)));
      cv::line(*canvas, cv::Point(x, timeline_y - 8),
               cv::Point(x, timeline_y + 8), cv::Scalar(80, 255, 255), 1,
               cv::LINE_AA);
      if (index > 0 &&
          imu_measurements[index].timestamp -
                  imu_measurements[index - 1].timestamp >
              imu_gap_warning) {
        const double previous_fraction = std::max(
            0.0, std::min(1.0, (imu_measurements[index - 1].timestamp -
                                previous_frame_time) /
                                   interval));
        const int gap_x = timeline_left + static_cast<int>(std::lround(
                                            0.5 *
                                            (fraction + previous_fraction) *
                                            (timeline_right - timeline_left)));
        cv::circle(*canvas, cv::Point(gap_x, timeline_y), 6,
                   cv::Scalar(30, 30, 255), cv::FILLED, cv::LINE_AA);
      }
    }
  }
  if (cross_camera_matching) {
    drawCrossCameraMatchOverlay(*cross_camera_matching,
                                maximum_displayed_matches, image_offsets,
                                image_scales, canvas);
  }
  if (spherical_coverage_panel) {
    appendSphericalCoveragePanel(*spherical_coverage_panel, canvas);
  }
  return true;
}

class PlaybackController {
 public:
  explicit PlaybackController(double playback_rate)
      : playback_rate_(playback_rate) {}

  bool paused() const { return paused_; }

  void frameDisplayed() {
    last_display_time_ = std::chrono::steady_clock::now();
    has_last_display_time_ = true;
  }

  bool waitBeforeFrame(double image_interval, cv::Mat* displayed_canvas) {
    if (!displayed_canvas || displayed_canvas->empty()) return true;
    if (paused_ && advance_one_frame_) {
      advance_one_frame_ = false;
      return true;
    }
    const double wait_seconds = std::max(0.0, image_interval / playback_rate_);
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
        has_last_display_time_
            ? std::chrono::duration<double>(now - last_display_time_).count()
            : 0.0;
    const auto deadline = now + std::chrono::duration<double>(
                                    std::max(0.0, wait_seconds - elapsed));
    while (true) {
      int wait_ms = 30;
      if (!paused_) {
        const double remaining = std::chrono::duration<double>(
                                     deadline - std::chrono::steady_clock::now())
                                     .count();
        if (remaining <= 0.0) return true;
        wait_ms = std::max(1, std::min(30, static_cast<int>(
                                                  std::ceil(remaining * 1000.0))));
      }
      const int key = cv::waitKey(wait_ms);
      if (!handleKey(key, displayed_canvas)) return false;
      if (advance_one_frame_) {
        advance_one_frame_ = false;
        return true;
      }
      if (!paused_ && wait_seconds <= 0.0) return true;
    }
  }

  bool pumpAfterDisplay(cv::Mat* displayed_canvas) {
    return handleKey(cv::waitKey(1), displayed_canvas);
  }

 private:
  bool handleKey(int key, cv::Mat* displayed_canvas) {
    if (key < 0) return true;
    const int normalized = key & 0xff;
    if (normalized == 27 || normalized == 'q' || normalized == 'Q') {
      return false;
    }
    if (normalized == ' ') {
      paused_ = !paused_;
      drawPlaybackState(displayed_canvas, paused_);
      cv::imshow(kWindowName, *displayed_canvas);
    } else if (paused_ && (normalized == 'n' || normalized == 'N')) {
      advance_one_frame_ = true;
    }
    return true;
  }

  double playback_rate_;
  bool paused_ = false;
  bool advance_one_frame_ = false;
  bool has_last_display_time_ = false;
  std::chrono::steady_clock::time_point last_display_time_;
};

}  // namespace

OfflineBagVisualizer::OfflineBagVisualizer(OfflineBagVisualizerOptions options)
    : options_(std::move(options)) {}

int OfflineBagVisualizer::run() {
  CameraRig temporal_camera_rig;
  std::unique_ptr<TemporalFrontend> temporal_frontend;
  if (options_.show_temporal_features || options_.show_cross_camera_matches) {
    std::string rig_error;
    if (!loadCameraRigFromYaml(options_.camera_config_file,
                               &temporal_camera_rig, &rig_error)) {
      std::cerr << "Cannot load temporal frontend camera rig: " << rig_error
                << std::endl;
      return 2;
    }
    temporal_frontend.reset(new TemporalFrontend(options_.frontend));
  }
  std::unique_ptr<OrbDescriptorExtractor> descriptor_extractor;
  std::unique_ptr<CrossCameraMatcher> cross_camera_matcher;
  if (options_.show_cross_camera_matches) {
    descriptor_extractor.reset(new OrbDescriptorExtractor(options_.descriptor));
    cross_camera_matcher.reset(new CrossCameraMatcher(options_.matcher));
    if (!cross_camera_matcher->isConfiguredPair(options_.match_camera_1,
                                                options_.match_camera_2)) {
      std::cerr << "Requested pair is not a configured Kalibr overlap pair."
                << std::endl;
      return 2;
    }
  }
  cv::Mat spherical_coverage_panel;
  if (options_.show_spherical_coverage) {
    std::string coverage_error;
    std::cout << "Precomputing sparse spherical coverage from: "
              << options_.camera_config_file << std::endl;
    if (!buildSphericalCoveragePanel(options_.camera_config_file,
                                     &spherical_coverage_panel,
                                     &coverage_error)) {
      std::cerr << "Cannot build spherical coverage panel: "
                << coverage_error << std::endl;
      return 2;
    }
  }

  EpipolarCurveOverlay epipolar_curve_overlay;
  if (options_.show_epipolar_curve) {
    std::string curve_error;
    std::cout << "Precomputing spherical epipolar curve from: "
              << options_.camera_config_file << std::endl;
    if (!buildEpipolarCurveOverlay(options_, &epipolar_curve_overlay,
                                   &curve_error)) {
      std::cerr << "Cannot build spherical epipolar curve: " << curve_error
                << std::endl;
      return 2;
    }
    std::cout << "  epipolar curve C" << epipolar_curve_overlay.source_camera
              << " -> C" << epipolar_curve_overlay.target_camera
              << ", source pixel=["
              << epipolar_curve_overlay.source_pixel.transpose()
              << "], projected samples="
              << epipolar_curve_overlay.projected_sample_count
              << ", continuous segments="
              << epipolar_curve_overlay.target_segments.size() << std::endl;
  }
  if (options_.show_temporal_features) {
    std::cout << "  temporal features: enabled (same-camera LK only)"
              << std::endl;
  }
  if (options_.show_cross_camera_matches) {
    std::cout << "  cross-camera candidates: enabled for C"
              << options_.match_camera_1 << "-C" << options_.match_camera_2
              << " (no triangulation or depth)" << std::endl;
  }

  rosbag::Bag bag;
  try {
    bag.open(options_.bag.bag_path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& exception) {
    std::cerr << "Cannot open bag: " << exception.what() << std::endl;
    return 2;
  }

  const std::vector<std::string> topics{
      options_.bag.camera_topics[0], options_.bag.camera_topics[1],
      options_.bag.camera_topics[2], options_.bag.camera_topics[3],
      options_.bag.imu_topic};
  rosbag::View complete_view(bag, rosbag::TopicQuery(topics));
  if (complete_view.size() == 0) {
    std::cerr << "The bag contains no messages on the configured topics."
              << std::endl;
    bag.close();
    return 3;
  }

  const ros::Time bag_start = complete_view.getBeginTime();
  const ros::Time bag_end = complete_view.getEndTime();
  const ros::Time processing_start =
      bag_start + ros::Duration(options_.bag.start_time_offset);
  ros::Time processing_end = bag_end;
  if (options_.bag.duration >= 0.0) {
    processing_end = std::min(
        bag_end, processing_start + ros::Duration(options_.bag.duration));
  }
  if (processing_start > bag_end || processing_end < processing_start) {
    std::cerr << "The configured time range does not overlap the bag."
              << std::endl;
    bag.close();
    return 4;
  }

  std::cout << "Offline bag visualizer\n  bag: " << options_.bag.bag_path
            << "\n  rate: " << options_.playback_rate
            << "\n  IMU gap warning: " << options_.imu_gap_warning << " s";
  for (std::size_t index = 0; index < options_.bag.camera_topics.size();
       ++index) {
    std::cout << "\n  C" << index << " " << kCameraNames[index] << ": "
              << options_.bag.camera_topics[index];
  }
  std::cout << "\n  IMU: " << options_.bag.imu_topic << std::endl;
  if (options_.show_spherical_coverage) {
    std::cout << "  spherical coverage: enabled (sparse bearings only)"
              << std::endl;
  }
  if (options_.show_epipolar_curve) {
    std::cout << "  spherical epipolar curve: enabled (geometry only)"
              << std::endl;
  }

  try {
    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
  } catch (const cv::Exception& exception) {
    std::cerr << "Cannot create OpenCV window: " << exception.what()
              << std::endl;
    bag.close();
    return 5;
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics), processing_start,
                    processing_end);
  FrameAssembler assembler(options_.bag.maximum_image_time_difference,
                           options_.bag.require_exact_image_timestamps);
  ImuIntervalBuffer imu_buffer(options_.bag.maximum_imu_time_difference);
  PlaybackController playback(options_.playback_rate);
  VisualizerStatistics run_statistics;
  std::deque<MultiCameraFrame> pending_frames;
  bool has_latest_imu_time = false;
  Timestamp latest_imu_time = 0.0;
  bool has_previous_frame = false;
  Timestamp previous_frame_time = 0.0;
  cv::Mat displayed_canvas;
  bool user_quit = false;
  bool visualization_error = false;

  const auto display_frame = [&](MultiCameraFrame frame) {
    const bool first_frame = !has_previous_frame;
    std::vector<ImuMeasurement> interval;
    if (first_frame) {
      imu_buffer.discardThrough(frame.timestamp);
    } else {
      interval = imu_buffer.extract(previous_frame_time, frame.timestamp);
      if (!playback.waitBeforeFrame(frame.timestamp - previous_frame_time,
                                    &displayed_canvas)) {
        user_quit = true;
        return false;
      }
    }

    IntervalDisplayStatistics interval_statistics;
    std::string render_error;
    cv::Mat canvas;
    MultiCameraTrackingResult temporal_tracking;
    if (temporal_frontend &&
        !temporal_frontend->processFrame(frame, temporal_camera_rig,
                                         &temporal_tracking)) {
      std::cerr << "Temporal frontend rejected frame at "
                << formatTimestamp(frame.timestamp) << std::endl;
      visualization_error = true;
      return false;
    }
    CrossCameraPairResult cross_camera_result;
    if (cross_camera_matcher) {
      std::array<const ImageFrame*, 4> images{{nullptr, nullptr, nullptr,
                                               nullptr}};
      for (const ImageFrame& image : frame.images) {
        if (image.camera_id >= 4U || images[image.camera_id] != nullptr) {
          visualization_error = true;
          return false;
        }
        images[image.camera_id] = &image;
      }
      if (!images[options_.match_camera_1] ||
          !images[options_.match_camera_2]) {
        visualization_error = true;
        return false;
      }
      CameraDescriptorSet first_descriptors;
      CameraDescriptorSet second_descriptors;
      DescriptorExtractionStatistics first_statistics;
      DescriptorExtractionStatistics second_statistics;
      if (!descriptor_extractor->extract(
              images[options_.match_camera_1]->image,
              temporal_tracking.cameras[options_.match_camera_1],
              &first_descriptors, &first_statistics) ||
          !descriptor_extractor->extract(
              images[options_.match_camera_2]->image,
              temporal_tracking.cameras[options_.match_camera_2],
              &second_descriptors, &second_statistics) ||
          !cross_camera_matcher->matchPair(
              first_descriptors, second_descriptors, temporal_camera_rig,
              &cross_camera_result)) {
        std::cerr << "Cross-camera visualization matching failed at "
                  << formatTimestamp(frame.timestamp) << std::endl;
        visualization_error = true;
        return false;
      }
    }
    if (!renderFrame(frame, run_statistics.processed_frames + 1, first_frame,
                     previous_frame_time, interval, options_.imu_gap_warning,
                     playback.paused(), &canvas, &interval_statistics,
                     options_.show_spherical_coverage
                         ? &spherical_coverage_panel
                         : nullptr,
                     options_.show_epipolar_curve
                         ? &epipolar_curve_overlay
                         : nullptr,
                     temporal_frontend ? &temporal_tracking : nullptr,
                     cross_camera_matcher ? &cross_camera_result : nullptr,
                     options_.maximum_displayed_matches,
                     &render_error)) {
      std::cerr << "Cannot render frame: " << render_error << std::endl;
      visualization_error = true;
      return false;
    }
    displayed_canvas = std::move(canvas);
    if (run_statistics.processed_frames == 0) {
      cv::resizeWindow(kWindowName, displayed_canvas.cols,
                       displayed_canvas.rows);
    }
    cv::imshow(kWindowName, displayed_canvas);
    playback.frameDisplayed();
    if (!playback.pumpAfterDisplay(&displayed_canvas)) {
      user_quit = true;
      return false;
    }

    ++run_statistics.processed_frames;
    if (interval_statistics.gap_warning) {
      ++run_statistics.frames_with_gap_warning;
    }
    if (!run_statistics.has_frame) {
      run_statistics.has_frame = true;
      run_statistics.first_frame_time = frame.timestamp;
    }
    run_statistics.last_frame_time = frame.timestamp;
    has_previous_frame = true;
    previous_frame_time = frame.timestamp;
    return true;
  };

  const auto display_ready_frames = [&](bool force) {
    while (!pending_frames.empty() &&
           (force ||
            (has_latest_imu_time &&
             latest_imu_time >= pending_frames.front().timestamp) ||
            pending_frames.size() > kMaximumPendingFrames)) {
      if (!display_frame(std::move(pending_frames.front()))) return false;
      pending_frames.pop_front();
    }
    return true;
  };

  try {
    for (const rosbag::MessageInstance& instance : view) {
      if (topicMatches(instance.getTopic(), options_.bag.imu_topic)) {
        const sensor_msgs::ImuConstPtr message =
            instance.instantiate<sensor_msgs::Imu>();
        if (!message) continue;
        ++run_statistics.imu_messages;
        ImuMeasurement measurement;
        if (!convertImuMessage(*message, &measurement)) continue;
        imu_buffer.add(measurement);
        if (!has_latest_imu_time || measurement.timestamp > latest_imu_time) {
          has_latest_imu_time = true;
          latest_imu_time = measurement.timestamp;
        }
      } else {
        for (CameraId camera_id = 0;
             camera_id < FrameAssembler::kCameraCount; ++camera_id) {
          if (!topicMatches(instance.getTopic(),
                            options_.bag.camera_topics[camera_id])) {
            continue;
          }
          const sensor_msgs::ImageConstPtr message =
              instance.instantiate<sensor_msgs::Image>();
          if (!message) break;
          MultiCameraFrame frame;
          if (assembler.addImage(camera_id, message, &frame)) {
            pending_frames.push_back(std::move(frame));
          }
          break;
        }
      }
      if (!display_ready_frames(false)) {
        break;
      }
    }
    if (!user_quit && !visualization_error) display_ready_frames(true);
  } catch (const cv::Exception& exception) {
    std::cerr << "OpenCV visualization failed: " << exception.what()
              << std::endl;
    visualization_error = true;
  }

  const std::size_t pending_images = assembler.pendingImageCount();
  assembler.discardPendingImages();
  const FrameAssembler::Statistics& frame_statistics = assembler.statistics();
  cv::destroyWindow(kWindowName);
  bag.close();

  std::cout << "\nOffline visualization summary"
            << "\n  processed four-camera frames: "
            << run_statistics.processed_frames
            << "\n  camera timestamp mismatches: "
            << frame_statistics.timestamp_mismatches
            << "\n  dropped images: " << frame_statistics.dropped_images
            << "\n  pending images at exit: " << pending_images
            << "\n  IMU message count: " << run_statistics.imu_messages
            << "\n  frames with IMU gap warning: "
            << run_statistics.frames_with_gap_warning
            << "\n  first frame timestamp: "
            << (run_statistics.has_frame
                    ? formatTimestamp(run_statistics.first_frame_time)
                    : "N/A")
            << "\n  last frame timestamp: "
            << (run_statistics.has_frame
                    ? formatTimestamp(run_statistics.last_frame_time)
                    : "N/A")
            << "\n  exit reason: "
            << (visualization_error
                    ? "visualization error"
                    : (user_quit ? "user exit" : "bag complete"))
            << std::endl;
  return visualization_error ? 6 : 0;
}

}  // namespace sphere_vio
