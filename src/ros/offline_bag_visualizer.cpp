#include "sphere_vio/ros/offline_bag_visualizer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
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
#include "sphere_vio/geometry/spherical_geometry.hpp"
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

  for (std::size_t camera_id = 0; camera_id < images.size(); ++camera_id) {
    cv::Mat bgr;
    if (!makeBgrImage(images[camera_id]->image, &bgr)) {
      if (error) {
        *error = "unsupported image encoding for camera " +
                 std::to_string(camera_id);
      }
      return false;
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
    if (!renderFrame(frame, run_statistics.processed_frames + 1, first_frame,
                     previous_frame_time, interval, options_.imu_gap_warning,
                     playback.paused(), &canvas, &interval_statistics,
                     options_.show_spherical_coverage
                         ? &spherical_coverage_panel
                         : nullptr,
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
