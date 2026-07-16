#include "sphere_vio/frontend/feature_tracker.hpp"

#include <cmath>
#include <utility>

#include <opencv2/video/tracking.hpp>

namespace sphere_vio {
namespace {

bool validOptions(const FeatureTrackerOptions& options) {
  return options.window_size.width > 0 && options.window_size.height > 0 &&
         options.pyramid_levels >= 0 && options.maximum_iterations > 0 &&
         std::isfinite(options.termination_epsilon) &&
         options.termination_epsilon > 0.0 &&
         std::isfinite(options.maximum_forward_backward_error) &&
         options.maximum_forward_backward_error >= 0.0 &&
         std::isfinite(options.maximum_lk_error) &&
         options.maximum_lk_error >= 0.0 && options.border_margin >= 0;
}

bool finitePoint(const cv::Point2f& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool insideBorder(const cv::Point2f& point, int width, int height,
                  int margin) {
  return finitePoint(point) && point.x >= static_cast<float>(margin) &&
         point.y >= static_cast<float>(margin) &&
         point.x < static_cast<float>(width - margin) &&
         point.y < static_cast<float>(height - margin);
}

}  // namespace

FeatureTracker::FeatureTracker(FeatureTrackerOptions options)
    : options_(std::move(options)) {}

bool FeatureTracker::track(
    const cv::Mat& previous_image, const cv::Mat& current_image,
    Timestamp current_timestamp, CameraId camera_id,
    const CameraRig& camera_rig,
    const std::vector<FeatureTrack>& input_tracks,
    std::vector<FeatureTrack>* output_tracks,
    FeatureTrackingStatistics* statistics) const {
  if (!output_tracks || !statistics) return false;
  output_tracks->clear();
  *statistics = FeatureTrackingStatistics{};
  statistics->input_tracks = input_tracks.size();

  const RigCamera* rig_camera = camera_rig.camera(camera_id);
  if (!validOptions(options_) || previous_image.empty() ||
      current_image.empty() || previous_image.type() != CV_8UC1 ||
      current_image.type() != CV_8UC1 ||
      previous_image.size() != current_image.size() ||
      !std::isfinite(current_timestamp) || !rig_camera ||
      current_image.cols <= 2 * options_.border_margin ||
      current_image.rows <= 2 * options_.border_margin) {
    return false;
  }
  if (input_tracks.empty()) return true;

  std::vector<cv::Point2f> previous_points;
  previous_points.reserve(input_tracks.size());
  for (const FeatureTrack& track : input_tracks) {
    if (track.camera_id != camera_id ||
        track.current.camera_id != camera_id ||
        !track.current.pixel.allFinite()) {
      return false;
    }
    previous_points.emplace_back(static_cast<float>(track.current.pixel.x()),
                                 static_cast<float>(track.current.pixel.y()));
  }

  std::vector<cv::Point2f> current_points;
  std::vector<unsigned char> forward_status;
  std::vector<float> forward_error;
  const cv::TermCriteria termination(
      cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
      options_.maximum_iterations, options_.termination_epsilon);
  try {
    cv::calcOpticalFlowPyrLK(
        previous_image, current_image, previous_points, current_points,
        forward_status, forward_error, options_.window_size,
        options_.pyramid_levels, termination);
  } catch (const cv::Exception&) {
    return false;
  }

  std::vector<cv::Point2f> backward_input;
  std::vector<std::size_t> backward_indices;
  backward_input.reserve(input_tracks.size());
  backward_indices.reserve(input_tracks.size());
  for (std::size_t i = 0; i < input_tracks.size(); ++i) {
    if (i < forward_status.size() && forward_status[i] != 0U &&
        i < current_points.size() && finitePoint(current_points[i]) &&
        insideBorder(current_points[i], current_image.cols, current_image.rows,
                     options_.border_margin) &&
        i < forward_error.size() && std::isfinite(forward_error[i]) &&
        forward_error[i] <= options_.maximum_lk_error) {
      backward_input.push_back(current_points[i]);
      backward_indices.push_back(i);
    }
  }

  std::vector<cv::Point2f> backward_points;
  std::vector<unsigned char> backward_status;
  std::vector<float> backward_error;
  if (!backward_input.empty()) {
    try {
      cv::calcOpticalFlowPyrLK(
          current_image, previous_image, backward_input, backward_points,
          backward_status, backward_error, options_.window_size,
          options_.pyramid_levels, termination);
    } catch (const cv::Exception&) {
      return false;
    }
  }

  std::vector<int> backward_lookup(input_tracks.size(), -1);
  for (std::size_t j = 0; j < backward_indices.size(); ++j) {
    backward_lookup[backward_indices[j]] = static_cast<int>(j);
  }

  double forward_backward_sum = 0.0;
  output_tracks->reserve(input_tracks.size());
  for (std::size_t i = 0; i < input_tracks.size(); ++i) {
    if (i >= forward_status.size() || forward_status[i] == 0U ||
        i >= current_points.size() || !finitePoint(current_points[i])) {
      ++statistics->forward_flow_failures;
      continue;
    }
    if (!insideBorder(current_points[i], current_image.cols,
                      current_image.rows, options_.border_margin)) {
      ++statistics->outside_image_rejections;
      continue;
    }
    if (i >= forward_error.size() || !std::isfinite(forward_error[i]) ||
        forward_error[i] > options_.maximum_lk_error) {
      ++statistics->lk_error_rejections;
      continue;
    }

    const int backward_index = backward_lookup[i];
    if (backward_index < 0 ||
        static_cast<std::size_t>(backward_index) >= backward_status.size() ||
        backward_status[static_cast<std::size_t>(backward_index)] == 0U ||
        static_cast<std::size_t>(backward_index) >= backward_points.size() ||
        !finitePoint(backward_points[static_cast<std::size_t>(backward_index)])) {
      ++statistics->backward_flow_failures;
      continue;
    }
    const double forward_backward_error = cv::norm(
        previous_points[i] -
        backward_points[static_cast<std::size_t>(backward_index)]);
    if (!std::isfinite(forward_backward_error) ||
        forward_backward_error > options_.maximum_forward_backward_error) {
      ++statistics->forward_backward_rejections;
      continue;
    }

    FeatureTrack tracked = input_tracks[i];
    FeatureObservation observation;
    observation.timestamp = current_timestamp;
    observation.camera_id = camera_id;
    observation.pixel = Eigen::Vector2d(current_points[i].x,
                                        current_points[i].y);
    observation.tracking_error = forward_error[i];
    if (!rig_camera->model->unproject(observation.pixel,
                                     &observation.bearing_c) ||
        !camera_rig.cameraBearingToBody(camera_id, observation.bearing_c,
                                       &observation.bearing_b)) {
      ++statistics->model_domain_rejections;
      continue;
    }

    tracked.previous = tracked.current;
    tracked.has_previous_observation = true;
    tracked.current = observation;
    tracked.camera_id = camera_id;
    ++tracked.age;
    ++tracked.total_observation_count;
    tracked.newly_detected = false;
    tracked.forward_backward_error = forward_backward_error;
    tracked.lk_error = forward_error[i];
    output_tracks->push_back(std::move(tracked));
    forward_backward_sum += forward_backward_error;
    statistics->maximum_forward_backward_error =
        std::max(statistics->maximum_forward_backward_error,
                 forward_backward_error);
  }

  statistics->successfully_tracked = output_tracks->size();
  if (!output_tracks->empty()) {
    statistics->average_forward_backward_error =
        forward_backward_sum / static_cast<double>(output_tracks->size());
  }
  return statistics->successfully_tracked + statistics->rejectedCount() ==
         statistics->input_tracks;
}

}  // namespace sphere_vio
