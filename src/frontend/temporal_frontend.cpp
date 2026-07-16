#include "sphere_vio/frontend/temporal_frontend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace sphere_vio {
namespace {

constexpr std::size_t kCameraCount = 4U;

void computeAgeStatistics(CameraTrackingResult* result) {
  if (!result || result->tracks.empty()) return;
  std::uint64_t age_sum = 0U;
  for (const FeatureTrack& track : result->tracks) {
    age_sum += track.age;
    result->maximum_track_age =
        std::max(result->maximum_track_age, track.age);
  }
  result->average_track_age =
      static_cast<double>(age_sum) / static_cast<double>(result->tracks.size());
}

}  // namespace

TemporalFrontend::TemporalFrontend(TemporalFrontendOptions options)
    : options_(std::move(options)),
      detector_(options_.detector),
      tracker_(options_.tracker) {}

bool TemporalFrontend::processFrame(const MultiCameraFrame& frame,
                                    const CameraRig& camera_rig,
                                    MultiCameraTrackingResult* result) {
  if (!result || !std::isfinite(frame.timestamp) ||
      frame.images.size() != kCameraCount || camera_rig.size() < kCameraCount ||
      !std::isfinite(options_.redetection_ratio) ||
      options_.redetection_ratio < 0.0 || options_.redetection_ratio > 1.0) {
    return false;
  }

  std::array<const ImageFrame*, kCameraCount> images{{nullptr, nullptr, nullptr,
                                                       nullptr}};
  for (const ImageFrame& image : frame.images) {
    if (image.camera_id >= kCameraCount || images[image.camera_id] != nullptr ||
        image.image.empty() || image.image.type() != CV_8UC1 ||
        !std::isfinite(image.timestamp) ||
        !camera_rig.hasCamera(image.camera_id)) {
      return false;
    }
    images[image.camera_id] = &image;
  }
  for (CameraId camera_id = 0U; camera_id < kCameraCount; ++camera_id) {
    if (!images[camera_id]) return false;
    const CameraTrackingState& state = states_[camera_id];
    const bool same_size =
        !state.initialized || state.previous_image.size() ==
                                  images[camera_id]->image.size();
    if (state.initialized && same_size &&
        images[camera_id]->timestamp <= state.previous_timestamp) {
      return false;
    }
  }

  std::array<CameraTrackingState, kCameraCount> next_states = states_;
  FeatureId next_feature_id = next_feature_id_;
  MultiCameraTrackingResult next_result;
  next_result.timestamp = frame.timestamp;

  for (CameraId camera_id = 0U; camera_id < kCameraCount; ++camera_id) {
    const auto start = std::chrono::steady_clock::now();
    const ImageFrame& image = *images[camera_id];
    CameraTrackingState& state = next_states[camera_id];
    CameraTrackingResult& camera_result = next_result.cameras[camera_id];
    camera_result.camera_id = camera_id;
    camera_result.timestamp = image.timestamp;

    const bool size_changed =
        state.initialized && state.previous_image.size() != image.image.size();
    if (!state.initialized || size_changed) {
      if (size_changed) {
        camera_result.reset_due_to_image_size = true;
        camera_result.input_tracks = state.tracks.size();
        camera_result.rejected_tracks = state.tracks.size();
      }
      state = CameraTrackingState{};
      std::vector<FeatureObservation> observations;
      if (!detector_.detect(image.image, image.timestamp, camera_id, camera_rig,
                            {}, options_.detector.maximum_features,
                            &observations,
                            &camera_result.detection_statistics)) {
        return false;
      }
      camera_result.tracks.reserve(observations.size());
      for (const FeatureObservation& observation : observations) {
        if (next_feature_id == std::numeric_limits<FeatureId>::max())
          return false;
        FeatureTrack track;
        track.id = next_feature_id++;
        track.camera_id = camera_id;
        track.current = observation;
        track.age = 1U;
        track.total_observation_count = 1U;
        track.newly_detected = true;
        camera_result.tracks.push_back(std::move(track));
      }
      camera_result.newly_detected = camera_result.tracks.size();
      camera_result.model_domain_rejections =
          camera_result.detection_statistics.model_domain_rejections;
      camera_result.initialized_this_frame = true;
    } else {
      camera_result.input_tracks = state.tracks.size();
      if (!tracker_.track(state.previous_image, image.image, image.timestamp,
                          camera_id, camera_rig, state.tracks,
                          &camera_result.tracks,
                          &camera_result.tracking_statistics)) {
        return false;
      }
      camera_result.successfully_tracked = camera_result.tracks.size();
      camera_result.rejected_tracks =
          camera_result.tracking_statistics.rejectedCount();
      camera_result.average_forward_backward_error =
          camera_result.tracking_statistics.average_forward_backward_error;
      camera_result.maximum_forward_backward_error =
          camera_result.tracking_statistics.maximum_forward_backward_error;
      camera_result.model_domain_rejections =
          camera_result.tracking_statistics.model_domain_rejections;

      const double redetection_threshold =
          options_.redetection_ratio *
          static_cast<double>(options_.detector.maximum_features);
      if (static_cast<double>(camera_result.tracks.size()) <
              redetection_threshold &&
          camera_result.tracks.size() < options_.detector.maximum_features) {
        std::vector<Eigen::Vector2d> occupied;
        occupied.reserve(camera_result.tracks.size());
        for (const FeatureTrack& track : camera_result.tracks)
          occupied.push_back(track.current.pixel);

        std::vector<FeatureObservation> observations;
        const std::size_t capacity =
            options_.detector.maximum_features - camera_result.tracks.size();
        if (!detector_.detect(image.image, image.timestamp, camera_id,
                              camera_rig, occupied, capacity, &observations,
                              &camera_result.detection_statistics)) {
          return false;
        }
        for (const FeatureObservation& observation : observations) {
          if (next_feature_id == std::numeric_limits<FeatureId>::max())
            return false;
          FeatureTrack track;
          track.id = next_feature_id++;
          track.camera_id = camera_id;
          track.current = observation;
          track.age = 1U;
          track.total_observation_count = 1U;
          track.newly_detected = true;
          camera_result.tracks.push_back(std::move(track));
        }
        camera_result.newly_detected = observations.size();
        camera_result.model_domain_rejections +=
            camera_result.detection_statistics.model_domain_rejections;
      }
    }

    computeAgeStatistics(&camera_result);
    state.initialized = true;
    state.previous_timestamp = image.timestamp;
    state.previous_image = image.image.clone();
    state.tracks = camera_result.tracks;
    ++state.frame_count;
    camera_result.processing_time_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
  }

  states_ = std::move(next_states);
  next_feature_id_ = next_feature_id;
  *result = std::move(next_result);
  return true;
}

void TemporalFrontend::reset() {
  for (CameraTrackingState& state : states_) state = CameraTrackingState{};
}

void TemporalFrontend::resetCamera(CameraId camera_id) {
  if (camera_id < states_.size()) states_[camera_id] = CameraTrackingState{};
}

}  // namespace sphere_vio
