#include "sphere_vio/frontend/landmark_track_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace sphere_vio {
namespace {

constexpr double kTimestampTolerance = 1e-9;
constexpr double kUnitBearingTolerance = 1e-6;

bool validOptions(const LandmarkTrackManagerOptions& options) {
  return options.minimum_confirmations_for_active >= 1U &&
         options.retire_after_frames_without_observation >=
             options.maximum_frames_without_observation &&
         options.maximum_observation_history >= 1U;
}

bool sameTimestamp(Timestamp lhs, Timestamp rhs) {
  return std::isfinite(lhs) && std::isfinite(rhs) &&
         std::abs(lhs - rhs) <= kTimestampTolerance;
}

bool unitFiniteBearing(const Eigen::Vector3d& bearing) {
  if (!bearing.allFinite()) return false;
  const double norm = bearing.norm();
  return std::isfinite(norm) &&
         std::abs(norm - 1.0) <= kUnitBearingTolerance;
}

bool validFeatureTrack(const FeatureTrack& track, CameraId camera_id) {
  return track.id != 0U && track.camera_id == camera_id &&
         track.current.camera_id == camera_id &&
         std::isfinite(track.current.timestamp) &&
         track.current.pixel.allFinite() &&
         unitFiniteBearing(track.current.bearing_c) &&
         unitFiniteBearing(track.current.bearing_b) && track.age > 0U;
}

double finiteQuality(double value) {
  return std::isfinite(value) ? value
                              : std::numeric_limits<double>::infinity();
}

bool diagnosticOrder(const TriangulationDiagnostic& lhs,
                     const TriangulationDiagnostic& rhs) {
  const double lhs_epipolar = finiteQuality(lhs.match.epipolar_error_maximum);
  const double rhs_epipolar = finiteQuality(rhs.match.epipolar_error_maximum);
  if (lhs_epipolar != rhs_epipolar) return lhs_epipolar < rhs_epipolar;
  const double lhs_reprojection =
      finiteQuality(lhs.maximum_angular_reprojection_error);
  const double rhs_reprojection =
      finiteQuality(rhs.maximum_angular_reprojection_error);
  if (lhs_reprojection != rhs_reprojection)
    return lhs_reprojection < rhs_reprojection;
  const double lhs_descriptor = finiteQuality(lhs.match.descriptor_distance);
  const double rhs_descriptor = finiteQuality(rhs.match.descriptor_distance);
  if (lhs_descriptor != rhs_descriptor)
    return lhs_descriptor < rhs_descriptor;
  if (lhs.match.camera_id_1 != rhs.match.camera_id_1)
    return lhs.match.camera_id_1 < rhs.match.camera_id_1;
  if (lhs.match.camera_id_2 != rhs.match.camera_id_2)
    return lhs.match.camera_id_2 < rhs.match.camera_id_2;
  if (lhs.match.feature_id_1 != rhs.match.feature_id_1)
    return lhs.match.feature_id_1 < rhs.match.feature_id_1;
  if (lhs.match.feature_id_2 != rhs.match.feature_id_2)
    return lhs.match.feature_id_2 < rhs.match.feature_id_2;
  if (lhs.admitted != rhs.admitted) return lhs.admitted > rhs.admitted;
  return static_cast<int>(lhs.status) < static_cast<int>(rhs.status);
}

void appendUnique(LandmarkTrackId id,
                  std::vector<LandmarkTrackId>* values) {
  if (!values ||
      std::find(values->begin(), values->end(), id) != values->end()) {
    return;
  }
  values->push_back(id);
}

LandmarkObservation makeObservation(
    const FeatureTrack& feature, Timestamp timestamp,
    std::uint64_t frame_index, bool from_confirmation,
    const TriangulationDiagnostic* diagnostic) {
  LandmarkObservation observation;
  observation.timestamp = timestamp;
  observation.frame_index = frame_index;
  observation.feature = TemporalFeatureKey{feature.camera_id, feature.id};
  observation.pixel = feature.current.pixel;
  observation.bearing_c = feature.current.bearing_c;
  observation.bearing_b = feature.current.bearing_b;
  observation.feature_age = feature.age;
  observation.from_cross_camera_confirmation = from_confirmation;
  if (diagnostic) {
    observation.descriptor_distance =
        diagnostic->match.descriptor_distance;
    observation.epipolar_error =
        diagnostic->match.epipolar_error_maximum;
  }
  return observation;
}

bool sameObservation(const LandmarkObservation& lhs,
                     const LandmarkObservation& rhs) {
  return sameTimestamp(lhs.timestamp, rhs.timestamp) &&
         lhs.feature == rhs.feature;
}

bool appendObservation(const LandmarkObservation& observation,
                       std::size_t maximum_history,
                       LandmarkTrack* track) {
  if (!track) return false;
  for (LandmarkObservation& existing : track->observations) {
    if (!sameObservation(existing, observation)) continue;
    if (observation.from_cross_camera_confirmation &&
        !existing.from_cross_camera_confirmation) {
      existing = observation;
    }
    return false;
  }
  track->observations.push_back(observation);
  ++track->total_observation_count;
  track->last_observation_timestamp = observation.timestamp;
  track->last_observation_frame_index = observation.frame_index;
  while (track->observations.size() > maximum_history)
    track->observations.pop_front();
  return true;
}

bool validDiagnosticMatch(const TriangulationDiagnostic& diagnostic) {
  const CrossCameraMatch& match = diagnostic.match;
  return match.camera_id_1 < 4U && match.camera_id_2 < 4U &&
         match.feature_id_1 != 0U && match.feature_id_2 != 0U &&
         match.pixel_1.allFinite() && match.pixel_2.allFinite() &&
         std::isfinite(match.descriptor_distance) &&
         std::isfinite(match.epipolar_error_maximum) &&
         std::isfinite(diagnostic.maximum_angular_reprojection_error);
}

}  // namespace

const char* landmarkTrackStateName(LandmarkTrackState state) {
  switch (state) {
    case LandmarkTrackState::kTentative:
      return "tentative";
    case LandmarkTrackState::kActive:
      return "active";
    case LandmarkTrackState::kStale:
      return "stale";
    case LandmarkTrackState::kRetired:
      return "retired";
    case LandmarkTrackState::kCount:
      return "count";
  }
  return "unknown";
}

const char* landmarkAssociationStatusName(LandmarkAssociationStatus status) {
  switch (status) {
    case LandmarkAssociationStatus::kCreated:
      return "created";
    case LandmarkAssociationStatus::kUpdatedExisting:
      return "updated_existing";
    case LandmarkAssociationStatus::kAttachedFirstFeature:
      return "attached_first_feature";
    case LandmarkAssociationStatus::kAttachedSecondFeature:
      return "attached_second_feature";
    case LandmarkAssociationStatus::kRejectedInvalidInput:
      return "rejected_invalid_input";
    case LandmarkAssociationStatus::kRejectedNotAdmitted:
      return "rejected_not_admitted";
    case LandmarkAssociationStatus::kRejectedMissingActiveFeature:
      return "rejected_missing_active_feature";
    case LandmarkAssociationStatus::kRejectedTimestampMismatch:
      return "rejected_timestamp_mismatch";
    case LandmarkAssociationStatus::kRejectedSameCamera:
      return "rejected_same_camera";
    case LandmarkAssociationStatus::kSameCameraMemberConflict:
      return "same_camera_member_conflict";
    case LandmarkAssociationStatus::kDifferentLandmarkConflict:
      return "different_landmark_conflict";
    case LandmarkAssociationStatus::kTrackRetired:
      return "track_retired";
    case LandmarkAssociationStatus::kCount:
      return "count";
  }
  return "unknown";
}

LandmarkTrackManager::LandmarkTrackManager(
    LandmarkTrackManagerOptions options, LandmarkTrackId initial_next_id)
    : options_(options), next_landmark_track_id_(initial_next_id) {}

bool LandmarkTrackManager::processFrame(
    const LandmarkTrackFrameInput& input,
    LandmarkTrackFrameResult* result) {
  if (!result || !validOptions(options_) ||
      !std::isfinite(input.timestamp) || input.frame_index == 0U ||
      (has_previous_frame_ &&
       (input.frame_index <= previous_frame_index_ ||
        input.timestamp <= previous_timestamp_))) {
    return false;
  }
  const auto start = std::chrono::steady_clock::now();

  std::map<TemporalFeatureKey, const FeatureTrack*> active_features;
  for (CameraId camera_id = 0U; camera_id < input.camera_tracking.size();
       ++camera_id) {
    const CameraTrackingResult& camera = input.camera_tracking[camera_id];
    if (camera.camera_id != camera_id || !std::isfinite(camera.timestamp) ||
        !sameTimestamp(camera.timestamp, input.timestamp)) {
      return false;
    }
    for (const FeatureTrack& feature : camera.tracks) {
      if (!validFeatureTrack(feature, camera_id)) return false;
      const TemporalFeatureKey key{camera_id, feature.id};
      if (!active_features.emplace(key, &feature).second) return false;
    }
  }

  LandmarkTrackFrameResult next_result;
  next_result.timestamp = input.timestamp;
  next_result.frame_index = input.frame_index;
  std::vector<TriangulationDiagnostic> diagnostics =
      input.triangulation_diagnostics;
  std::stable_sort(diagnostics.begin(), diagnostics.end(), diagnosticOrder);

  for (const TriangulationDiagnostic& diagnostic : diagnostics) {
    LandmarkAssociationResult association;
    association.match = diagnostic.match;
    const CrossCameraMatch& match = diagnostic.match;
    if (!validDiagnosticMatch(diagnostic)) {
      association.status =
          LandmarkAssociationStatus::kRejectedInvalidInput;
      next_result.associations.push_back(std::move(association));
      continue;
    }
    if (!diagnostic.admitted ||
        diagnostic.status != TriangulationCandidateStatus::kAccepted) {
      association.status =
          LandmarkAssociationStatus::kRejectedNotAdmitted;
      next_result.associations.push_back(std::move(association));
      continue;
    }
    if (match.camera_id_1 == match.camera_id_2) {
      association.status = LandmarkAssociationStatus::kRejectedSameCamera;
      next_result.associations.push_back(std::move(association));
      continue;
    }

    const TemporalFeatureKey first_key{match.camera_id_1,
                                       match.feature_id_1};
    const TemporalFeatureKey second_key{match.camera_id_2,
                                        match.feature_id_2};
    const auto first_active = active_features.find(first_key);
    const auto second_active = active_features.find(second_key);
    if (first_active == active_features.end() ||
        second_active == active_features.end()) {
      association.status =
          LandmarkAssociationStatus::kRejectedMissingActiveFeature;
      next_result.associations.push_back(std::move(association));
      continue;
    }
    if (!sameTimestamp(first_active->second->current.timestamp,
                       input.timestamp) ||
        !sameTimestamp(second_active->second->current.timestamp,
                       input.timestamp)) {
      association.status =
          LandmarkAssociationStatus::kRejectedTimestampMismatch;
      next_result.associations.push_back(std::move(association));
      continue;
    }
    const auto retired_first = retired_features_.find(first_key);
    const auto retired_second = retired_features_.find(second_key);
    if (retired_first != retired_features_.end() ||
        retired_second != retired_features_.end()) {
      association.status = LandmarkAssociationStatus::kTrackRetired;
      association.has_landmark_track_id = true;
      association.landmark_track_id =
          retired_first != retired_features_.end() ? retired_first->second
                                                    : retired_second->second;
      next_result.associations.push_back(std::move(association));
      continue;
    }

    const auto first_assignment = feature_to_landmark_.find(first_key);
    const auto second_assignment = feature_to_landmark_.find(second_key);
    const bool has_first = first_assignment != feature_to_landmark_.end();
    const bool has_second = second_assignment != feature_to_landmark_.end();
    LandmarkTrack* target = nullptr;

    if (!has_first && !has_second) {
      if (next_landmark_track_id_.value == 0U ||
          next_landmark_track_id_.value ==
              std::numeric_limits<std::uint64_t>::max()) {
        association.status =
            LandmarkAssociationStatus::kRejectedInvalidInput;
        next_result.associations.push_back(std::move(association));
        continue;
      }
      LandmarkTrack track;
      track.id = next_landmark_track_id_;
      ++next_landmark_track_id_.value;
      track.creation_timestamp = input.timestamp;
      track.last_observation_timestamp = input.timestamp;
      track.last_cross_camera_confirmation_timestamp = input.timestamp;
      track.creation_frame_index = input.frame_index;
      track.last_observation_frame_index = input.frame_index;
      track.last_confirmation_frame_index = input.frame_index;
      track.member_features.emplace(first_key.camera_id, first_key);
      track.member_features.emplace(second_key.camera_id, second_key);
      track.maximum_simultaneous_camera_count = 2U;
      const LandmarkTrackId id = track.id;
      tracks_.emplace(id, std::move(track));
      feature_to_landmark_.emplace(first_key, id);
      feature_to_landmark_.emplace(second_key, id);
      target = &tracks_.find(id)->second;
      association.status = LandmarkAssociationStatus::kCreated;
      association.has_landmark_track_id = true;
      association.landmark_track_id = id;
      appendUnique(id, &next_result.created);
    } else if (has_first && has_second) {
      if (first_assignment->second != second_assignment->second) {
        association.status =
            LandmarkAssociationStatus::kDifferentLandmarkConflict;
        ++next_result.association_conflicts;
        next_result.associations.push_back(std::move(association));
        continue;
      }
      target = &tracks_.find(first_assignment->second)->second;
      association.status = LandmarkAssociationStatus::kUpdatedExisting;
      association.has_landmark_track_id = true;
      association.landmark_track_id = target->id;
      appendUnique(target->id, &next_result.updated);
    } else {
      const LandmarkTrackId id =
          has_first ? first_assignment->second : second_assignment->second;
      target = &tracks_.find(id)->second;
      const TemporalFeatureKey& key_to_attach =
          has_first ? second_key : first_key;
      const auto camera_member =
          target->member_features.find(key_to_attach.camera_id);
      if (camera_member != target->member_features.end() &&
          camera_member->second != key_to_attach) {
        association.status =
            LandmarkAssociationStatus::kSameCameraMemberConflict;
        association.has_landmark_track_id = true;
        association.landmark_track_id = id;
        ++next_result.association_conflicts;
        next_result.associations.push_back(std::move(association));
        continue;
      }
      target->member_features.emplace(key_to_attach.camera_id,
                                      key_to_attach);
      feature_to_landmark_.emplace(key_to_attach, id);
      target->maximum_simultaneous_camera_count = std::max(
          target->maximum_simultaneous_camera_count,
          target->member_features.size());
      association.status = has_first
                               ? LandmarkAssociationStatus::
                                     kAttachedSecondFeature
                               : LandmarkAssociationStatus::
                                     kAttachedFirstFeature;
      association.has_landmark_track_id = true;
      association.landmark_track_id = id;
      appendUnique(id, &next_result.updated);
    }

    if (!target || target->state == LandmarkTrackState::kRetired) {
      association.status = LandmarkAssociationStatus::kTrackRetired;
      next_result.associations.push_back(std::move(association));
      continue;
    }
    appendObservation(makeObservation(*first_active->second, input.timestamp,
                                      input.frame_index, true, &diagnostic),
                      options_.maximum_observation_history, target);
    appendObservation(makeObservation(*second_active->second, input.timestamp,
                                      input.frame_index, true, &diagnostic),
                      options_.maximum_observation_history, target);
    ++target->cross_camera_confirmation_count;
    if (target->distinct_confirmation_frame_count == 0U ||
        target->last_confirmation_frame_index != input.frame_index) {
      ++target->distinct_confirmation_frame_count;
    }
    target->last_cross_camera_confirmation_timestamp = input.timestamp;
    target->last_confirmation_frame_index = input.frame_index;
    target->confirmation_stale = false;
    target->has_latest_triangulation_diagnostic = true;
    target->latest_triangulation_diagnostic = diagnostic;

    if (target->state == LandmarkTrackState::kStale) {
      target->state = LandmarkTrackState::kTentative;
    }
    if (target->state == LandmarkTrackState::kTentative &&
        target->member_features.size() >= 2U &&
        target->distinct_confirmation_frame_count >=
            options_.minimum_confirmations_for_active) {
      target->state = LandmarkTrackState::kActive;
      if (!target->ever_active) {
        target->ever_active = true;
        target->has_activation_frame_index = true;
        target->activation_frame_index = input.frame_index;
      }
      appendUnique(target->id, &next_result.activated);
    }
    next_result.associations.push_back(std::move(association));
  }

  // Same-camera LK continuation contributes observations, never confirmation.
  for (auto& entry : tracks_) {
    LandmarkTrack& track = entry.second;
    if (track.state == LandmarkTrackState::kRetired) continue;
    for (const auto& member : track.member_features) {
      const auto active = active_features.find(member.second);
      if (active == active_features.end() ||
          !sameTimestamp(active->second->current.timestamp, input.timestamp)) {
        continue;
      }
      appendObservation(makeObservation(*active->second, input.timestamp,
                                        input.frame_index, false, nullptr),
                        options_.maximum_observation_history, &track);
    }
  }

  for (auto& entry : tracks_) {
    LandmarkTrack& track = entry.second;
    if (track.state == LandmarkTrackState::kRetired) continue;
    const std::uint64_t frames_without_observation =
        input.frame_index - track.last_observation_frame_index;
    const std::uint64_t frames_without_confirmation =
        input.frame_index - track.last_confirmation_frame_index;
    track.confirmation_stale =
        frames_without_confirmation >
        options_.maximum_frames_without_confirmation;
    if (frames_without_observation >
            options_.maximum_frames_without_observation &&
        track.state != LandmarkTrackState::kStale) {
      track.state = LandmarkTrackState::kStale;
      appendUnique(track.id, &next_result.marked_stale);
    }
    if (frames_without_observation >
        options_.retire_after_frames_without_observation) {
      track.state = LandmarkTrackState::kRetired;
      track.has_retirement_frame_index = true;
      track.retirement_frame_index = input.frame_index;
      for (const auto& member : track.member_features) {
        const auto assignment = feature_to_landmark_.find(member.second);
        if (assignment != feature_to_landmark_.end() &&
            assignment->second == track.id) {
          feature_to_landmark_.erase(assignment);
        }
        retired_features_[member.second] = track.id;
      }
      track.observations.clear();
      track.has_latest_triangulation_diagnostic = false;
      track.latest_triangulation_diagnostic = TriangulationDiagnostic{};
      appendUnique(track.id, &next_result.retired);
    }
  }

  if (!checkConsistency()) return false;
  has_previous_frame_ = true;
  previous_timestamp_ = input.timestamp;
  previous_frame_index_ = input.frame_index;
  next_result.processing_time_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  *result = std::move(next_result);
  return true;
}

const LandmarkTrack* LandmarkTrackManager::track(LandmarkTrackId id) const {
  const auto found = tracks_.find(id);
  return found == tracks_.end() ? nullptr : &found->second;
}

bool LandmarkTrackManager::landmarkForFeature(
    const TemporalFeatureKey& feature, LandmarkTrackId* id) const {
  if (!id) return false;
  const auto found = feature_to_landmark_.find(feature);
  if (found == feature_to_landmark_.end()) return false;
  *id = found->second;
  return true;
}

std::vector<LandmarkTrack> LandmarkTrackManager::activeTracks() const {
  std::vector<LandmarkTrack> output;
  for (const auto& entry : tracks_) {
    if (entry.second.state != LandmarkTrackState::kRetired)
      output.push_back(entry.second);
  }
  return output;
}

std::vector<LandmarkTrack> LandmarkTrackManager::allTracks() const {
  std::vector<LandmarkTrack> output;
  output.reserve(tracks_.size());
  for (const auto& entry : tracks_) output.push_back(entry.second);
  return output;
}

std::size_t LandmarkTrackManager::liveTrackCount() const {
  std::size_t count = 0U;
  for (const auto& entry : tracks_) {
    if (entry.second.state != LandmarkTrackState::kRetired) ++count;
  }
  return count;
}

bool LandmarkTrackManager::checkConsistency() const {
  std::set<TemporalFeatureKey> live_features;
  for (const auto& entry : tracks_) {
    const LandmarkTrack& track = entry.second;
    if (track.id != entry.first || track.id.value == 0U) return false;
    std::set<CameraId> cameras;
    for (const auto& member : track.member_features) {
      if (member.first != member.second.camera_id ||
          member.second.camera_id >= 4U || member.second.feature_id == 0U ||
          !cameras.insert(member.first).second) {
        return false;
      }
      if (track.state != LandmarkTrackState::kRetired) {
        if (!live_features.insert(member.second).second) return false;
        const auto assignment = feature_to_landmark_.find(member.second);
        if (assignment == feature_to_landmark_.end() ||
            assignment->second != track.id) {
          return false;
        }
      }
    }
    if (track.observations.size() > options_.maximum_observation_history)
      return false;
  }
  for (const auto& assignment : feature_to_landmark_) {
    const auto found = tracks_.find(assignment.second);
    if (found == tracks_.end() ||
        found->second.state == LandmarkTrackState::kRetired) {
      return false;
    }
    const auto member =
        found->second.member_features.find(assignment.first.camera_id);
    if (member == found->second.member_features.end() ||
        member->second != assignment.first) {
      return false;
    }
  }
  return true;
}

void LandmarkTrackManager::reset() {
  feature_to_landmark_.clear();
  retired_features_.clear();
  tracks_.clear();
  has_previous_frame_ = false;
  previous_timestamp_ = 0.0;
  previous_frame_index_ = 0U;
}

}  // namespace sphere_vio
