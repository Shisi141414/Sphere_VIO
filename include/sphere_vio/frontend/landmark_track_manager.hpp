#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "sphere_vio/frontend/landmark_track.hpp"
#include "sphere_vio/frontend/temporal_frontend.hpp"

namespace sphere_vio {

struct LandmarkTrackManagerOptions {
  std::uint64_t minimum_confirmations_for_active = 2U;
  std::uint64_t maximum_frames_without_observation = 5U;
  std::uint64_t maximum_frames_without_confirmation = 30U;
  std::uint64_t retire_after_frames_without_observation = 60U;
  std::size_t maximum_observation_history = 40U;
};

enum class LandmarkAssociationStatus {
  kCreated = 0,
  kUpdatedExisting,
  kAttachedFirstFeature,
  kAttachedSecondFeature,
  kRejectedInvalidInput,
  kRejectedNotAdmitted,
  kRejectedMissingActiveFeature,
  kRejectedTimestampMismatch,
  kRejectedSameCamera,
  kSameCameraMemberConflict,
  kDifferentLandmarkConflict,
  kTrackRetired,
  kCount
};

const char* landmarkAssociationStatusName(LandmarkAssociationStatus status);

struct LandmarkAssociationResult {
  LandmarkAssociationStatus status =
      LandmarkAssociationStatus::kRejectedInvalidInput;
  CrossCameraMatch match;
  bool has_landmark_track_id = false;
  LandmarkTrackId landmark_track_id;
};

struct LandmarkTrackFrameInput {
  Timestamp timestamp = 0.0;
  std::uint64_t frame_index = 0U;
  std::array<CameraTrackingResult, 4> camera_tracking;
  std::vector<TriangulationDiagnostic> triangulation_diagnostics;
};

struct LandmarkTrackFrameResult {
  Timestamp timestamp = 0.0;
  std::uint64_t frame_index = 0U;
  std::vector<LandmarkAssociationResult> associations;
  std::vector<LandmarkTrackId> created;
  std::vector<LandmarkTrackId> updated;
  std::vector<LandmarkTrackId> activated;
  std::vector<LandmarkTrackId> marked_stale;
  std::vector<LandmarkTrackId> retired;
  std::size_t association_conflicts = 0U;
  double processing_time_seconds = 0.0;
};

class LandmarkTrackManager {
 public:
  explicit LandmarkTrackManager(
      LandmarkTrackManagerOptions options = {},
      LandmarkTrackId initial_next_id = LandmarkTrackId{1U});

  const LandmarkTrackManagerOptions& options() const { return options_; }

  bool processFrame(const LandmarkTrackFrameInput& input,
                    LandmarkTrackFrameResult* result);

  const LandmarkTrack* track(LandmarkTrackId id) const;
  bool landmarkForFeature(const TemporalFeatureKey& feature,
                          LandmarkTrackId* id) const;
  std::vector<LandmarkTrack> activeTracks() const;
  std::vector<LandmarkTrack> allTracks() const;
  std::size_t liveTrackCount() const;
  bool checkConsistency() const;

  // Clears hypotheses but deliberately does not rewind the monotonic id.
  void reset();

 private:
  LandmarkTrackManagerOptions options_;
  LandmarkTrackId next_landmark_track_id_;
  std::map<TemporalFeatureKey, LandmarkTrackId> feature_to_landmark_;
  std::map<TemporalFeatureKey, LandmarkTrackId> retired_features_;
  std::map<LandmarkTrackId, LandmarkTrack> tracks_;
  bool has_previous_frame_ = false;
  Timestamp previous_timestamp_ = 0.0;
  std::uint64_t previous_frame_index_ = 0U;
};

}  // namespace sphere_vio
