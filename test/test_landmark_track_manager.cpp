#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "sphere_vio/frontend/landmark_track_manager.hpp"

namespace sphere_vio {
namespace {

struct FeatureSpec {
  CameraId camera_id;
  FeatureId feature_id;
  std::uint32_t age = 1U;
  double timestamp_offset = 0.0;
};

FeatureTrack makeFeature(const FeatureSpec& spec, Timestamp timestamp) {
  FeatureTrack feature;
  feature.id = spec.feature_id;
  feature.camera_id = spec.camera_id;
  feature.age = spec.age;
  feature.total_observation_count = spec.age;
  feature.current.timestamp = timestamp + spec.timestamp_offset;
  feature.current.camera_id = spec.camera_id;
  feature.current.pixel = Eigen::Vector2d(10.0 + spec.feature_id,
                                          20.0 + spec.camera_id);
  feature.current.bearing_c = Eigen::Vector3d(0.1, 0.2, 1.0).normalized();
  feature.current.bearing_b = Eigen::Vector3d(0.2, 0.1, 1.0).normalized();
  return feature;
}

TriangulationDiagnostic makeDiagnostic(CameraId camera_1, FeatureId feature_1,
                                       CameraId camera_2, FeatureId feature_2,
                                       double epipolar = 1e-4,
                                       double reprojection = 2e-4,
                                       double descriptor = 10.0) {
  TriangulationDiagnostic diagnostic;
  diagnostic.admitted = true;
  diagnostic.triangulation_succeeded = true;
  diagnostic.status = TriangulationCandidateStatus::kAccepted;
  diagnostic.match.camera_id_1 = camera_1;
  diagnostic.match.camera_id_2 = camera_2;
  diagnostic.match.feature_id_1 = feature_1;
  diagnostic.match.feature_id_2 = feature_2;
  diagnostic.match.pixel_1 = Eigen::Vector2d(10.0 + feature_1,
                                              20.0 + camera_1);
  diagnostic.match.pixel_2 = Eigen::Vector2d(10.0 + feature_2,
                                              20.0 + camera_2);
  diagnostic.match.descriptor_distance = descriptor;
  diagnostic.match.ratio = 0.5;
  diagnostic.match.epipolar_error_forward = epipolar;
  diagnostic.match.epipolar_error_backward = epipolar;
  diagnostic.match.epipolar_error_maximum = epipolar;
  diagnostic.epipolar_error = epipolar;
  diagnostic.maximum_angular_reprojection_error = reprojection;
  return diagnostic;
}

LandmarkTrackFrameInput makeInput(
    std::uint64_t frame_index, Timestamp timestamp,
    const std::vector<FeatureSpec>& features,
    std::vector<TriangulationDiagnostic> diagnostics = {}) {
  LandmarkTrackFrameInput input;
  input.frame_index = frame_index;
  input.timestamp = timestamp;
  input.triangulation_diagnostics = std::move(diagnostics);
  for (CameraId camera_id = 0U; camera_id < 4U; ++camera_id) {
    input.camera_tracking[camera_id].camera_id = camera_id;
    input.camera_tracking[camera_id].timestamp = timestamp;
  }
  for (const FeatureSpec& spec : features) {
    input.camera_tracking[spec.camera_id].tracks.push_back(
        makeFeature(spec, timestamp));
  }
  return input;
}

LandmarkTrackFrameResult process(
    LandmarkTrackManager* manager, std::uint64_t frame_index,
    const std::vector<FeatureSpec>& features,
    std::vector<TriangulationDiagnostic> diagnostics = {}) {
  LandmarkTrackFrameResult result;
  EXPECT_TRUE(manager->processFrame(
      makeInput(frame_index, static_cast<double>(frame_index), features,
                std::move(diagnostics)),
      &result));
  return result;
}

const LandmarkTrack& onlyTrack(const LandmarkTrackManager& manager) {
  const std::vector<LandmarkTrack> tracks = manager.allTracks();
  EXPECT_EQ(1U, tracks.size());
  return *manager.track(tracks.front().id);
}

TEST(LandmarkTrackManagerTest, TwoUnassignedFeaturesCreateTentativeTrack) {
  LandmarkTrackManager manager;
  const auto result = process(
      &manager, 1U, {{0U, 10U}, {1U, 20U}},
      {makeDiagnostic(0U, 10U, 1U, 20U)});
  ASSERT_EQ(1U, result.created.size());
  const LandmarkTrack* track = manager.track(result.created.front());
  ASSERT_NE(nullptr, track);
  EXPECT_EQ(LandmarkTrackState::kTentative, track->state);
  EXPECT_EQ(2U, track->member_features.size());
  EXPECT_EQ(2U, track->total_observation_count);
  EXPECT_EQ(1U, track->cross_camera_confirmation_count);
}

TEST(LandmarkTrackManagerTest, ExistingTrackAttachesThirdCameraFeature) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  const auto result = process(
      &manager, 2U, {{0U, 10U, 2U}, {1U, 20U, 2U}, {2U, 30U}},
      {makeDiagnostic(0U, 10U, 2U, 30U)});
  ASSERT_EQ(1U, result.associations.size());
  EXPECT_EQ(LandmarkAssociationStatus::kAttachedSecondFeature,
            result.associations[0].status);
  EXPECT_EQ(3U, onlyTrack(manager).member_features.size());
}

TEST(LandmarkTrackManagerTest, ExistingPairUpdatesWithoutDuplicateMembers) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  const auto result = process(
      &manager, 2U, {{0U, 10U, 2U}, {1U, 20U, 2U}},
      {makeDiagnostic(0U, 10U, 1U, 20U)});
  EXPECT_EQ(LandmarkAssociationStatus::kUpdatedExisting,
            result.associations[0].status);
  EXPECT_EQ(2U, onlyTrack(manager).member_features.size());
  EXPECT_EQ(2U, onlyTrack(manager).cross_camera_confirmation_count);
}

TEST(LandmarkTrackManagerTest, DifferentTracksConflictWithoutMerge) {
  LandmarkTrackManager manager;
  process(&manager, 1U,
          {{0U, 10U}, {1U, 20U}, {2U, 30U}, {3U, 40U}},
          {makeDiagnostic(0U, 10U, 1U, 20U),
           makeDiagnostic(2U, 30U, 3U, 40U)});
  const auto result = process(
      &manager, 2U,
      {{0U, 10U, 2U}, {1U, 20U, 2U}, {2U, 30U, 2U}, {3U, 40U, 2U}},
      {makeDiagnostic(0U, 10U, 2U, 30U)});
  EXPECT_EQ(LandmarkAssociationStatus::kDifferentLandmarkConflict,
            result.associations[0].status);
  EXPECT_EQ(1U, result.association_conflicts);
  EXPECT_EQ(2U, manager.allTracks().size());
}

TEST(LandmarkTrackManagerTest, SameCameraSecondMemberConflicts) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  const auto result = process(
      &manager, 2U, {{0U, 10U, 2U}, {0U, 11U}, {1U, 20U, 2U}},
      {makeDiagnostic(0U, 11U, 1U, 20U)});
  EXPECT_EQ(LandmarkAssociationStatus::kSameCameraMemberConflict,
            result.associations[0].status);
  EXPECT_EQ(2U, onlyTrack(manager).member_features.size());
}

TEST(LandmarkTrackManagerTest, RejectedDiagnosticDoesNotCreateTrack) {
  LandmarkTrackManager manager;
  TriangulationDiagnostic diagnostic = makeDiagnostic(0U, 10U, 1U, 20U);
  diagnostic.admitted = false;
  diagnostic.status = TriangulationCandidateStatus::kNegativeDepth;
  const auto result = process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
                              {diagnostic});
  EXPECT_EQ(LandmarkAssociationStatus::kRejectedNotAdmitted,
            result.associations[0].status);
  EXPECT_TRUE(manager.allTracks().empty());
}

TEST(LandmarkTrackManagerTest, InconsistentAdmittedFlagIsRejected) {
  LandmarkTrackManager manager;
  TriangulationDiagnostic diagnostic = makeDiagnostic(0U, 10U, 1U, 20U);
  diagnostic.status = TriangulationCandidateStatus::kDepthOutOfRange;
  const auto result = process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
                              {diagnostic});
  EXPECT_EQ(LandmarkAssociationStatus::kRejectedNotAdmitted,
            result.associations[0].status);
  EXPECT_TRUE(manager.allTracks().empty());
}

TEST(LandmarkTrackManagerTest, MissingActiveFeatureIsExplicit) {
  LandmarkTrackManager manager;
  const auto result = process(&manager, 1U, {{0U, 10U}},
                              {makeDiagnostic(0U, 10U, 1U, 20U)});
  EXPECT_EQ(LandmarkAssociationStatus::kRejectedMissingActiveFeature,
            result.associations[0].status);
}

TEST(LandmarkTrackManagerTest, FeatureTimestampMismatchIsExplicit) {
  LandmarkTrackManager manager;
  const auto result = process(&manager, 1U,
                              {{0U, 10U}, {1U, 20U, 1U, -0.01}},
                              {makeDiagnostic(0U, 10U, 1U, 20U)});
  EXPECT_EQ(LandmarkAssociationStatus::kRejectedTimestampMismatch,
            result.associations[0].status);
}

TEST(LandmarkTrackManagerTest, FeatureMapsToAtMostOneLiveTrack) {
  LandmarkTrackManager manager;
  const auto result = process(
      &manager, 1U, {{0U, 10U}, {1U, 20U}, {2U, 30U}},
      {makeDiagnostic(0U, 10U, 1U, 20U),
       makeDiagnostic(0U, 10U, 2U, 30U, 2e-4)});
  EXPECT_EQ(1U, manager.allTracks().size());
  LandmarkTrackId first;
  LandmarkTrackId second;
  ASSERT_TRUE(manager.landmarkForFeature({0U, 10U}, &first));
  ASSERT_TRUE(manager.landmarkForFeature({2U, 30U}, &second));
  EXPECT_EQ(first, second);
  EXPECT_TRUE(manager.checkConsistency());
  EXPECT_EQ(1U, result.created.size());
}

TEST(LandmarkTrackManagerTest, TrackHasAtMostOneMemberPerCamera) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  process(&manager, 2U, {{0U, 10U, 2U}, {0U, 11U}, {1U, 20U, 2U}},
          {makeDiagnostic(0U, 11U, 1U, 20U)});
  const LandmarkTrack& track = onlyTrack(manager);
  EXPECT_EQ(1U, track.member_features.count(0U));
  EXPECT_EQ(10U, track.member_features.at(0U).feature_id);
}

TEST(LandmarkTrackManagerTest, ConfirmationAndTemporalUpdateDeduplicate) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  const LandmarkTrack& track = onlyTrack(manager);
  EXPECT_EQ(2U, track.observations.size());
  EXPECT_EQ(2U, track.total_observation_count);
}

TEST(LandmarkTrackManagerTest, TemporalUpdateDoesNotAddConfirmation) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  process(&manager, 2U, {{0U, 10U, 2U}, {1U, 20U, 2U}});
  const LandmarkTrack& track = onlyTrack(manager);
  EXPECT_EQ(4U, track.total_observation_count);
  EXPECT_EQ(1U, track.cross_camera_confirmation_count);
  EXPECT_EQ(1U, track.distinct_confirmation_frame_count);
}

TEST(LandmarkTrackManagerTest, SameFrameConfirmationsCountOneDistinctFrame) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U),
           makeDiagnostic(0U, 10U, 1U, 20U, 2e-4)});
  const LandmarkTrack& track = onlyTrack(manager);
  EXPECT_EQ(2U, track.cross_camera_confirmation_count);
  EXPECT_EQ(1U, track.distinct_confirmation_frame_count);
  EXPECT_EQ(LandmarkTrackState::kTentative, track.state);
}

TEST(LandmarkTrackManagerTest, TwoConfirmationFramesActivateTrack) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  const auto result = process(
      &manager, 2U, {{0U, 10U, 2U}, {1U, 20U, 2U}},
      {makeDiagnostic(0U, 10U, 1U, 20U)});
  EXPECT_EQ(1U, result.activated.size());
  EXPECT_EQ(LandmarkTrackState::kActive, onlyTrack(manager).state);
  EXPECT_TRUE(onlyTrack(manager).ever_active);
}

LandmarkTrackManagerOptions shortLifecycleOptions() {
  LandmarkTrackManagerOptions options;
  options.minimum_confirmations_for_active = 2U;
  options.maximum_frames_without_observation = 0U;
  options.maximum_frames_without_confirmation = 1U;
  options.retire_after_frames_without_observation = 2U;
  options.maximum_observation_history = 10U;
  return options;
}

TEST(LandmarkTrackManagerTest, ActiveTrackBecomesStaleWithoutObservation) {
  LandmarkTrackManager manager(shortLifecycleOptions());
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  process(&manager, 2U, {{0U, 10U, 2U}, {1U, 20U, 2U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  const auto result = process(&manager, 3U, {});
  EXPECT_EQ(1U, result.marked_stale.size());
  EXPECT_EQ(LandmarkTrackState::kStale, onlyTrack(manager).state);
}

TEST(LandmarkTrackManagerTest, AdmittedEdgeRecoversStaleTrack) {
  LandmarkTrackManager manager(shortLifecycleOptions());
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  process(&manager, 2U, {});
  ASSERT_EQ(LandmarkTrackState::kStale, onlyTrack(manager).state);
  const auto result = process(
      &manager, 3U, {{0U, 10U, 3U}, {1U, 20U, 3U}},
      {makeDiagnostic(0U, 10U, 1U, 20U)});
  EXPECT_EQ(LandmarkTrackState::kActive, onlyTrack(manager).state);
  EXPECT_EQ(1U, result.activated.size());
}

TEST(LandmarkTrackManagerTest, StaleTrackEventuallyRetires) {
  LandmarkTrackManager manager(shortLifecycleOptions());
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  process(&manager, 2U, {});
  process(&manager, 3U, {});
  const auto result = process(&manager, 4U, {});
  EXPECT_EQ(1U, result.retired.size());
  EXPECT_EQ(LandmarkTrackState::kRetired, onlyTrack(manager).state);
  EXPECT_TRUE(onlyTrack(manager).observations.empty());
}

TEST(LandmarkTrackManagerTest, RetiredTrackCannotReactivate) {
  LandmarkTrackManager manager(shortLifecycleOptions());
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  process(&manager, 2U, {});
  process(&manager, 3U, {});
  process(&manager, 4U, {});
  const auto result = process(
      &manager, 5U, {{0U, 10U, 5U}, {1U, 20U, 5U}},
      {makeDiagnostic(0U, 10U, 1U, 20U)});
  EXPECT_EQ(LandmarkAssociationStatus::kTrackRetired,
            result.associations[0].status);
  EXPECT_EQ(LandmarkTrackState::kRetired, onlyTrack(manager).state);
}

TEST(LandmarkTrackManagerTest, LandmarkIdsAreMonotonicAndNeverReused) {
  LandmarkTrackManager manager;
  const auto first = process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
                             {makeDiagnostic(0U, 10U, 1U, 20U)});
  const auto second = process(&manager, 2U,
                              {{0U, 10U, 2U}, {1U, 20U, 2U},
                               {2U, 30U}, {3U, 40U}},
                              {makeDiagnostic(2U, 30U, 3U, 40U)});
  EXPECT_LT(first.created[0].value, second.created[0].value);
}

TEST(LandmarkTrackManagerTest, ResetDoesNotRewindLandmarkId) {
  LandmarkTrackManager manager;
  const auto first = process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
                             {makeDiagnostic(0U, 10U, 1U, 20U)});
  manager.reset();
  const auto second = process(&manager, 1U, {{2U, 30U}, {3U, 40U}},
                              {makeDiagnostic(2U, 30U, 3U, 40U)});
  EXPECT_LT(first.created[0].value, second.created[0].value);
}

TEST(LandmarkTrackManagerTest, ObservationHistoryIsBounded) {
  LandmarkTrackManagerOptions options;
  options.maximum_observation_history = 3U;
  LandmarkTrackManager manager(options);
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  process(&manager, 2U, {{0U, 10U, 2U}, {1U, 20U, 2U}});
  process(&manager, 3U, {{0U, 10U, 3U}, {1U, 20U, 3U}});
  EXPECT_EQ(3U, onlyTrack(manager).observations.size());
}

TEST(LandmarkTrackManagerTest, TotalObservationCountSurvivesTrimming) {
  LandmarkTrackManagerOptions options;
  options.maximum_observation_history = 2U;
  LandmarkTrackManager manager(options);
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  process(&manager, 2U, {{0U, 10U, 2U}, {1U, 20U, 2U}});
  EXPECT_EQ(2U, onlyTrack(manager).observations.size());
  EXPECT_EQ(4U, onlyTrack(manager).total_observation_count);
}

TEST(LandmarkTrackManagerTest, DirectEdgesBuildThreeCameraHypothesis) {
  LandmarkTrackManager manager;
  process(&manager, 1U,
          {{0U, 10U}, {1U, 20U}, {2U, 30U}},
          {makeDiagnostic(0U, 10U, 1U, 20U),
           makeDiagnostic(0U, 10U, 2U, 30U, 2e-4)});
  EXPECT_EQ(3U, onlyTrack(manager).member_features.size());
}

TEST(LandmarkTrackManagerTest, DirectEdgesBuildFourCameraHypothesis) {
  LandmarkTrackManager manager;
  process(&manager, 1U,
          {{0U, 10U}, {1U, 20U}, {2U, 30U}, {3U, 40U}},
          {makeDiagnostic(0U, 10U, 1U, 20U),
           makeDiagnostic(0U, 10U, 2U, 30U, 2e-4),
           makeDiagnostic(1U, 20U, 3U, 40U, 3e-4)});
  EXPECT_EQ(4U, onlyTrack(manager).member_features.size());
}

TEST(LandmarkTrackManagerTest, QualitySortingControlsCompetingCandidate) {
  LandmarkTrackManager manager;
  const auto result = process(
      &manager, 1U, {{0U, 10U}, {1U, 20U}, {1U, 21U}},
      {makeDiagnostic(0U, 10U, 1U, 20U, 3e-4),
       makeDiagnostic(0U, 10U, 1U, 21U, 1e-4)});
  ASSERT_EQ(2U, result.associations.size());
  EXPECT_EQ(21U, result.associations[0].match.feature_id_2);
  EXPECT_EQ(LandmarkAssociationStatus::kCreated,
            result.associations[0].status);
  EXPECT_EQ(LandmarkAssociationStatus::kSameCameraMemberConflict,
            result.associations[1].status);
}

TEST(LandmarkTrackManagerTest, ConflictOutputIgnoresInputPermutation) {
  auto run = [](bool reverse) {
    LandmarkTrackManager manager;
    std::vector<TriangulationDiagnostic> diagnostics{
        makeDiagnostic(0U, 10U, 1U, 20U, 3e-4),
        makeDiagnostic(0U, 10U, 1U, 21U, 1e-4)};
    if (reverse) std::reverse(diagnostics.begin(), diagnostics.end());
    return process(&manager, 1U, {{0U, 10U}, {1U, 20U}, {1U, 21U}},
                   diagnostics);
  };
  const auto first = run(false);
  const auto second = run(true);
  ASSERT_EQ(first.associations.size(), second.associations.size());
  for (std::size_t index = 0U; index < first.associations.size(); ++index) {
    EXPECT_EQ(first.associations[index].status,
              second.associations[index].status);
    EXPECT_EQ(first.associations[index].match.feature_id_2,
              second.associations[index].match.feature_id_2);
  }
}

TEST(LandmarkTrackManagerTest, C2C3MappingRemainsExplicit) {
  LandmarkTrackManager manager;
  const auto result = process(&manager, 1U, {{2U, 30U}, {3U, 40U}},
                              {makeDiagnostic(2U, 30U, 3U, 40U)});
  const LandmarkTrack* track = manager.track(result.created[0]);
  ASSERT_NE(nullptr, track);
  EXPECT_EQ(30U, track->member_features.at(2U).feature_id);
  EXPECT_EQ(40U, track->member_features.at(3U).feature_id);
}

TEST(LandmarkTrackManagerTest, EmptyFrameIsSafe) {
  LandmarkTrackManager manager;
  const auto result = process(&manager, 1U, {});
  EXPECT_TRUE(result.associations.empty());
  EXPECT_TRUE(manager.allTracks().empty());
}

TEST(LandmarkTrackManagerTest, NonFiniteObservationFailsSafely) {
  LandmarkTrackManager manager;
  auto input = makeInput(1U, 1.0, {{0U, 10U}, {1U, 20U}},
                         {makeDiagnostic(0U, 10U, 1U, 20U)});
  input.camera_tracking[0].tracks[0].current.pixel.x() =
      std::numeric_limits<double>::quiet_NaN();
  LandmarkTrackFrameResult result;
  EXPECT_FALSE(manager.processFrame(input, &result));
  EXPECT_TRUE(manager.allTracks().empty());
}

TEST(LandmarkTrackManagerTest, SameCameraCandidateIsRejected) {
  LandmarkTrackManager manager;
  const auto result = process(&manager, 1U, {{0U, 10U}, {0U, 11U}},
                              {makeDiagnostic(0U, 10U, 0U, 11U)});
  EXPECT_EQ(LandmarkAssociationStatus::kRejectedSameCamera,
            result.associations[0].status);
}

TEST(LandmarkTrackManagerTest, ActiveTemporalTracksSurviveNoConfirmations) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  for (std::uint64_t frame = 2U; frame <= 10U; ++frame) {
    process(&manager, frame,
            {{0U, 10U, static_cast<std::uint32_t>(frame)},
             {1U, 20U, static_cast<std::uint32_t>(frame)}});
  }
  EXPECT_EQ(LandmarkTrackState::kTentative, onlyTrack(manager).state);
  EXPECT_EQ(1U, onlyTrack(manager).cross_camera_confirmation_count);
  EXPECT_EQ(20U, onlyTrack(manager).total_observation_count);
}

TEST(LandmarkTrackManagerTest, ConfirmationStalenessIsDiagnosticOnly) {
  LandmarkTrackManagerOptions options;
  options.maximum_frames_without_confirmation = 1U;
  LandmarkTrackManager manager(options);
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  process(&manager, 2U, {{0U, 10U, 2U}, {1U, 20U, 2U}});
  process(&manager, 3U, {{0U, 10U, 3U}, {1U, 20U, 3U}});
  EXPECT_TRUE(onlyTrack(manager).confirmation_stale);
  EXPECT_EQ(LandmarkTrackState::kTentative, onlyTrack(manager).state);
}

TEST(LandmarkTrackManagerTest, InvalidOptionsAreRejected) {
  LandmarkTrackManagerOptions options;
  options.maximum_observation_history = 0U;
  LandmarkTrackManager manager(options);
  LandmarkTrackFrameResult result;
  EXPECT_FALSE(manager.processFrame(makeInput(1U, 1.0, {}), &result));
}

TEST(LandmarkTrackManagerTest, IdOverflowFailsWithoutCreatingTrack) {
  LandmarkTrackManager manager(
      LandmarkTrackManagerOptions{},
      LandmarkTrackId{std::numeric_limits<std::uint64_t>::max()});
  const auto result = process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
                              {makeDiagnostic(0U, 10U, 1U, 20U)});
  EXPECT_EQ(LandmarkAssociationStatus::kRejectedInvalidInput,
            result.associations[0].status);
  EXPECT_TRUE(manager.allTracks().empty());
}

TEST(LandmarkTrackManagerTest, NonMonotonicFramesFailWithoutMutation) {
  LandmarkTrackManager manager;
  process(&manager, 1U, {{0U, 10U}, {1U, 20U}},
          {makeDiagnostic(0U, 10U, 1U, 20U)});
  LandmarkTrackFrameResult result;
  EXPECT_FALSE(manager.processFrame(makeInput(1U, 2.0, {}), &result));
  EXPECT_EQ(1U, manager.allTracks().size());
}

}  // namespace
}  // namespace sphere_vio
