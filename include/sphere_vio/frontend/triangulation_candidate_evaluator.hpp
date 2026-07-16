#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/geometry/triangulation.hpp"

namespace sphere_vio {

enum class TriangulationCandidateStatus {
  kAccepted = 0,
  kInvalidMatch,
  kTriangulationFailed,
  kRayAngleTooSmall,
  kNegativeDepth,
  kDepthOutOfRange,
  kClosestDistanceTooLarge,
  kAngularReprojectionErrorTooLarge,
  kEpipolarErrorTooLarge,
  kNonFiniteResult,
  kCount
};

const char* triangulationCandidateStatusName(
    TriangulationCandidateStatus status);

struct TriangulationCandidateOptions {
  double minimum_ray_angle = 0.003;
  double minimum_depth = 0.1;
  double maximum_depth = 50.0;
  double maximum_closest_ray_distance = 0.02;
  double maximum_angular_reprojection_error = 0.003;
  double maximum_epipolar_error = 0.003;
};

struct TriangulationDiagnostic {
  CrossCameraMatch match;
  TriangulationCandidateStatus status =
      TriangulationCandidateStatus::kInvalidMatch;
  bool triangulation_succeeded = false;
  bool admitted = false;
  TriangulationResult triangulation;

  Eigen::Vector3d point_b = Eigen::Vector3d::Zero();
  double depth_1 = 0.0;
  double depth_2 = 0.0;
  double minimum_depth = 0.0;
  double maximum_depth = 0.0;
  double relative_depth_difference = 0.0;
  double closest_distance_to_baseline_ratio = 0.0;
  double ray_angle = 0.0;
  double closest_ray_distance = 0.0;
  double angular_reprojection_error_1 = 0.0;
  double angular_reprojection_error_2 = 0.0;
  double maximum_angular_reprojection_error = 0.0;
  double epipolar_error = 0.0;
};

// A currently admitted geometric observation. This is deliberately not a
// landmark, carries no persistent id, and is never written into FeatureTrack.
struct TriangulationCandidate {
  CrossCameraMatch match;
  TriangulationResult triangulation;
  Eigen::Vector3d point_b = Eigen::Vector3d::Zero();
  double depth_1 = 0.0;
  double depth_2 = 0.0;
  double minimum_depth = 0.0;
  double maximum_depth = 0.0;
  double relative_depth_difference = 0.0;
  double closest_distance_to_baseline_ratio = 0.0;
};

struct TriangulationCandidatePairResult {
  CameraId camera_id_1 = 0U;
  CameraId camera_id_2 = 0U;
  std::size_t input_matches = 0U;
  std::size_t triangulation_successes = 0U;
  std::vector<TriangulationDiagnostic> diagnostics;
  std::vector<TriangulationCandidate> candidates;
  double processing_time_seconds = 0.0;
};

struct DeterministicQuantiles {
  bool valid = false;
  std::size_t count = 0U;
  double minimum = 0.0;
  double p10 = 0.0;
  double median = 0.0;
  double p90 = 0.0;
  double p95 = 0.0;
  double maximum = 0.0;
};

// Uses sorted linear interpolation at p * (N - 1). Non-finite samples are
// rejected so statistics cannot silently hide invalid geometry.
bool computeDeterministicQuantiles(const std::vector<double>& samples,
                                   DeterministicQuantiles* quantiles);

// Re-applies only the deterministic admission gates to an existing geometry
// result. This is used by threshold scans to avoid re-triangulation.
bool applyTriangulationCandidateAdmission(
    const TriangulationCandidateOptions& options,
    TriangulationDiagnostic* diagnostic);

class TriangulationCandidateEvaluator {
 public:
  explicit TriangulationCandidateEvaluator(
      TriangulationCandidateOptions options = {});

  const TriangulationCandidateOptions& options() const { return options_; }

  bool evaluateMatch(const CrossCameraMatch& match,
                     const CameraDescriptorSet& descriptors_1,
                     const CameraDescriptorSet& descriptors_2,
                     const CameraRig& camera_rig,
                     TriangulationDiagnostic* diagnostic) const;

  bool evaluatePair(const CrossCameraPairResult& matches,
                    const CameraDescriptorSet& descriptors_1,
                    const CameraDescriptorSet& descriptors_2,
                    const CameraRig& camera_rig,
                    TriangulationCandidatePairResult* result) const;

 private:
  TriangulationCandidateOptions options_;
};

}  // namespace sphere_vio
