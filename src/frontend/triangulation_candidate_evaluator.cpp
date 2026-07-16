#include "sphere_vio/frontend/triangulation_candidate_evaluator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace sphere_vio {
namespace {

constexpr double kCoreMinimumRayAngle = 1e-12;

bool validOptions(const TriangulationCandidateOptions& options) {
  return std::isfinite(options.minimum_ray_angle) &&
         options.minimum_ray_angle >= 0.0 &&
         std::isfinite(options.minimum_depth) &&
         options.minimum_depth >= 0.0 &&
         std::isfinite(options.maximum_depth) &&
         options.maximum_depth >= options.minimum_depth &&
         std::isfinite(options.maximum_closest_ray_distance) &&
         options.maximum_closest_ray_distance >= 0.0 &&
         std::isfinite(options.maximum_angular_reprojection_error) &&
         options.maximum_angular_reprojection_error >= 0.0 &&
         std::isfinite(options.maximum_epipolar_error) &&
         options.maximum_epipolar_error >= 0.0;
}

bool findUniqueFeature(const CameraDescriptorSet& descriptors,
                       FeatureId feature_id, Eigen::Vector3d* bearing_c) {
  if (!bearing_c || feature_id == 0U ||
      descriptors.feature_ids.size() != descriptors.bearings_c.size()) {
    return false;
  }
  bool found = false;
  for (std::size_t index = 0U; index < descriptors.feature_ids.size();
       ++index) {
    if (descriptors.feature_ids[index] != feature_id) continue;
    if (found || !descriptors.bearings_c[index].allFinite() ||
        descriptors.bearings_c[index].norm() <= 1e-12) {
      return false;
    }
    *bearing_c = descriptors.bearings_c[index];
    found = true;
  }
  return found;
}

bool finiteMatch(const CrossCameraMatch& match) {
  return match.camera_id_1 < match.camera_id_2 && match.camera_id_2 < 4U &&
         match.feature_id_1 != 0U && match.feature_id_2 != 0U &&
         match.pixel_1.allFinite() && match.pixel_2.allFinite() &&
         std::isfinite(match.descriptor_distance) &&
         std::isfinite(match.ratio) &&
         std::isfinite(match.epipolar_error_forward) &&
         std::isfinite(match.epipolar_error_backward) &&
         std::isfinite(match.epipolar_error_maximum);
}

bool finiteGeometry(const TriangulationDiagnostic& diagnostic) {
  return diagnostic.point_b.allFinite() &&
         std::isfinite(diagnostic.depth_1) &&
         std::isfinite(diagnostic.depth_2) &&
         std::isfinite(diagnostic.minimum_depth) &&
         std::isfinite(diagnostic.maximum_depth) &&
         std::isfinite(diagnostic.relative_depth_difference) &&
         std::isfinite(diagnostic.closest_distance_to_baseline_ratio) &&
         std::isfinite(diagnostic.ray_angle) &&
         std::isfinite(diagnostic.closest_ray_distance) &&
         std::isfinite(diagnostic.angular_reprojection_error_1) &&
         std::isfinite(diagnostic.angular_reprojection_error_2) &&
         std::isfinite(diagnostic.maximum_angular_reprojection_error) &&
         std::isfinite(diagnostic.epipolar_error);
}

TriangulationCandidate makeCandidate(
    const TriangulationDiagnostic& diagnostic) {
  TriangulationCandidate candidate;
  candidate.match = diagnostic.match;
  candidate.triangulation = diagnostic.triangulation;
  candidate.point_b = diagnostic.point_b;
  candidate.depth_1 = diagnostic.depth_1;
  candidate.depth_2 = diagnostic.depth_2;
  candidate.minimum_depth = diagnostic.minimum_depth;
  candidate.maximum_depth = diagnostic.maximum_depth;
  candidate.relative_depth_difference =
      diagnostic.relative_depth_difference;
  candidate.closest_distance_to_baseline_ratio =
      diagnostic.closest_distance_to_baseline_ratio;
  return candidate;
}

double interpolatedQuantile(const std::vector<double>& sorted, double p) {
  if (sorted.size() == 1U) return sorted.front();
  const double position = p * static_cast<double>(sorted.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
}

}  // namespace

const char* triangulationCandidateStatusName(
    TriangulationCandidateStatus status) {
  switch (status) {
    case TriangulationCandidateStatus::kAccepted:
      return "accepted";
    case TriangulationCandidateStatus::kInvalidMatch:
      return "invalid_match";
    case TriangulationCandidateStatus::kTriangulationFailed:
      return "triangulation_failed";
    case TriangulationCandidateStatus::kRayAngleTooSmall:
      return "ray_angle_too_small";
    case TriangulationCandidateStatus::kNegativeDepth:
      return "negative_depth";
    case TriangulationCandidateStatus::kDepthOutOfRange:
      return "depth_out_of_range";
    case TriangulationCandidateStatus::kClosestDistanceTooLarge:
      return "closest_distance_too_large";
    case TriangulationCandidateStatus::kAngularReprojectionErrorTooLarge:
      return "angular_reprojection_error_too_large";
    case TriangulationCandidateStatus::kEpipolarErrorTooLarge:
      return "epipolar_error_too_large";
    case TriangulationCandidateStatus::kNonFiniteResult:
      return "non_finite_result";
    case TriangulationCandidateStatus::kCount:
      return "count";
  }
  return "unknown";
}

bool computeDeterministicQuantiles(const std::vector<double>& samples,
                                   DeterministicQuantiles* quantiles) {
  if (!quantiles) return false;
  *quantiles = DeterministicQuantiles{};
  for (const double sample : samples) {
    if (!std::isfinite(sample)) return false;
  }
  quantiles->count = samples.size();
  if (samples.empty()) return true;
  std::vector<double> sorted = samples;
  std::stable_sort(sorted.begin(), sorted.end());
  quantiles->valid = true;
  quantiles->minimum = sorted.front();
  quantiles->p10 = interpolatedQuantile(sorted, 0.10);
  quantiles->median = interpolatedQuantile(sorted, 0.50);
  quantiles->p90 = interpolatedQuantile(sorted, 0.90);
  quantiles->p95 = interpolatedQuantile(sorted, 0.95);
  quantiles->maximum = sorted.back();
  return true;
}

bool applyTriangulationCandidateAdmission(
    const TriangulationCandidateOptions& options,
    TriangulationDiagnostic* diagnostic) {
  if (!diagnostic || !validOptions(options)) return false;
  diagnostic->admitted = false;
  if (diagnostic->status == TriangulationCandidateStatus::kInvalidMatch ||
      diagnostic->status ==
          TriangulationCandidateStatus::kTriangulationFailed ||
      diagnostic->status == TriangulationCandidateStatus::kNegativeDepth ||
      diagnostic->status ==
          TriangulationCandidateStatus::kNonFiniteResult) {
    return true;
  }
  if (!diagnostic->triangulation_succeeded) {
    diagnostic->status =
        TriangulationCandidateStatus::kTriangulationFailed;
    return true;
  }
  if (!finiteGeometry(*diagnostic)) {
    diagnostic->status = TriangulationCandidateStatus::kNonFiniteResult;
  } else if (diagnostic->depth_1 <= 0.0 || diagnostic->depth_2 <= 0.0) {
    diagnostic->status = TriangulationCandidateStatus::kNegativeDepth;
  } else if (diagnostic->ray_angle < options.minimum_ray_angle) {
    diagnostic->status = TriangulationCandidateStatus::kRayAngleTooSmall;
  } else if (diagnostic->minimum_depth < options.minimum_depth ||
             diagnostic->maximum_depth > options.maximum_depth) {
    diagnostic->status = TriangulationCandidateStatus::kDepthOutOfRange;
  } else if (diagnostic->closest_ray_distance >
             options.maximum_closest_ray_distance) {
    diagnostic->status =
        TriangulationCandidateStatus::kClosestDistanceTooLarge;
  } else if (diagnostic->maximum_angular_reprojection_error >
             options.maximum_angular_reprojection_error) {
    diagnostic->status = TriangulationCandidateStatus::
        kAngularReprojectionErrorTooLarge;
  } else if (diagnostic->epipolar_error > options.maximum_epipolar_error) {
    diagnostic->status =
        TriangulationCandidateStatus::kEpipolarErrorTooLarge;
  } else {
    diagnostic->status = TriangulationCandidateStatus::kAccepted;
    diagnostic->admitted = true;
  }
  return true;
}

TriangulationCandidateEvaluator::TriangulationCandidateEvaluator(
    TriangulationCandidateOptions options)
    : options_(options) {}

bool TriangulationCandidateEvaluator::evaluateMatch(
    const CrossCameraMatch& match,
    const CameraDescriptorSet& first_descriptors,
    const CameraDescriptorSet& second_descriptors,
    const CameraRig& camera_rig,
    TriangulationDiagnostic* diagnostic) const {
  if (!diagnostic || !validOptions(options_)) return false;
  *diagnostic = TriangulationDiagnostic{};
  diagnostic->match = match;
  diagnostic->epipolar_error = match.epipolar_error_maximum;

  const CameraDescriptorSet* descriptors_1 = &first_descriptors;
  const CameraDescriptorSet* descriptors_2 = &second_descriptors;
  if (descriptors_1->camera_id > descriptors_2->camera_id)
    std::swap(descriptors_1, descriptors_2);
  if (!finiteMatch(match) ||
      descriptors_1->camera_id != match.camera_id_1 ||
      descriptors_2->camera_id != match.camera_id_2 ||
      descriptors_1->timestamp != descriptors_2->timestamp) {
    return true;
  }
  Eigen::Vector3d bearing_c1;
  Eigen::Vector3d bearing_c2;
  if (!findUniqueFeature(*descriptors_1, match.feature_id_1, &bearing_c1) ||
      !findUniqueFeature(*descriptors_2, match.feature_id_2, &bearing_c2)) {
    return true;
  }
  const RigCamera* camera_1 = camera_rig.camera(match.camera_id_1);
  const RigCamera* camera_2 = camera_rig.camera(match.camera_id_2);
  if (!camera_1 || !camera_2) return true;

  TriangulationOptions core_options;
  core_options.minimum_ray_angle = kCoreMinimumRayAngle;
  core_options.minimum_depth = 0.0;
  core_options.maximum_closest_ray_distance =
      std::numeric_limits<double>::infinity();
  core_options.maximum_angular_reprojection_error =
      std::numeric_limits<double>::infinity();
  if (!triangulateBodyBearings(*camera_1, bearing_c1, *camera_2, bearing_c2,
                               core_options, &diagnostic->triangulation)) {
    if (diagnostic->triangulation.status ==
        TriangulationStatus::kNegativeDepth) {
      diagnostic->status = TriangulationCandidateStatus::kNegativeDepth;
    } else if (diagnostic->triangulation.status ==
               TriangulationStatus::kNumericalFailure) {
      diagnostic->status =
          TriangulationCandidateStatus::kNonFiniteResult;
    } else {
      diagnostic->status =
          TriangulationCandidateStatus::kTriangulationFailed;
    }
    diagnostic->depth_1 = diagnostic->triangulation.depth_1;
    diagnostic->depth_2 = diagnostic->triangulation.depth_2;
    diagnostic->ray_angle = diagnostic->triangulation.ray_angle;
    return true;
  }

  diagnostic->triangulation_succeeded = true;
  diagnostic->status = TriangulationCandidateStatus::kAccepted;
  diagnostic->point_b = diagnostic->triangulation.point_common;
  diagnostic->depth_1 = diagnostic->triangulation.depth_1;
  diagnostic->depth_2 = diagnostic->triangulation.depth_2;
  diagnostic->minimum_depth =
      std::min(diagnostic->depth_1, diagnostic->depth_2);
  diagnostic->maximum_depth =
      std::max(diagnostic->depth_1, diagnostic->depth_2);
  diagnostic->relative_depth_difference =
      std::abs(diagnostic->depth_1 - diagnostic->depth_2) /
      diagnostic->maximum_depth;
  diagnostic->closest_distance_to_baseline_ratio =
      diagnostic->triangulation.closest_ray_distance /
      diagnostic->triangulation.baseline;
  diagnostic->ray_angle = diagnostic->triangulation.ray_angle;
  diagnostic->closest_ray_distance =
      diagnostic->triangulation.closest_ray_distance;
  diagnostic->angular_reprojection_error_1 =
      diagnostic->triangulation.angular_reprojection_error_1;
  diagnostic->angular_reprojection_error_2 =
      diagnostic->triangulation.angular_reprojection_error_2;
  diagnostic->maximum_angular_reprojection_error =
      diagnostic->triangulation.maximum_angular_reprojection_error;
  return applyTriangulationCandidateAdmission(options_, diagnostic);
}

bool TriangulationCandidateEvaluator::evaluatePair(
    const CrossCameraPairResult& matches,
    const CameraDescriptorSet& descriptors_1,
    const CameraDescriptorSet& descriptors_2,
    const CameraRig& camera_rig,
    TriangulationCandidatePairResult* result) const {
  if (!result || !validOptions(options_)) return false;
  *result = TriangulationCandidatePairResult{};
  const auto start = std::chrono::steady_clock::now();
  result->camera_id_1 = matches.camera_id_1;
  result->camera_id_2 = matches.camera_id_2;
  result->input_matches = matches.matches.size();
  result->diagnostics.reserve(matches.matches.size());
  result->candidates.reserve(matches.matches.size());
  for (const CrossCameraMatch& match : matches.matches) {
    TriangulationDiagnostic diagnostic;
    if (!evaluateMatch(match, descriptors_1, descriptors_2, camera_rig,
                       &diagnostic)) {
      return false;
    }
    if (diagnostic.triangulation_succeeded)
      ++result->triangulation_successes;
    if (diagnostic.admitted)
      result->candidates.push_back(makeCandidate(diagnostic));
    result->diagnostics.push_back(std::move(diagnostic));
  }
  result->processing_time_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  return true;
}

}  // namespace sphere_vio
