#include "sphere_vio/frontend/cross_camera_matcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

#include <opencv2/features2d.hpp>

#include "sphere_vio/geometry/epipolar_geometry.hpp"

namespace sphere_vio {
namespace {

constexpr int kOrbDescriptorBytes = 32;

std::pair<CameraId, CameraId> canonicalPair(CameraId first,
                                            CameraId second) {
  return first < second ? std::make_pair(first, second)
                        : std::make_pair(second, first);
}

bool validOptions(const CrossCameraMatcherOptions& options) {
  if (!std::isfinite(options.maximum_descriptor_distance) ||
      options.maximum_descriptor_distance < 0.0 ||
      !std::isfinite(options.ratio_test) || options.ratio_test <= 0.0 ||
      options.ratio_test >= 1.0 ||
      !std::isfinite(options.maximum_epipolar_angle) ||
      options.maximum_epipolar_angle < 0.0 ||
      options.maximum_matches_per_pair == 0U) {
    return false;
  }
  std::set<std::pair<CameraId, CameraId>> unique_pairs;
  for (const auto& pair : options.camera_pairs) {
    if (pair.first >= 4U || pair.second >= 4U || pair.first >= pair.second ||
        !unique_pairs.insert(pair).second) {
      return false;
    }
  }
  return !options.camera_pairs.empty();
}

bool validDescriptorSet(const CameraDescriptorSet& set) {
  const std::size_t size = set.feature_ids.size();
  if (set.camera_id >= 4U || !std::isfinite(set.timestamp) ||
      set.pixels.size() != size || set.bearings_c.size() != size ||
      set.bearings_b.size() != size) {
    return false;
  }
  if (size == 0U) return set.descriptors.empty();
  if (set.descriptors.type() != CV_8UC1 ||
      set.descriptors.cols != kOrbDescriptorBytes ||
      set.descriptors.rows != static_cast<int>(size)) {
    return false;
  }
  for (std::size_t index = 0U; index < size; ++index) {
    if (set.feature_ids[index] == 0U || !set.pixels[index].allFinite() ||
        !set.bearings_c[index].allFinite() ||
        !set.bearings_b[index].allFinite()) {
      return false;
    }
  }
  return true;
}

struct DirectionalCandidate {
  bool has_best = false;
  bool absolute_pass = false;
  bool ratio_pass = false;
  int best_index = -1;
  double best_distance = 0.0;
  double ratio = std::numeric_limits<double>::infinity();
};

std::vector<DirectionalCandidate> evaluateDirectionalCandidates(
    const cv::Mat& queries, const cv::Mat& trains,
    const CrossCameraMatcherOptions& options) {
  std::vector<DirectionalCandidate> candidates(
      static_cast<std::size_t>(queries.rows));
  if (queries.empty() || trains.empty()) return candidates;
  std::vector<std::vector<cv::DMatch>> knn_matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING, false);
  matcher.knnMatch(queries, trains, knn_matches, 2);
  for (std::size_t query_index = 0U;
       query_index < candidates.size() && query_index < knn_matches.size();
       ++query_index) {
    if (knn_matches[query_index].empty()) continue;
    std::stable_sort(knn_matches[query_index].begin(),
                     knn_matches[query_index].end(),
                     [](const cv::DMatch& lhs, const cv::DMatch& rhs) {
                       if (lhs.distance != rhs.distance)
                         return lhs.distance < rhs.distance;
                       return lhs.trainIdx < rhs.trainIdx;
                     });
    DirectionalCandidate& candidate = candidates[query_index];
    const cv::DMatch& best = knn_matches[query_index][0];
    candidate.has_best = true;
    candidate.best_index = best.trainIdx;
    candidate.best_distance = best.distance;
    candidate.absolute_pass =
        std::isfinite(candidate.best_distance) &&
        candidate.best_distance <= options.maximum_descriptor_distance;
    if (knn_matches[query_index].size() < 2U) continue;
    const double second_distance = knn_matches[query_index][1].distance;
    if (!std::isfinite(second_distance) || second_distance <= 0.0) continue;
    candidate.ratio = candidate.best_distance / second_distance;
    candidate.ratio_pass = candidate.absolute_pass &&
                           candidate.best_distance <
                               options.ratio_test * second_distance;
  }
  return candidates;
}

bool finalMatchOrder(const CrossCameraMatch& lhs,
                     const CrossCameraMatch& rhs) {
  if (lhs.camera_id_1 != rhs.camera_id_1)
    return lhs.camera_id_1 < rhs.camera_id_1;
  if (lhs.camera_id_2 != rhs.camera_id_2)
    return lhs.camera_id_2 < rhs.camera_id_2;
  if (lhs.feature_id_1 != rhs.feature_id_1)
    return lhs.feature_id_1 < rhs.feature_id_1;
  return lhs.feature_id_2 < rhs.feature_id_2;
}

}  // namespace

CrossCameraMatcher::CrossCameraMatcher(CrossCameraMatcherOptions options)
    : options_(std::move(options)) {}

bool CrossCameraMatcher::isConfiguredPair(CameraId camera_id_1,
                                          CameraId camera_id_2) const {
  if (camera_id_1 == camera_id_2) return false;
  const auto pair = canonicalPair(camera_id_1, camera_id_2);
  return std::find(options_.camera_pairs.begin(), options_.camera_pairs.end(),
                   pair) != options_.camera_pairs.end();
}

bool CrossCameraMatcher::matchPair(
    const CameraDescriptorSet& first_set,
    const CameraDescriptorSet& second_set, const CameraRig& camera_rig,
    CrossCameraPairResult* result) const {
  if (!result) return false;
  *result = CrossCameraPairResult{};
  const auto start = std::chrono::steady_clock::now();
  if (!validOptions(options_) || !validDescriptorSet(first_set) ||
      !validDescriptorSet(second_set) ||
      first_set.camera_id == second_set.camera_id ||
      first_set.timestamp != second_set.timestamp ||
      !isConfiguredPair(first_set.camera_id, second_set.camera_id)) {
    return false;
  }

  const CameraDescriptorSet* descriptors_1 = &first_set;
  const CameraDescriptorSet* descriptors_2 = &second_set;
  if (descriptors_1->camera_id > descriptors_2->camera_id)
    std::swap(descriptors_1, descriptors_2);
  result->camera_id_1 = descriptors_1->camera_id;
  result->camera_id_2 = descriptors_2->camera_id;
  result->descriptors_1 = descriptors_1->feature_ids.size();
  result->descriptors_2 = descriptors_2->feature_ids.size();
  if (descriptors_1->feature_ids.empty() ||
      descriptors_2->feature_ids.empty()) {
    result->processing_time_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    return true;
  }

  const std::vector<DirectionalCandidate> forward =
      evaluateDirectionalCandidates(descriptors_1->descriptors,
                                    descriptors_2->descriptors, options_);
  const std::vector<DirectionalCandidate> backward =
      evaluateDirectionalCandidates(descriptors_2->descriptors,
                                    descriptors_1->descriptors, options_);
  RelativePose relative_pose;
  const RigCamera* camera_1 = camera_rig.camera(result->camera_id_1);
  const RigCamera* camera_2 = camera_rig.camera(result->camera_id_2);
  const bool valid_geometry = camera_1 && camera_2 &&
                              relativeCameraPose(*camera_1, *camera_2,
                                                 &relative_pose);

  std::vector<CrossCameraMatch> epipolar_candidates;
  for (std::size_t index_1 = 0U; index_1 < forward.size(); ++index_1) {
    const DirectionalCandidate& candidate = forward[index_1];
    if (!candidate.has_best) continue;
    ++result->raw_candidates;
    if (!candidate.absolute_pass) {
      ++result->rejected_absolute_distance;
      continue;
    }
    ++result->absolute_distance_accepted;
    if (!candidate.ratio_pass) {
      ++result->rejected_ratio;
      continue;
    }
    ++result->ratio_accepted;
    const std::size_t index_2 =
        static_cast<std::size_t>(candidate.best_index);
    const bool mutual =
        index_2 < backward.size() && backward[index_2].has_best &&
        backward[index_2].absolute_pass && backward[index_2].ratio_pass &&
        backward[index_2].best_index == static_cast<int>(index_1);
    if (options_.require_mutual_best && !mutual) {
      ++result->rejected_non_mutual;
      continue;
    }
    ++result->mutual_accepted;
    if (!valid_geometry) {
      ++result->rejected_degenerate_geometry;
      continue;
    }

    EpipolarError epipolar_error;
    if (!symmetricEpipolarAngularError(
            descriptors_1->bearings_c[index_1],
            descriptors_2->bearings_c[index_2],
            relative_pose.R_target_source, relative_pose.t_target_source,
            &epipolar_error)) {
      ++result->rejected_degenerate_geometry;
      continue;
    }
    if (epipolar_error.maximum > options_.maximum_epipolar_angle) {
      ++result->rejected_epipolar;
      continue;
    }

    CrossCameraMatch match;
    match.camera_id_1 = result->camera_id_1;
    match.camera_id_2 = result->camera_id_2;
    match.feature_id_1 = descriptors_1->feature_ids[index_1];
    match.feature_id_2 = descriptors_2->feature_ids[index_2];
    match.pixel_1 = descriptors_1->pixels[index_1];
    match.pixel_2 = descriptors_2->pixels[index_2];
    match.descriptor_distance = candidate.best_distance;
    match.ratio = candidate.ratio;
    match.epipolar_error_forward = epipolar_error.forward;
    match.epipolar_error_backward = epipolar_error.backward;
    match.epipolar_error_maximum = epipolar_error.maximum;
    epipolar_candidates.push_back(std::move(match));
    ++result->epipolar_accepted;
  }

  std::stable_sort(
      epipolar_candidates.begin(), epipolar_candidates.end(),
      [](const CrossCameraMatch& lhs, const CrossCameraMatch& rhs) {
        if (lhs.descriptor_distance != rhs.descriptor_distance)
          return lhs.descriptor_distance < rhs.descriptor_distance;
        if (lhs.epipolar_error_maximum != rhs.epipolar_error_maximum)
          return lhs.epipolar_error_maximum < rhs.epipolar_error_maximum;
        if (lhs.feature_id_1 != rhs.feature_id_1)
          return lhs.feature_id_1 < rhs.feature_id_1;
        return lhs.feature_id_2 < rhs.feature_id_2;
      });
  std::set<FeatureId> used_1;
  std::set<FeatureId> used_2;
  for (const CrossCameraMatch& candidate : epipolar_candidates) {
    if (!used_1.insert(candidate.feature_id_1).second ||
        !used_2.insert(candidate.feature_id_2).second) {
      ++result->rejected_duplicate;
      continue;
    }
    result->matches.push_back(candidate);
  }
  std::stable_sort(result->matches.begin(), result->matches.end(),
                   finalMatchOrder);
  if (result->matches.size() > options_.maximum_matches_per_pair) {
    result->matches.resize(options_.maximum_matches_per_pair);
  }
  result->processing_time_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  return true;
}

bool CrossCameraMatcher::matchConfiguredPairs(
    const std::vector<CameraDescriptorSet>& descriptor_sets,
    const CameraRig& camera_rig,
    std::vector<CrossCameraPairResult>* results) const {
  if (!results || !validOptions(options_) || descriptor_sets.size() != 4U) {
    return false;
  }
  results->clear();
  std::vector<const CameraDescriptorSet*> by_camera(4U, nullptr);
  for (const CameraDescriptorSet& set : descriptor_sets) {
    if (set.camera_id >= 4U || by_camera[set.camera_id] != nullptr) return false;
    by_camera[set.camera_id] = &set;
  }
  for (const auto& pair : options_.camera_pairs) {
    if (!by_camera[pair.first] || !by_camera[pair.second]) return false;
    CrossCameraPairResult pair_result;
    if (!matchPair(*by_camera[pair.first], *by_camera[pair.second], camera_rig,
                   &pair_result)) {
      return false;
    }
    results->push_back(std::move(pair_result));
  }
  std::stable_sort(results->begin(), results->end(),
                   [](const CrossCameraPairResult& lhs,
                      const CrossCameraPairResult& rhs) {
                     if (lhs.camera_id_1 != rhs.camera_id_1)
                       return lhs.camera_id_1 < rhs.camera_id_1;
                     return lhs.camera_id_2 < rhs.camera_id_2;
                   });
  return true;
}

}  // namespace sphere_vio
