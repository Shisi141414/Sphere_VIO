#pragma once

#include <string>

#include "sphere_vio/backend/msckf.hpp"
#include "sphere_vio/ros/offline_feature_runner.hpp"
#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/frontend/landmark_track_manager.hpp"
#include "sphere_vio/frontend/superpoint_extractor.hpp"
#include "sphere_vio/frontend/temporal_frontend.hpp"
#include "sphere_vio/frontend/triangulation_candidate_evaluator.hpp"

namespace sphere_vio {

bool loadSynchronizationOptions(const std::string& config_file,
                                OfflineFeatureRunnerOptions* options,
                                std::string* error);

bool loadTemporalFrontendOptions(const std::string& config_file,
                                 TemporalFrontendOptions* options,
                                 std::string* error);

bool loadSuperPointOptions(const std::string& config_file,
                           SuperPointExtractorOptions* options,
                           std::string* error);

bool loadCrossCameraOptions(const std::string& config_file,
                            OrbDescriptorExtractorOptions* descriptor_options,
                            CrossCameraMatcherOptions* matcher_options,
                            std::string* error);

bool loadTriangulationCandidateOptions(
    const std::string& config_file,
    TriangulationCandidateOptions* candidate_options,
    std::string* error);

bool loadLandmarkTrackManagerOptions(
    const std::string& config_file,
    LandmarkTrackManagerOptions* landmark_options,
    std::string* error);

bool loadMsckfOptions(const std::string& config_file,
                      MsckfOptions* msckf_options, std::string* error);

bool loadRuntimeGovernorOptions(const std::string& config_file,
                                RuntimeGovernorOptions* options,
                                std::string* error);

}  // namespace sphere_vio
