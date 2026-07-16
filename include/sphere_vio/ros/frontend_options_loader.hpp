#pragma once

#include <string>

#include "sphere_vio/frontend/cross_camera_matcher.hpp"
#include "sphere_vio/frontend/temporal_frontend.hpp"
#include "sphere_vio/frontend/triangulation_candidate_evaluator.hpp"

namespace sphere_vio {

bool loadTemporalFrontendOptions(const std::string& config_file,
                                 TemporalFrontendOptions* options,
                                 std::string* error);

bool loadCrossCameraOptions(const std::string& config_file,
                            OrbDescriptorExtractorOptions* descriptor_options,
                            CrossCameraMatcherOptions* matcher_options,
                            std::string* error);

bool loadTriangulationCandidateOptions(
    const std::string& config_file,
    TriangulationCandidateOptions* candidate_options,
    std::string* error);

}  // namespace sphere_vio
