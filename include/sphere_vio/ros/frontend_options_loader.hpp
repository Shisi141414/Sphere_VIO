#pragma once

#include <string>

#include "sphere_vio/frontend/temporal_frontend.hpp"

namespace sphere_vio {

bool loadTemporalFrontendOptions(const std::string& config_file,
                                 TemporalFrontendOptions* options,
                                 std::string* error);

}  // namespace sphere_vio
