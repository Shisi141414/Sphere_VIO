#pragma once

#include <string>

#include "sphere_vio/panorama/panorama_remapper.hpp"
#include "sphere_vio/ros/offline_bag_runner.hpp"

namespace sphere_vio {

struct OfflinePanoramaRunnerOptions {
  OfflineBagConfiguration bag;
  std::string camera_config_file;
  std::string system_config_file;
  std::string save_first_frame_directory;
  PanoramaSpec panorama;
  PanoramaRemapOptions remap;
};

bool loadPanoramaConfiguration(const std::string& system_config_file,
                               PanoramaSpec* panorama,
                               PanoramaRemapOptions* remap,
                               std::string* error = nullptr);

class OfflinePanoramaRunner {
 public:
  explicit OfflinePanoramaRunner(OfflinePanoramaRunnerOptions options);
  int run();

 private:
  OfflinePanoramaRunnerOptions options_;
};

}  // namespace sphere_vio
