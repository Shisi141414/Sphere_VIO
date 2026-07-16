#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>

#include "sphere_vio/ros/offline_panorama_runner.hpp"

namespace {
std::string sibling(const std::string& path, const std::string& name) {
  const auto slash = path.find_last_of("/\\");
  return slash == std::string::npos ? name : path.substr(0, slash + 1) + name;
}
void usage() {
  std::cout << "Usage: sphere_vio_panorama_runner --config FILE --bag BAG "
               "[--system-config FILE] [--cameras FILE] [--start-offset S] "
               "[--duration S] [--parallel-cameras|--sequential-cameras] "
               "[--save-first-frame DIR]\n";
}
}  // namespace

int main(int argc, char** argv) {
  std::string config, bag, system, cameras, save, error;
  bool parallel_override = false, parallel = true;
  double start = 0, duration = -1; bool has_start = false, has_duration = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help") { usage(); return 0; }
    if (arg == "--parallel-cameras" || arg == "--sequential-cameras") {
      parallel_override = true; parallel = arg == "--parallel-cameras"; continue;
    }
    if (i + 1 >= argc) { usage(); return 1; }
    const std::string value(argv[++i]);
    if (arg == "--config") config = value;
    else if (arg == "--bag") bag = value;
    else if (arg == "--system-config" || arg == "--frontend-config") system = value;
    else if (arg == "--cameras") cameras = value;
    else if (arg == "--save-first-frame") save = value;
    else if (arg == "--start-offset") { start = std::stod(value); has_start = true; }
    else if (arg == "--duration") { duration = std::stod(value); has_duration = true; }
    else { usage(); return 1; }
  }
  if (config.empty() || bag.empty()) { usage(); return 1; }
  sphere_vio::OfflinePanoramaRunnerOptions options;
  if (!sphere_vio::loadOfflineBagConfiguration(config, bag, &options.bag, &error)) {
    std::cerr << error << std::endl; return 1;
  }
  if (has_start) options.bag.start_time_offset = start;
  if (has_duration) options.bag.duration = duration;
  options.system_config_file = system.empty() ? sibling(config, "system.yaml") : system;
  options.camera_config_file = cameras.empty() ? sibling(config, "cameras.yaml") : cameras;
  options.save_first_frame_directory = save;
  if (!sphere_vio::loadPanoramaConfiguration(options.system_config_file,
          &options.panorama, &options.remap, &error)) {
    std::cerr << error << std::endl; return 1;
  }
  if (parallel_override) options.remap.parallel_cameras = parallel;
  return sphere_vio::OfflinePanoramaRunner(std::move(options)).run();
}
