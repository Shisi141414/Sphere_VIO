#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "sphere_vio/ros/frontend_options_loader.hpp"
#include "sphere_vio/ros/offline_feature_runner.hpp"

namespace {

struct CommandLineOptions {
  std::string config_file;
  std::string frontend_config_file;
  std::string camera_config_file;
  std::string bag_path;
  double start_offset = 0.0;
  double duration = -1.0;
  bool has_start_offset = false;
  bool has_duration = false;
  bool cross_camera_matching = false;
  bool triangulation_candidates = false;
  bool triangulation_threshold_sweep = false;
  bool landmark_tracks = false;
};

void printUsage() {
  std::cout
      << "Usage: sphere_vio_feature_runner --config FILE --bag BAG [options]\n"
         "Options:\n"
         "  --frontend-config FILE  Frontend YAML (default: sibling system.yaml)\n"
         "  --cameras FILE         Camera YAML (default: sibling cameras.yaml)\n"
         "  --start-offset SEC     Offset from bag start\n"
         "  --duration SEC         Negative means through bag end\n"
         "  --cross-camera-matching  Enable configured overlap-pair matching\n"
         "  --triangulation-candidates  Evaluate current-frame geometric candidates\n"
         "  --triangulation-threshold-sweep  Add single-variable gate scans\n"
         "  --landmark-tracks  Enable observation-association hypotheses\n"
         "  --help                 Show this message"
      << std::endl;
}

bool parseDouble(const std::string& text, double* value) {
  if (!value) return false;
  try {
    std::size_t consumed = 0U;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed)) return false;
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool parseCommandLine(int argc, char** argv, CommandLineOptions* options,
                      bool* help_requested, std::string* error) {
  if (!options || !help_requested) return false;
  *help_requested = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument == "--help" || argument == "-h") {
      *help_requested = true;
      return true;
    }
    if (argument == "--cross-camera-matching") {
      options->cross_camera_matching = true;
      continue;
    }
    if (argument == "--triangulation-candidates") {
      options->triangulation_candidates = true;
      options->cross_camera_matching = true;
      continue;
    }
    if (argument == "--triangulation-threshold-sweep") {
      options->triangulation_threshold_sweep = true;
      options->triangulation_candidates = true;
      options->cross_camera_matching = true;
      continue;
    }
    if (argument == "--landmark-tracks") {
      options->landmark_tracks = true;
      options->triangulation_candidates = true;
      options->cross_camera_matching = true;
      continue;
    }
    if (i + 1 >= argc) {
      if (error) *error = "missing value after " + argument;
      return false;
    }
    const std::string value(argv[++i]);
    if (argument == "--config") {
      options->config_file = value;
    } else if (argument == "--frontend-config") {
      options->frontend_config_file = value;
    } else if (argument == "--cameras") {
      options->camera_config_file = value;
    } else if (argument == "--bag") {
      options->bag_path = value;
    } else if (argument == "--start-offset") {
      if (!parseDouble(value, &options->start_offset)) return false;
      options->has_start_offset = true;
    } else if (argument == "--duration") {
      if (!parseDouble(value, &options->duration)) return false;
      options->has_duration = true;
    } else {
      if (error) *error = "unknown argument: " + argument;
      return false;
    }
  }
  if (options->config_file.empty() || options->bag_path.empty()) {
    if (error) *error = "--config and --bag are required";
    return false;
  }
  if (options->has_start_offset && options->start_offset < 0.0) {
    if (error) *error = "--start-offset cannot be negative";
    return false;
  }
  return true;
}

std::string siblingPath(const std::string& path, const std::string& name) {
  const std::size_t separator = path.find_last_of("/\\");
  if (separator == std::string::npos) return name;
  return path.substr(0, separator + 1U) + name;
}

}  // namespace

int main(int argc, char** argv) {
  CommandLineOptions command_line;
  bool help_requested = false;
  std::string error;
  if (!parseCommandLine(argc, argv, &command_line, &help_requested, &error)) {
    std::cerr << "Invalid command line: " << error << std::endl;
    printUsage();
    return EXIT_FAILURE;
  }
  if (help_requested) {
    printUsage();
    return EXIT_SUCCESS;
  }

  sphere_vio::OfflineFeatureRunnerOptions options;
  if (!sphere_vio::loadOfflineBagConfiguration(
          command_line.config_file, command_line.bag_path, &options.bag,
          &error)) {
    std::cerr << "Invalid offline configuration: " << error << std::endl;
    return EXIT_FAILURE;
  }
  if (command_line.has_start_offset)
    options.bag.start_time_offset = command_line.start_offset;
  if (command_line.has_duration) options.bag.duration = command_line.duration;

  const std::string frontend_config =
      command_line.frontend_config_file.empty()
          ? siblingPath(command_line.config_file, "system.yaml")
          : command_line.frontend_config_file;
  if (!sphere_vio::loadTemporalFrontendOptions(
          frontend_config, &options.frontend, &error)) {
    std::cerr << "Invalid frontend configuration: " << error << std::endl;
    return EXIT_FAILURE;
  }
  options.cross_camera_matching = command_line.cross_camera_matching;
  options.triangulation_candidates = command_line.triangulation_candidates;
  options.triangulation_threshold_sweep =
      command_line.triangulation_threshold_sweep;
  options.landmark_tracks = command_line.landmark_tracks;
  if (options.cross_camera_matching &&
      !sphere_vio::loadCrossCameraOptions(
          frontend_config, &options.descriptor, &options.matcher, &error)) {
    std::cerr << "Invalid cross-camera configuration: " << error << std::endl;
    return EXIT_FAILURE;
  }
  if (options.triangulation_candidates &&
      !sphere_vio::loadTriangulationCandidateOptions(
          frontend_config, &options.triangulation_candidate, &error)) {
    std::cerr << "Invalid triangulation candidate configuration: " << error
              << std::endl;
    return EXIT_FAILURE;
  }
  if (options.landmark_tracks &&
      !sphere_vio::loadLandmarkTrackManagerOptions(
          frontend_config, &options.landmark_track, &error)) {
    std::cerr << "Invalid landmark track configuration: " << error
              << std::endl;
    return EXIT_FAILURE;
  }
  options.camera_config_file =
      command_line.camera_config_file.empty()
          ? siblingPath(command_line.config_file, "cameras.yaml")
          : command_line.camera_config_file;
  return sphere_vio::OfflineFeatureRunner(std::move(options)).run();
}
