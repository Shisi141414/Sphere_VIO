#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "sphere_vio/ros/offline_bag_visualizer.hpp"
#include "sphere_vio/ros/frontend_options_loader.hpp"

namespace {

struct CommandLineOptions {
  std::string config_file;
  std::string bag_path;
  double playback_rate = 1.0;
  double imu_gap_warning = 0.008;
  double start_offset = 0.0;
  double duration = -1.0;
  bool has_start_offset = false;
  bool has_duration = false;
  bool show_spherical_coverage = false;
  bool show_epipolar_curve = false;
  bool show_temporal_features = false;
  std::string frontend_config_file;
  std::string camera_config_file;
  int epipolar_source_camera = 0;
  int epipolar_target_camera = 1;
  double epipolar_source_u = -1.0;
  double epipolar_source_v = -1.0;
};

void printUsage() {
  std::cout
      << "Usage: sphere_vio_bag_visualizer --config FILE --bag BAG [options]\n"
         "Options:\n"
         "  --rate VALUE             Playback rate (default: 1.0)\n"
         "  --start-offset SECONDS   Offset from bag start\n"
         "  --duration SECONDS       Negative means through bag end\n"
         "  --imu-gap-warning SEC    Display warning threshold (default: 0.008)\n"
         "  --show-spherical-coverage  Add sparse Body-bearing ERP panel\n"
         "  --cameras FILE           Camera YAML for geometry overlays\n"
         "  --show-epipolar-curve   Draw a calibrated spherical epipolar curve\n"
         "  --show-temporal-features  Draw same-camera LK feature tracks\n"
         "  --frontend-config FILE Frontend YAML (default: sibling system.yaml)\n"
         "  --epipolar-source-camera ID  Source camera C0..C3 (default: 0)\n"
         "  --epipolar-target-camera ID  Target camera C0..C3 (default: 1)\n"
         "  --epipolar-source-u VALUE    Source pixel u (default: image center)\n"
         "  --epipolar-source-v VALUE    Source pixel v (default: image center)\n"
         "  --help                   Show this message"
      << std::endl;
}

bool parseDouble(const std::string& text, double* value) {
  if (!value) return false;
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed)) return false;
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool parseCameraId(const std::string& text, int* camera_id) {
  if (!camera_id) return false;
  try {
    std::size_t consumed = 0;
    const int parsed = std::stoi(text, &consumed);
    if (consumed != text.size() || parsed < 0 || parsed > 3) return false;
    *camera_id = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool parseCommandLine(int argc, char** argv, CommandLineOptions* options,
                      bool* help_requested, std::string* error) {
  if (!options || !help_requested) return false;
  *help_requested = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help" || argument == "-h") {
      *help_requested = true;
      return true;
    }
    if (argument == "--show-spherical-coverage") {
      options->show_spherical_coverage = true;
      continue;
    }
    if (argument == "--show-epipolar-curve") {
      options->show_epipolar_curve = true;
      continue;
    }
    if (argument == "--show-temporal-features") {
      options->show_temporal_features = true;
      continue;
    }
    if (index + 1 >= argc) {
      if (error) *error = "missing value after " + argument;
      return false;
    }
    const std::string value(argv[++index]);
    if (argument == "--config") {
      options->config_file = value;
    } else if (argument == "--bag") {
      options->bag_path = value;
    } else if (argument == "--rate") {
      if (!parseDouble(value, &options->playback_rate)) {
        if (error) *error = "invalid --rate value: " + value;
        return false;
      }
    } else if (argument == "--start-offset") {
      if (!parseDouble(value, &options->start_offset)) {
        if (error) *error = "invalid --start-offset value: " + value;
        return false;
      }
      options->has_start_offset = true;
    } else if (argument == "--duration") {
      if (!parseDouble(value, &options->duration)) {
        if (error) *error = "invalid --duration value: " + value;
        return false;
      }
      options->has_duration = true;
    } else if (argument == "--imu-gap-warning") {
      if (!parseDouble(value, &options->imu_gap_warning)) {
        if (error) *error = "invalid --imu-gap-warning value: " + value;
        return false;
      }
    } else if (argument == "--cameras") {
      options->camera_config_file = value;
    } else if (argument == "--frontend-config") {
      options->frontend_config_file = value;
    } else if (argument == "--epipolar-source-camera") {
      if (!parseCameraId(value, &options->epipolar_source_camera)) {
        if (error) *error = "invalid source camera id: " + value;
        return false;
      }
    } else if (argument == "--epipolar-target-camera") {
      if (!parseCameraId(value, &options->epipolar_target_camera)) {
        if (error) *error = "invalid target camera id: " + value;
        return false;
      }
    } else if (argument == "--epipolar-source-u") {
      if (!parseDouble(value, &options->epipolar_source_u)) {
        if (error) *error = "invalid source pixel u: " + value;
        return false;
      }
    } else if (argument == "--epipolar-source-v") {
      if (!parseDouble(value, &options->epipolar_source_v)) {
        if (error) *error = "invalid source pixel v: " + value;
        return false;
      }
    } else {
      if (error) *error = "unknown argument: " + argument;
      return false;
    }
  }
  if (options->config_file.empty()) {
    if (error) *error = "--config is required";
    return false;
  }
  if (options->playback_rate <= 0.0) {
    if (error) *error = "--rate must be greater than zero";
    return false;
  }
  if (options->imu_gap_warning <= 0.0) {
    if (error) *error = "--imu-gap-warning must be greater than zero";
    return false;
  }
  if (options->has_start_offset && options->start_offset < 0.0) {
    if (error) *error = "--start-offset cannot be negative";
    return false;
  }
  if (options->show_epipolar_curve &&
      options->epipolar_source_camera == options->epipolar_target_camera) {
    if (error) *error = "epipolar source and target cameras must differ";
    return false;
  }
  if (options->epipolar_source_u < 0.0 &&
      options->epipolar_source_u != -1.0) {
    if (error) *error = "epipolar source u cannot be negative";
    return false;
  }
  if (options->epipolar_source_v < 0.0 &&
      options->epipolar_source_v != -1.0) {
    if (error) *error = "epipolar source v cannot be negative";
    return false;
  }
  return true;
}

std::string siblingCameraConfigPath(const std::string& offline_config_path) {
  const std::size_t separator = offline_config_path.find_last_of("/\\");
  if (separator == std::string::npos) return "cameras.yaml";
  return offline_config_path.substr(0, separator + 1) + "cameras.yaml";
}

std::string siblingSystemConfigPath(const std::string& offline_config_path) {
  const std::size_t separator = offline_config_path.find_last_of("/\\");
  if (separator == std::string::npos) return "system.yaml";
  return offline_config_path.substr(0, separator + 1) + "system.yaml";
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

  sphere_vio::OfflineBagConfiguration bag_configuration;
  if (!sphere_vio::loadOfflineBagConfiguration(
          command_line.config_file, command_line.bag_path, &bag_configuration,
          &error)) {
    std::cerr << "Invalid offline configuration: " << error << std::endl;
    return EXIT_FAILURE;
  }
  if (command_line.has_start_offset) {
    bag_configuration.start_time_offset = command_line.start_offset;
  }
  if (command_line.has_duration) {
    bag_configuration.duration = command_line.duration;
  }

  sphere_vio::OfflineBagVisualizerOptions options;
  options.bag = std::move(bag_configuration);
  options.playback_rate = command_line.playback_rate;
  options.imu_gap_warning = command_line.imu_gap_warning;
  options.show_spherical_coverage = command_line.show_spherical_coverage;
  options.show_epipolar_curve = command_line.show_epipolar_curve;
  options.show_temporal_features = command_line.show_temporal_features;
  options.camera_config_file = command_line.camera_config_file.empty()
                                   ? siblingCameraConfigPath(
                                         command_line.config_file)
                                   : command_line.camera_config_file;
  options.epipolar_source_camera = command_line.epipolar_source_camera;
  options.epipolar_target_camera = command_line.epipolar_target_camera;
  options.epipolar_source_u = command_line.epipolar_source_u;
  options.epipolar_source_v = command_line.epipolar_source_v;
  if (options.show_temporal_features) {
    const std::string frontend_config =
        command_line.frontend_config_file.empty()
            ? siblingSystemConfigPath(command_line.config_file)
            : command_line.frontend_config_file;
    if (!sphere_vio::loadTemporalFrontendOptions(frontend_config,
                                                  &options.frontend, &error)) {
      std::cerr << "Invalid frontend configuration: " << error << std::endl;
      return EXIT_FAILURE;
    }
  }
  return sphere_vio::OfflineBagVisualizer(std::move(options)).run();
}
