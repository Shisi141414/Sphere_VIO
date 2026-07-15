#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "sphere_vio/ros/offline_bag_visualizer.hpp"

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
};

void printUsage() {
  std::cout
      << "Usage: sphere_vio_bag_visualizer --config FILE --bag BAG [options]\n"
         "Options:\n"
         "  --rate VALUE             Playback rate (default: 1.0)\n"
         "  --start-offset SECONDS   Offset from bag start\n"
         "  --duration SECONDS       Negative means through bag end\n"
         "  --imu-gap-warning SEC    Display warning threshold (default: 0.008)\n"
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
  return true;
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
  return sphere_vio::OfflineBagVisualizer(std::move(options)).run();
}
