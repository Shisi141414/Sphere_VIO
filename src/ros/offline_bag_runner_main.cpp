#include <iostream>
#include <string>

#include "sphere_vio/ros/offline_bag_runner.hpp"

namespace {

bool readRemapping(const std::string& argument, const std::string& key,
                   std::string* value) {
  const std::string prefix = "_" + key + ":=";
  if (argument.compare(0, prefix.size(), prefix) != 0) return false;
  *value = argument.substr(prefix.size());
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_file;
  std::string bag_path;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--config" && index + 1 < argc) {
      config_file = argv[++index];
    } else if (argument == "--bag" && index + 1 < argc) {
      bag_path = argv[++index];
    } else if (!readRemapping(argument, "config_file", &config_file)) {
      readRemapping(argument, "bag_path", &bag_path);
    }
  }
  if (config_file.empty()) {
    std::cerr << "Usage: sphere_vio_bag_runner --config FILE [--bag BAG]\n"
                 "   or: sphere_vio_bag_runner _config_file:=FILE "
                 "_bag_path:=BAG"
              << std::endl;
    return 1;
  }
  sphere_vio::OfflineBagConfiguration configuration;
  std::string error;
  if (!sphere_vio::loadOfflineBagConfiguration(config_file, bag_path,
                                                &configuration, &error)) {
    std::cerr << "Invalid offline configuration: " << error << std::endl;
    return 1;
  }
  return sphere_vio::OfflineBagRunner(std::move(configuration)).run();
}
