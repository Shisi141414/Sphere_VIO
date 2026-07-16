#include "sphere_vio/ros/offline_panorama_runner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <yaml-cpp/yaml.h>

#include "sphere_vio/common/camera_rig_loader.hpp"
#include "sphere_vio/ros/frame_assembler.hpp"

namespace sphere_vio {
namespace {

bool topicMatches(const std::string& recorded, const std::string& configured) {
  return recorded == configured || "/" + recorded == configured;
}

bool parseRotation(const YAML::Node& node, Eigen::Matrix3d* rotation) {
  if (!rotation || !node || node["rows"].as<int>() != 3 ||
      node["cols"].as<int>() != 3 || !node["data"].IsSequence() ||
      node["data"].size() != 9U) return false;
  for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column)
    (*rotation)(row, column) = node["data"][row * 3 + column].as<double>();
  return rotation->allFinite();
}

bool saveFirstFrame(const std::string& directory,
                    const PanoramaRemapResult& result) {
  if (directory.empty()) return true;
  for (int id = 0; id < 4; ++id) {
    if (!cv::imwrite(directory + "/frame0000_c" + std::to_string(id) + ".png",
                     result.layers[id].image)) return false;
  }
  cv::Mat coverage_display, owner_display(result.owner_camera_id.size(),
                                           CV_8UC1, cv::Scalar(0));
  result.coverage_count.convertTo(coverage_display, CV_8UC1, 255.0 / 4.0);
  for (int y = 0; y < owner_display.rows; ++y) for (int x = 0; x < owner_display.cols; ++x) {
    const int owner = result.owner_camera_id.at<std::int8_t>(y, x);
    owner_display.at<std::uint8_t>(y, x) = owner < 0 ? 0 : 50 + owner * 60;
  }
  return cv::imwrite(directory + "/frame0000_coverage.png", coverage_display) &&
         cv::imwrite(directory + "/frame0000_owner.png", owner_display) &&
         cv::imwrite(directory + "/frame0000_composite.png",
                     result.owner_selected_composite);
}

}  // namespace

bool loadPanoramaConfiguration(const std::string& path, PanoramaSpec* panorama,
                               PanoramaRemapOptions* remap,
                               std::string* error) {
  if (!panorama || !remap) return false;
  try {
    const YAML::Node p = YAML::LoadFile(path)["panorama"];
    if (!p || p["model"].as<std::string>() != "uspm") {
      if (error) *error = "panorama.model must be uspm";
      return false;
    }
    panorama->width = p["width"].as<int>();
    panorama->height = p["height"].as<int>();
    panorama->horizontal_fov = p["horizontal_fov"].as<double>();
    panorama->vertical_fov = p["vertical_fov"].as<double>();
    panorama->sphere_radius = p["sphere_radius"].as<double>();
    if (!parseRotation(p["R_p_b"], &panorama->frame.R_p_b)) {
      if (error) *error = "panorama.R_p_b must be a finite 3x3 matrix";
      return false;
    }
    const YAML::Node r = p["remap"];
    if (r) {
      const std::string interpolation = r["interpolation"].as<std::string>();
      const std::string border = r["border_mode"].as<std::string>();
      if (interpolation != "linear" || border != "constant") {
        if (error) *error = "only linear interpolation and constant border are supported";
        return false;
      }
      remap->interpolation = cv::INTER_LINEAR;
      remap->border_mode = cv::BORDER_CONSTANT;
      remap->sampling_margin = r["sampling_margin"].as<double>();
      remap->parallel_cameras = r["parallel_cameras"].as<bool>();
      if (r["owner_policy"].as<std::string>() !=
          "maximum_camera_forward_component") {
        if (error) *error = "unsupported panorama owner_policy";
        return false;
      }
    }
  } catch (const YAML::Exception& exception) {
    if (error) *error = exception.what();
    return false;
  }
  return true;
}

OfflinePanoramaRunner::OfflinePanoramaRunner(OfflinePanoramaRunnerOptions options)
    : options_(std::move(options)) {}

int OfflinePanoramaRunner::run() {
  CameraRig rig; std::string error;
  if (!loadCameraRigFromYaml(options_.camera_config_file, &rig, &error)) {
    std::cerr << "Cannot load CameraRig: " << error << std::endl; return 2;
  }
  PanoramaRemapper remapper;
  if (!remapper.initialize(rig, options_.panorama, options_.remap, &error)) {
    std::cerr << "Cannot initialize PanoramaRemapper: " << error << std::endl; return 3;
  }
  std::cout << std::fixed << std::setprecision(3)
            << "USPM static map: " << remapper.precomputeTimeMs() << " ms, "
            << remapper.staticMapBytes() / (1024.0 * 1024.0) << " MiB\n";
  const double pixels = options_.panorama.width * options_.panorama.height;
  for (const auto& map : remapper.cameraRemaps())
    std::cout << "  C" << map.camera_id << " valid=" << map.valid_pixel_count
              << " (" << 100.0 * map.valid_pixel_count / pixels << "%)\n";
  std::array<std::size_t, 5> coverage{};
  for (int y = 0; y < remapper.coverageCount().rows; ++y)
    for (int x = 0; x < remapper.coverageCount().cols; ++x)
      ++coverage[remapper.coverageCount().at<std::uint8_t>(y, x)];
  for (int count = 0; count <= 4; ++count)
    std::cout << "  coverage[" << count << "]=" << coverage[count]
              << " (" << 100.0 * coverage[count] / pixels << "%)\n";
  for (const auto& overlap : remapper.configuredOverlaps())
    std::cout << "  overlap C" << overlap.camera_id_1 << "-C"
              << overlap.camera_id_2 << "=" << overlap.pixel_count << '\n';
  std::array<std::size_t, 4> owners{};
  std::size_t unowned = 0U;
  for (int y = 0; y < remapper.ownerCameraId().rows; ++y)
    for (int x = 0; x < remapper.ownerCameraId().cols; ++x) {
      const int owner = remapper.ownerCameraId().at<std::int8_t>(y, x);
      if (owner < 0) ++unowned; else ++owners[owner];
    }
  std::cout << "  owner[-1]=" << unowned << '\n';
  for (int id = 0; id < 4; ++id)
    std::cout << "  owner[C" << id << "]=" << owners[id] << " ("
              << 100.0 * owners[id] / pixels << "%)\n";

  rosbag::Bag bag;
  try { bag.open(options_.bag.bag_path, rosbag::bagmode::Read); }
  catch (const rosbag::BagException& exception) {
    std::cerr << "Cannot open bag: " << exception.what() << std::endl; return 4;
  }
  const std::vector<std::string> topics(options_.bag.camera_topics.begin(),
                                         options_.bag.camera_topics.end());
  rosbag::View complete(bag, rosbag::TopicQuery(topics));
  const ros::Time begin = complete.getBeginTime() +
                          ros::Duration(options_.bag.start_time_offset);
  ros::Time end = complete.getEndTime();
  if (options_.bag.duration >= 0)
    end = std::min(end, begin + ros::Duration(options_.bag.duration));
  rosbag::View view(bag, rosbag::TopicQuery(topics), begin, end);
  FrameAssembler assembler(options_.bag.maximum_image_time_difference,
                           options_.bag.require_exact_image_timestamps);
  std::uint64_t succeeded = 0, failed = 0;
  double total_ms = 0, maximum_ms = 0, composite_ms = 0;
  bool saved = false;
  for (const auto& instance : view) {
    for (CameraId id = 0; id < 4; ++id) {
      if (!topicMatches(instance.getTopic(), options_.bag.camera_topics[id])) continue;
      const auto message = instance.instantiate<sensor_msgs::Image>();
      if (!message) break;
      MultiCameraFrame frame;
      if (!assembler.addImage(id, message, &frame)) break;
      std::array<cv::Mat, 4> images;
      for (const ImageFrame& image : frame.images) {
        if (image.camera_id < 4U) images[image.camera_id] = image.image;
      }
      PanoramaRemapResult result;
      if (!remapper.remap(images, &result, &error)) {
        ++failed; std::cerr << "Remap failed: " << error << std::endl; break;
      }
      ++succeeded; total_ms += result.processing_time_ms;
      maximum_ms = std::max(maximum_ms, result.processing_time_ms);
      composite_ms += result.composite_time_ms;
      if (succeeded == 1U) {
        for (int camera = 0; camera < 4; ++camera) {
          double minimum = 0.0, maximum = 0.0;
          cv::minMaxLoc(result.layers[camera].image, &minimum, &maximum);
          std::cout << "  first layer C" << camera << " range=[" << minimum
                    << ',' << maximum << "] nonzero="
                    << cv::countNonZero(result.layers[camera].image) << '\n';
        }
        double minimum = 0.0, maximum = 0.0;
        cv::minMaxLoc(result.owner_selected_composite, &minimum, &maximum);
        std::cout << "  first composite range=[" << minimum << ',' << maximum
                  << "] nonzero="
                  << cv::countNonZero(result.owner_selected_composite) << '\n';
      }
      if (!saved && !options_.save_first_frame_directory.empty()) {
        if (!saveFirstFrame(options_.save_first_frame_directory, result)) {
          std::cerr << "Cannot save first-frame diagnostics" << std::endl; return 5;
        }
        saved = true;
      }
      break;
    }
  }
  bag.close();
  const auto& stats = assembler.statistics();
  std::cout << std::fixed << std::setprecision(6)
            << "USPM panorama bag summary\n  complete frames: " << stats.completed_frames
            << "\n  dropped / mismatch: " << stats.dropped_images << " / "
            << stats.timestamp_mismatches << "\n  remap success / failure: "
            << succeeded << " / " << failed << "\n  remap ms average / max: "
            << (succeeded ? total_ms / succeeded : 0) << " / " << maximum_ms
            << "\n  composite average ms: "
            << (succeeded ? composite_ms / succeeded : 0) << std::endl;
  return failed == 0 ? 0 : 6;
}

}  // namespace sphere_vio
