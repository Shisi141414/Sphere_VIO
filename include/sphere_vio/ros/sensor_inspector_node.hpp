#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>

namespace sphere_vio {

class SensorInspectorNode {
 public:
  SensorInspectorNode(ros::NodeHandle node_handle,
                      ros::NodeHandle private_node_handle);

 private:
  static constexpr std::size_t kCameraCount = 4;

  struct StreamStatistics {
    std::uint64_t messages = 0;
    std::uint64_t sequence_gap_count = 0;
    std::uint64_t timestamp_gap_count = 0;
    std::uint64_t non_monotonic_timestamps = 0;
    std::uint64_t zero_timestamps = 0;
    bool has_previous = false;
    std::uint32_t previous_sequence = 0;
    ros::Time previous_stamp;
    ros::WallTime first_arrival;
    ros::WallTime last_arrival;
    double interval_sum = 0.0;
    std::uint64_t interval_count = 0;
  };

  using Image = sensor_msgs::Image;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      Image, Image, Image, Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void imageCallback(std::size_t camera_index, const ImageConstPtr& image);
  void synchronizedImageCallback(const ImageConstPtr& image0,
                                 const ImageConstPtr& image1,
                                 const ImageConstPtr& image2,
                                 const ImageConstPtr& image3);
  void imuCallback(const sensor_msgs::ImuConstPtr& imu);
  void reportCallback(const ros::TimerEvent& event);
  void updateStatistics(const std_msgs::Header& header,
                        const ros::WallTime& arrival,
                        double expected_period,
                        StreamStatistics* statistics);
  double measuredRate(const StreamStatistics& statistics) const;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  std::array<std::string, kCameraCount> camera_topics_;
  std::string imu_topic_;
  std::array<std::unique_ptr<message_filters::Subscriber<Image>>, kCameraCount>
      image_subscribers_;
  std::unique_ptr<Synchronizer> image_synchronizer_;
  ros::Subscriber imu_subscriber_;
  ros::Timer report_timer_;

  std::array<StreamStatistics, kCameraCount> image_statistics_;
  StreamStatistics imu_statistics_;
  std::array<std::uint32_t, kCameraCount> image_widths_{};
  std::array<std::uint32_t, kCameraCount> image_heights_{};
  std::array<std::string, kCameraCount> image_encodings_;
  std::array<bool, kCameraCount> image_metadata_changed_{};
  std::uint64_t synchronized_sets_ = 0;
  std::uint64_t synchronization_violations_ = 0;
  double largest_image_time_difference_ = 0.0;

  int image_queue_size_ = 10;
  int imu_queue_size_ = 2000;
  double maximum_image_time_difference_ = 0.003;
  double expected_image_rate_ = 0.0;
  double expected_imu_rate_ = 0.0;
  double delayed_period_factor_ = 1.5;
  double report_period_ = 5.0;
};

}  // namespace sphere_vio
