#include "sphere_vio/ros/sensor_inspector_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

#include <boost/bind/bind.hpp>

namespace sphere_vio {
namespace {

constexpr double kMinimumPeriod = 1e-9;

}  // namespace

SensorInspectorNode::SensorInspectorNode(ros::NodeHandle node_handle,
                                         ros::NodeHandle private_node_handle)
    : node_handle_(std::move(node_handle)),
      private_node_handle_(std::move(private_node_handle)) {
  for (std::size_t index = 0; index < kCameraCount; ++index) {
    const std::string default_topic =
        "/camera" + std::to_string(index) + "/image_raw";
    private_node_handle_.param("topics/camera" + std::to_string(index),
                               camera_topics_[index], default_topic);
  }
  private_node_handle_.param("topics/imu", imu_topic_,
                             std::string("/imu/data_raw"));
  private_node_handle_.param("synchronization/image_queue_size",
                             image_queue_size_, image_queue_size_);
  private_node_handle_.param("synchronization/imu_queue_size", imu_queue_size_,
                             imu_queue_size_);
  private_node_handle_.param(
      "synchronization/maximum_image_time_difference",
      maximum_image_time_difference_, maximum_image_time_difference_);
  private_node_handle_.param("inspection/expected_image_rate",
                             expected_image_rate_, expected_image_rate_);
  private_node_handle_.param("inspection/expected_imu_rate", expected_imu_rate_,
                             expected_imu_rate_);
  private_node_handle_.param("inspection/delayed_period_factor",
                             delayed_period_factor_, delayed_period_factor_);
  private_node_handle_.param("inspection/report_period", report_period_,
                             report_period_);

  image_queue_size_ = std::max(1, image_queue_size_);
  imu_queue_size_ = std::max(1, imu_queue_size_);
  report_period_ = std::max(0.1, report_period_);
  delayed_period_factor_ = std::max(1.0, delayed_period_factor_);

  for (std::size_t index = 0; index < kCameraCount; ++index) {
    image_subscribers_[index].reset(
        new message_filters::Subscriber<Image>(node_handle_,
                                                camera_topics_[index],
                                                image_queue_size_));
    image_subscribers_[index]->registerCallback(
        boost::bind(&SensorInspectorNode::imageCallback, this, index,
                    boost::placeholders::_1));
  }

  image_synchronizer_.reset(new Synchronizer(
      SyncPolicy(static_cast<std::uint32_t>(image_queue_size_)),
      *image_subscribers_[0], *image_subscribers_[1], *image_subscribers_[2],
      *image_subscribers_[3]));
  image_synchronizer_->registerCallback(boost::bind(
      &SensorInspectorNode::synchronizedImageCallback, this,
      boost::placeholders::_1, boost::placeholders::_2,
      boost::placeholders::_3, boost::placeholders::_4));

  imu_subscriber_ = node_handle_.subscribe(
      imu_topic_, static_cast<std::uint32_t>(imu_queue_size_),
      &SensorInspectorNode::imuCallback, this);
  report_timer_ = node_handle_.createTimer(
      ros::Duration(report_period_), &SensorInspectorNode::reportCallback, this);

  ROS_INFO_STREAM("Sensor inspector started. Cameras: ["
                  << camera_topics_[0] << ", " << camera_topics_[1] << ", "
                  << camera_topics_[2] << ", " << camera_topics_[3]
                  << "], IMU: " << imu_topic_);
}

void SensorInspectorNode::imageCallback(std::size_t camera_index,
                                        const ImageConstPtr& image) {
  updateStatistics(image->header, ros::WallTime::now(),
                   expected_image_rate_ > 0.0 ? 1.0 / expected_image_rate_ : 0.0,
                   &image_statistics_[camera_index]);

  if (image_statistics_[camera_index].messages == 1) {
    image_widths_[camera_index] = image->width;
    image_heights_[camera_index] = image->height;
    image_encodings_[camera_index] = image->encoding;
    ROS_INFO_STREAM("camera" << camera_index << " "
                              << camera_topics_[camera_index] << ": "
                              << image->width << "x" << image->height << " "
                              << image->encoding << ", first header stamp="
                              << std::fixed << std::setprecision(9)
                              << image->header.stamp.toSec());
    return;
  }

  if (image->width != image_widths_[camera_index] ||
      image->height != image_heights_[camera_index] ||
      image->encoding != image_encodings_[camera_index]) {
    image_metadata_changed_[camera_index] = true;
    ROS_WARN_STREAM_THROTTLE(
        5.0, "camera" << camera_index
                       << " resolution/encoding changed from "
                       << image_widths_[camera_index] << "x"
                       << image_heights_[camera_index] << " "
                       << image_encodings_[camera_index] << " to "
                       << image->width << "x" << image->height << " "
                       << image->encoding);
  }
}

void SensorInspectorNode::synchronizedImageCallback(
    const ImageConstPtr& image0, const ImageConstPtr& image1,
    const ImageConstPtr& image2, const ImageConstPtr& image3) {
  const std::array<ros::Time, kCameraCount> stamps{
      image0->header.stamp, image1->header.stamp, image2->header.stamp,
      image3->header.stamp};
  const auto minimum = std::min_element(stamps.begin(), stamps.end());
  const auto maximum = std::max_element(stamps.begin(), stamps.end());
  const double difference = (*maximum - *minimum).toSec();
  ++synchronized_sets_;
  largest_image_time_difference_ =
      std::max(largest_image_time_difference_, difference);
  if (difference > maximum_image_time_difference_) {
    ++synchronization_violations_;
    ROS_WARN_STREAM_THROTTLE(
        2.0, "Four-camera timestamp difference " << difference
                                                   << " s exceeds limit "
                                                   << maximum_image_time_difference_
                                                   << " s");
  }
}

void SensorInspectorNode::imuCallback(const sensor_msgs::ImuConstPtr& imu) {
  updateStatistics(imu->header, ros::WallTime::now(),
                   expected_imu_rate_ > 0.0 ? 1.0 / expected_imu_rate_ : 0.0,
                   &imu_statistics_);
  if (imu_statistics_.messages == 1) {
    ROS_INFO_STREAM("IMU " << imu_topic_ << ": first header stamp="
                            << std::fixed << std::setprecision(9)
                            << imu->header.stamp.toSec());
  }
}

void SensorInspectorNode::updateStatistics(
    const std_msgs::Header& header, const ros::WallTime& arrival,
    double expected_period, StreamStatistics* statistics) {
  ++statistics->messages;
  if (header.stamp.isZero()) {
    ++statistics->zero_timestamps;
  }

  if (!statistics->has_previous) {
    statistics->has_previous = true;
    statistics->previous_sequence = header.seq;
    statistics->previous_stamp = header.stamp;
    statistics->first_arrival = arrival;
    statistics->last_arrival = arrival;
    return;
  }

  if (header.seq > statistics->previous_sequence + 1U) {
    statistics->sequence_gap_count +=
        static_cast<std::uint64_t>(header.seq - statistics->previous_sequence - 1U);
  }

  const double stamp_interval = (header.stamp - statistics->previous_stamp).toSec();
  if (stamp_interval <= 0.0) {
    ++statistics->non_monotonic_timestamps;
  } else {
    statistics->interval_sum += stamp_interval;
    ++statistics->interval_count;
    if (expected_period > kMinimumPeriod &&
        stamp_interval > delayed_period_factor_ * expected_period) {
      ++statistics->timestamp_gap_count;
    }
  }

  statistics->previous_sequence = header.seq;
  statistics->previous_stamp = header.stamp;
  statistics->last_arrival = arrival;
}

double SensorInspectorNode::measuredRate(
    const StreamStatistics& statistics) const {
  if (statistics.interval_count == 0 ||
      statistics.interval_sum <= kMinimumPeriod) {
    return 0.0;
  }
  return static_cast<double>(statistics.interval_count) /
         statistics.interval_sum;
}

void SensorInspectorNode::reportCallback(const ros::TimerEvent&) {
  std::ostringstream report;
  report << std::fixed << std::setprecision(3) << "Sensor inspector report";
  for (std::size_t index = 0; index < kCameraCount; ++index) {
    const StreamStatistics& statistics = image_statistics_[index];
    report << "\n  camera" << index << " topic=" << camera_topics_[index]
           << " format=";
    if (statistics.messages == 0) {
      report << "unknown";
    } else {
      report << image_widths_[index] << "x" << image_heights_[index] << "/"
             << image_encodings_[index];
    }
    report << " rate=" << measuredRate(statistics)
           << " Hz messages=" << statistics.messages
           << " sequence_gap_count=" << statistics.sequence_gap_count
           << " timestamp_gap_count=" << statistics.timestamp_gap_count
           << " non_monotonic=" << statistics.non_monotonic_timestamps
           << " zero_stamps=" << statistics.zero_timestamps
           << " metadata_changed="
           << (image_metadata_changed_[index] ? "yes" : "no");
  }
  report << "\n  four_camera synchronized_sets=" << synchronized_sets_
         << " maximum_stamp_difference=" << largest_image_time_difference_
         << " s limit=" << maximum_image_time_difference_
         << " s violations=" << synchronization_violations_;
  report << "\n  imu topic=" << imu_topic_
         << " rate=" << measuredRate(imu_statistics_)
         << " Hz messages=" << imu_statistics_.messages
         << " sequence_gap_count=" << imu_statistics_.sequence_gap_count
         << " timestamp_gap_count=" << imu_statistics_.timestamp_gap_count
         << " non_monotonic=" << imu_statistics_.non_monotonic_timestamps
         << " zero_stamps=" << imu_statistics_.zero_timestamps;
  ROS_INFO_STREAM(report.str());
}

}  // namespace sphere_vio
