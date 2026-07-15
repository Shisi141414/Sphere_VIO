#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>

#include "sphere_vio/multi_camera_frame.hpp"

namespace sphere_vio {
namespace {

constexpr std::size_t kCameraCount = 4;

}  // namespace

class OfflineFrameAssembler {
 public:
  using Image = sensor_msgs::Image;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  using ExactSyncPolicy = message_filters::sync_policies::ExactTime<
      Image, Image, Image, Image>;
  using Synchronizer = message_filters::Synchronizer<ExactSyncPolicy>;

  OfflineFrameAssembler() : private_node_handle_("~") {
    private_node_handle_.param("camera0_topic", camera_topics_[0],
                               std::string("/fisheye/left/image_raw"));
    private_node_handle_.param("camera1_topic", camera_topics_[1],
                               std::string("/fisheye/right/image_raw"));
    private_node_handle_.param("camera2_topic", camera_topics_[2],
                               std::string("/fisheye/bleft/image_raw"));
    private_node_handle_.param("camera3_topic", camera_topics_[3],
                               std::string("/fisheye/bright/image_raw"));
    private_node_handle_.param("imu_topic", imu_topic_,
                               std::string("/imu_data_raw"));
    private_node_handle_.param("sync_queue_size", sync_queue_size_, 3);
    private_node_handle_.param("imu_queue_size", imu_queue_size_, 2000);
    private_node_handle_.param("expected_image_rate", expected_image_rate_,
                               20.0);
    private_node_handle_.param("expected_imu_rate", expected_imu_rate_, 200.0);
    private_node_handle_.param("report_every_n_frames", report_every_n_frames_,
                               20);

    sync_queue_size_ = std::max(1, sync_queue_size_);
    imu_queue_size_ = std::max(1, imu_queue_size_);
    report_every_n_frames_ = std::max(0, report_every_n_frames_);
    // There is deliberately no extra public tuning parameter for this queue.
    // This limit bounds retained full-resolution image messages if IMU stops.
    maximum_pending_frames_ =
        static_cast<std::size_t>(std::max(2, sync_queue_size_ * 10));

    for (std::size_t index = 0; index < kCameraCount; ++index) {
      camera_subscribers_[index].reset(
          new message_filters::Subscriber<Image>(
              node_handle_, camera_topics_[index], sync_queue_size_));
      camera_subscribers_[index]->registerCallback(boost::bind(
          &OfflineFrameAssembler::cameraInputCallback, this, index,
          boost::placeholders::_1));
    }
    synchronizer_.reset(new Synchronizer(
        ExactSyncPolicy(static_cast<std::uint32_t>(sync_queue_size_)),
        *camera_subscribers_[0], *camera_subscribers_[1],
        *camera_subscribers_[2], *camera_subscribers_[3]));
    synchronizer_->registerCallback(boost::bind(
        &OfflineFrameAssembler::cameraCallback, this, boost::placeholders::_1,
        boost::placeholders::_2, boost::placeholders::_3,
        boost::placeholders::_4));

    imu_subscriber_ = node_handle_.subscribe(
        imu_topic_, static_cast<std::uint32_t>(imu_queue_size_),
        &OfflineFrameAssembler::imuCallback, this);
    diagnostic_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(5.0), &OfflineFrameAssembler::diagnosticCallback,
        this);

    ROS_INFO_STREAM("Offline frame assembler started. Cameras: ["
                    << camera_topics_[0] << ", " << camera_topics_[1] << ", "
                    << camera_topics_[2] << ", " << camera_topics_[3]
                    << "], IMU: " << imu_topic_
                    << ", exact sync queue: " << sync_queue_size_
                    << ", IMU buffer limit: " << imu_queue_size_);
  }

 private:
  struct PendingCameraFrame {
    ros::Time stamp;
    std::array<ImageConstPtr, kCameraCount> images;
  };

  struct ReadyFrame {
    MultiCameraFrame frame;
    std::size_t pending_queue_size = 0;
    std::size_t imu_queue_size = 0;
  };

  void cameraInputCallback(std::size_t camera_index, const ImageConstPtr&) {
    ++camera_message_counts_[camera_index];
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& imu) {
    ++imu_message_count_;
    bool non_monotonic = false;
    bool overflow = false;
    ros::Time previous_latest;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      previous_latest = latest_imu_stamp_;
      non_monotonic = !latest_imu_stamp_.isZero() &&
                      imu->header.stamp < latest_imu_stamp_;

      const auto position = std::upper_bound(
          imu_queue_.begin(), imu_queue_.end(), imu->header.stamp,
          [](const ros::Time& stamp, const sensor_msgs::ImuConstPtr& queued) {
            return stamp < queued->header.stamp;
          });
      imu_queue_.insert(position, imu);
      if (latest_imu_stamp_.isZero() || imu->header.stamp > latest_imu_stamp_) {
        latest_imu_stamp_ = imu->header.stamp;
      }
      while (imu_queue_.size() > static_cast<std::size_t>(imu_queue_size_)) {
        imu_queue_.pop_front();
        overflow = true;
      }
    }
    if (non_monotonic) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Non-monotonic IMU timestamp: latest=" << previous_latest
                                                        << ", received="
                                                        << imu->header.stamp);
    }
    if (overflow) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "IMU buffer reached its limit; oldest samples were discarded");
    }
    drainReadyFrames();
  }

  void cameraCallback(const ImageConstPtr& image0, const ImageConstPtr& image1,
                      const ImageConstPtr& image2,
                      const ImageConstPtr& image3) {
    ++synchronized_frame_count_;
    PendingCameraFrame pending{image0->header.stamp,
                               {image0, image1, image2, image3}};
    bool non_monotonic = false;
    bool overflow = false;
    ros::Time previous_stamp;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!pending_camera_frames_.empty()) {
        previous_stamp = pending_camera_frames_.back().stamp;
        non_monotonic = pending.stamp <= previous_stamp;
      } else if (has_previous_frame_) {
        previous_stamp = previous_frame_stamp_;
        non_monotonic = pending.stamp <= previous_stamp;
      }
      if (!non_monotonic) {
        pending_camera_frames_.push_back(std::move(pending));
        while (pending_camera_frames_.size() > maximum_pending_frames_) {
          pending_camera_frames_.pop_front();
          overflow = true;
        }
      }
    }
    if (non_monotonic) {
      ROS_WARN_STREAM("Dropping non-monotonic camera frame: previous="
                      << previous_stamp << ", received=" << image0->header.stamp);
    }
    if (overflow) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Pending camera queue reached its limit; oldest frame was "
               "discarded while waiting for IMU");
    }
    drainReadyFrames();
  }

  void drainReadyFrames() {
    // Serializes drains so frames are processed in timestamp order, while the
    // data mutex itself is only held for inexpensive queue operations.
    std::lock_guard<std::mutex> drain_lock(drain_mutex_);
    std::vector<ReadyFrame> ready_frames;
    bool initialized = false;
    ros::Time initialized_stamp;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      while (!pending_camera_frames_.empty() &&
             !latest_imu_stamp_.isZero() &&
             latest_imu_stamp_ >= pending_camera_frames_.front().stamp) {
        PendingCameraFrame current =
            std::move(pending_camera_frames_.front());
        pending_camera_frames_.pop_front();

        if (!has_previous_frame_) {
          while (!imu_queue_.empty() &&
                 imu_queue_.front()->header.stamp <= current.stamp) {
            imu_queue_.pop_front();
          }
          previous_frame_stamp_ = current.stamp;
          has_previous_frame_ = true;
          initialized = true;
          initialized_stamp = current.stamp;
          continue;
        }

        MultiCameraFrame frame;
        frame.previous_stamp = previous_frame_stamp_;
        frame.stamp = current.stamp;
        frame.images = std::move(current.images);
        while (!imu_queue_.empty() &&
               imu_queue_.front()->header.stamp <= frame.previous_stamp) {
          imu_queue_.pop_front();
        }
        while (!imu_queue_.empty() &&
               imu_queue_.front()->header.stamp <= frame.stamp) {
          frame.imu_measurements.push_back(imu_queue_.front());
          imu_queue_.pop_front();
        }
        previous_frame_stamp_ = frame.stamp;
        ready_frames.push_back(
            {std::move(frame), pending_camera_frames_.size(), imu_queue_.size()});
      }
    }

    if (initialized) {
      ROS_INFO_STREAM("Initialized IMU interval at first camera stamp "
                      << initialized_stamp << "; no algorithm frame emitted");
    }
    for (const ReadyFrame& ready : ready_frames) {
      processFrame(ready);
    }
  }

  void processFrame(const ReadyFrame& ready) {
    const MultiCameraFrame& frame = ready.frame;
    std::array<cv_bridge::CvImageConstPtr, kCameraCount> cv_images;
    try {
      for (std::size_t index = 0; index < kCameraCount; ++index) {
        // toCvShare preserves shared ownership and does not clone image data.
        cv_images[index] = cv_bridge::toCvShare(
            frame.images[index], sensor_msgs::image_encodings::MONO8);
      }
    } catch (const cv_bridge::Exception& exception) {
      ROS_ERROR_STREAM("Cannot read synchronized images through cv_bridge: "
                       << exception.what());
      return;
    }

    const double interval = (frame.stamp - frame.previous_stamp).toSec();
    const double expected_imu_count = interval * expected_imu_rate_;
    const double expected_image_period =
        expected_image_rate_ > 0.0 ? 1.0 / expected_image_rate_ : 0.0;
    const std::uint64_t processed_frame_count = ++processed_frame_count_;

    if (frame.imu_measurements.empty()) {
      ROS_WARN_STREAM("Empty IMU interval (" << frame.previous_stamp << ", "
                                              << frame.stamp << "]");
    }
    if (expected_image_period > 0.0 &&
        (interval > 1.5 * expected_image_period ||
         interval < 0.5 * expected_image_period)) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Abnormal image interval: " << interval << " s (expected "
                                            << expected_image_period << " s)");
    }
    if (expected_imu_count > 0.0 &&
        std::abs(static_cast<double>(frame.imu_measurements.size()) -
                 expected_imu_count) /
                expected_imu_count >
            0.4) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Unexpected IMU count: actual="
                   << frame.imu_measurements.size()
                   << ", theoretical=" << expected_imu_count);
    }

    if (report_every_n_frames_ > 0 &&
        processed_frame_count %
                static_cast<std::uint64_t>(report_every_n_frames_) ==
            0) {
      std::ostringstream report;
      report << std::fixed << std::setprecision(6) << "Assembled frame "
             << processed_frame_count << "\n  stamp: " << frame.stamp.toSec()
             << "\n  image interval: " << interval << " s\n  images: ";
      for (std::size_t index = 0; index < kCameraCount; ++index) {
        if (index > 0) report << ", ";
        report << "camera" << index << "=" << cv_images[index]->image.cols
               << "x" << cv_images[index]->image.rows << "/"
               << frame.images[index]->encoding;
      }
      report << "\n  IMU samples: " << frame.imu_measurements.size()
             << " (theoretical=" << expected_imu_count << ")"
             << "\n  pending camera queue: " << ready.pending_queue_size
             << "\n  IMU buffer: " << ready.imu_queue_size;
      ROS_INFO_STREAM(report.str());
    }
  }

  void diagnosticCallback(const ros::WallTimerEvent&) {
    std::array<std::uint64_t, kCameraCount> camera_counts{};
    for (std::size_t index = 0; index < kCameraCount; ++index) {
      camera_counts[index] = camera_message_counts_[index].load();
    }
    const std::uint64_t synchronized_count = synchronized_frame_count_.load();
    const std::uint64_t imu_count = imu_message_count_.load();
    const std::uint64_t processed_count = processed_frame_count_.load();

    std::size_t pending_count = 0;
    std::size_t buffered_imu_count = 0;
    ros::Time oldest_pending_stamp;
    ros::Time latest_imu_stamp;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      pending_count = pending_camera_frames_.size();
      buffered_imu_count = imu_queue_.size();
      latest_imu_stamp = latest_imu_stamp_;
      if (!pending_camera_frames_.empty()) {
        oldest_pending_stamp = pending_camera_frames_.front().stamp;
      }
    }

    std::ostringstream report;
    report << "Offline assembler status: camera messages=["
           << camera_counts[0] << ", " << camera_counts[1] << ", "
           << camera_counts[2] << ", " << camera_counts[3]
           << "], exact synchronized sets=" << synchronized_count
           << ", IMU messages=" << imu_count
           << ", completed frames=" << processed_count
           << ", pending images=" << pending_count
           << ", buffered IMU=" << buffered_imu_count << ". ";

    if (camera_counts[0] == 0 && camera_counts[1] == 0 &&
        camera_counts[2] == 0 && camera_counts[3] == 0 && imu_count == 0) {
      report << "Waiting for input. Start rosbag play with --clock and verify "
                "the configured topic names.";
    } else if (camera_counts[0] == 0 || camera_counts[1] == 0 ||
               camera_counts[2] == 0 || camera_counts[3] == 0) {
      report << "Waiting for one or more camera topics.";
    } else if (synchronized_count == 0) {
      report << "Camera messages arrived, but ExactTime found no four-camera "
                "timestamp match.";
    } else if (imu_count == 0) {
      report << "Synchronized images arrived, but no IMU messages arrived.";
    } else if (pending_count > 0 && latest_imu_stamp < oldest_pending_stamp) {
      report << "Waiting for IMU to reach oldest image stamp "
             << oldest_pending_stamp << " (latest IMU " << latest_imu_stamp
             << ").";
    } else {
      report << "Input and assembly are active.";
    }
    ROS_INFO_STREAM(report.str());
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  std::array<std::string, kCameraCount> camera_topics_;
  std::string imu_topic_;
  std::array<std::unique_ptr<message_filters::Subscriber<Image>>, kCameraCount>
      camera_subscribers_;
  std::unique_ptr<Synchronizer> synchronizer_;
  ros::Subscriber imu_subscriber_;
  ros::WallTimer diagnostic_timer_;

  int sync_queue_size_ = 3;
  int imu_queue_size_ = 2000;
  int report_every_n_frames_ = 20;
  double expected_image_rate_ = 20.0;
  double expected_imu_rate_ = 200.0;
  std::size_t maximum_pending_frames_ = 30;

  std::mutex data_mutex_;
  std::mutex drain_mutex_;
  std::deque<sensor_msgs::ImuConstPtr> imu_queue_;
  std::deque<PendingCameraFrame> pending_camera_frames_;
  ros::Time latest_imu_stamp_;
  ros::Time previous_frame_stamp_;
  bool has_previous_frame_ = false;
  std::array<std::atomic<std::uint64_t>, kCameraCount> camera_message_counts_{};
  std::atomic<std::uint64_t> synchronized_frame_count_{0};
  std::atomic<std::uint64_t> imu_message_count_{0};
  std::atomic<std::uint64_t> processed_frame_count_{0};
};

}  // namespace sphere_vio

int main(int argc, char** argv) {
  ros::init(argc, argv, "offline_frame_assembler");
  sphere_vio::OfflineFrameAssembler assembler;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
