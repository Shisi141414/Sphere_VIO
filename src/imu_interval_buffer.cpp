#include "sphere_vio/imu_interval_buffer.hpp"

#include <algorithm>
#include <limits>

namespace sphere_vio {

ImuIntervalBuffer::ImuIntervalBuffer(double large_interval_threshold)
    : large_interval_threshold_(large_interval_threshold) {}

bool ImuIntervalBuffer::add(const ImuMeasurement& measurement) {
  ++statistics_.received_measurements;
  bool strictly_increasing = true;
  if (has_last_received_timestamp_) {
    const double interval = measurement.timestamp - last_received_timestamp_;
    if (interval == 0.0) {
      ++statistics_.duplicate_timestamps;
      strictly_increasing = false;
    } else if (interval < 0.0) {
      ++statistics_.non_monotonic_timestamps;
      strictly_increasing = false;
    } else {
      if (statistics_.interval_count == 0) {
        statistics_.minimum_interval = interval;
        statistics_.maximum_interval = interval;
      } else {
        statistics_.minimum_interval =
            std::min(statistics_.minimum_interval, interval);
        statistics_.maximum_interval =
            std::max(statistics_.maximum_interval, interval);
      }
      statistics_.interval_sum += interval;
      ++statistics_.interval_count;
      if (large_interval_threshold_ > 0.0 &&
          interval > large_interval_threshold_) {
        ++statistics_.large_intervals;
      }
    }
  }
  has_last_received_timestamp_ = true;
  last_received_timestamp_ = measurement.timestamp;

  const auto position = std::upper_bound(
      measurements_.begin(), measurements_.end(), measurement.timestamp,
      [](Timestamp timestamp, const ImuMeasurement& queued) {
        return timestamp < queued.timestamp;
      });
  measurements_.insert(position, measurement);
  return strictly_increasing;
}

std::vector<ImuMeasurement> ImuIntervalBuffer::extract(
    Timestamp previous_time, Timestamp current_time) {
  std::vector<ImuMeasurement> interval;
  if (current_time <= previous_time) return interval;
  while (!measurements_.empty() &&
         measurements_.front().timestamp <= previous_time) {
    measurements_.pop_front();
  }
  while (!measurements_.empty() &&
         measurements_.front().timestamp <= current_time) {
    interval.push_back(measurements_.front());
    measurements_.pop_front();
  }
  return interval;
}

void ImuIntervalBuffer::discardThrough(Timestamp time) {
  while (!measurements_.empty() && measurements_.front().timestamp <= time) {
    measurements_.pop_front();
  }
}

std::size_t ImuIntervalBuffer::size() const { return measurements_.size(); }

const ImuIntervalBuffer::Statistics& ImuIntervalBuffer::statistics() const {
  return statistics_;
}

}  // namespace sphere_vio
