#include "sphere_vio/imu_interval_buffer.hpp"

#include <algorithm>
#include <cmath>
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

  const ImuMeasurement* before_previous =
      has_retained_measurement_ ? &retained_measurement_ : nullptr;
  const ImuMeasurement* first_after_previous = nullptr;
  const ImuMeasurement* before_current = before_previous;
  const ImuMeasurement* first_after_current = nullptr;

  for (const ImuMeasurement& measurement : measurements_) {
    if (measurement.timestamp <= previous_time) {
      before_previous = &measurement;
      before_current = &measurement;
      continue;
    }
    if (!first_after_previous) first_after_previous = &measurement;
    if (measurement.timestamp <= current_time) {
      before_current = &measurement;
      continue;
    }
    first_after_current = &measurement;
    break;
  }

  const auto append_unique = [&interval](const ImuMeasurement& measurement) {
    if (interval.empty() ||
        std::abs(interval.back().timestamp - measurement.timestamp) > 1e-12) {
      interval.push_back(measurement);
    }
  };

  if (before_previous && first_after_previous &&
      before_previous->timestamp < previous_time &&
      first_after_previous->timestamp > previous_time) {
    append_unique(interpolate(*before_previous, *first_after_previous,
                              previous_time));
    ++statistics_.boundary_interpolations;
  } else if (before_previous && before_previous->timestamp == previous_time) {
    append_unique(*before_previous);
  } else if (first_after_previous) {
    ImuMeasurement held = *first_after_previous;
    held.timestamp = previous_time;
    append_unique(held);
    ++statistics_.boundary_holds;
  } else {
    return interval;
  }

  for (const ImuMeasurement& measurement : measurements_) {
    if (measurement.timestamp > previous_time &&
        measurement.timestamp < current_time) {
      append_unique(measurement);
    }
  }

  if (before_current && first_after_current &&
      before_current->timestamp < current_time &&
      first_after_current->timestamp > current_time) {
    append_unique(interpolate(*before_current, *first_after_current,
                              current_time));
    ++statistics_.boundary_interpolations;
  } else if (before_current && before_current->timestamp == current_time) {
    append_unique(*before_current);
  } else if (before_current) {
    // Offline rosbag iteration often reaches an image before its first IMU
    // sample after that image timestamp. Hold the newest value instead of
    // silently advancing the filter clock without an integration segment.
    ImuMeasurement held = *before_current;
    held.timestamp = current_time;
    append_unique(held);
    ++statistics_.boundary_holds;
  } else {
    interval.clear();
    return interval;
  }

  while (!measurements_.empty() &&
         measurements_.front().timestamp <= current_time) {
    retained_measurement_ = measurements_.front();
    has_retained_measurement_ = true;
    measurements_.pop_front();
  }
  return interval;
}

void ImuIntervalBuffer::discardThrough(Timestamp time) {
  while (!measurements_.empty() && measurements_.front().timestamp <= time) {
    retained_measurement_ = measurements_.front();
    has_retained_measurement_ = true;
    measurements_.pop_front();
  }
}

ImuMeasurement ImuIntervalBuffer::interpolate(const ImuMeasurement& first,
                                               const ImuMeasurement& second,
                                               Timestamp timestamp) {
  ImuMeasurement result = first;
  result.timestamp = timestamp;
  const double duration = second.timestamp - first.timestamp;
  if (!std::isfinite(duration) || duration <= 0.0) return result;
  const double ratio = std::max(0.0, std::min(
      1.0, (timestamp - first.timestamp) / duration));
  result.acceleration =
      first.acceleration + ratio * (second.acceleration - first.acceleration);
  result.angular_velocity = first.angular_velocity +
      ratio * (second.angular_velocity - first.angular_velocity);
  return result;
}

std::size_t ImuIntervalBuffer::size() const { return measurements_.size(); }

const ImuIntervalBuffer::Statistics& ImuIntervalBuffer::statistics() const {
  return statistics_;
}

}  // namespace sphere_vio
