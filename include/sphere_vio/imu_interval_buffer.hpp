#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "sphere_vio/common/types.hpp"

namespace sphere_vio {

class ImuIntervalBuffer {
 public:
  struct Statistics {
    std::uint64_t received_measurements = 0;
    std::uint64_t non_monotonic_timestamps = 0;
    std::uint64_t duplicate_timestamps = 0;
    std::uint64_t large_intervals = 0;
    double minimum_interval = 0.0;
    double maximum_interval = 0.0;
    double interval_sum = 0.0;
    std::uint64_t interval_count = 0;
  };

  explicit ImuIntervalBuffer(double large_interval_threshold = 0.05);

  bool add(const ImuMeasurement& measurement);
  std::vector<ImuMeasurement> extract(Timestamp previous_time,
                                      Timestamp current_time);
  void discardThrough(Timestamp time);
  std::size_t size() const;
  const Statistics& statistics() const;

 private:
  double large_interval_threshold_;
  bool has_last_received_timestamp_ = false;
  Timestamp last_received_timestamp_ = 0.0;
  std::deque<ImuMeasurement> measurements_;
  Statistics statistics_;
};

}  // namespace sphere_vio
