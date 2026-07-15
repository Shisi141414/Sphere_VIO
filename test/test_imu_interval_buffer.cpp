#include <gtest/gtest.h>

#include "sphere_vio/imu_interval_buffer.hpp"

namespace sphere_vio {
namespace {

ImuMeasurement measurement(double timestamp) {
  ImuMeasurement value;
  value.timestamp = timestamp;
  return value;
}

TEST(ImuIntervalBufferTest, ExtractsOpenClosedInterval) {
  ImuIntervalBuffer buffer;
  buffer.add(measurement(1.0));
  buffer.add(measurement(1.5));
  buffer.add(measurement(2.0));
  buffer.add(measurement(2.5));
  const std::vector<ImuMeasurement> values = buffer.extract(1.0, 2.0);
  ASSERT_EQ(2U, values.size());
  EXPECT_DOUBLE_EQ(1.5, values[0].timestamp);
  EXPECT_DOUBLE_EQ(2.0, values[1].timestamp);
  EXPECT_EQ(1U, buffer.size());
}

TEST(ImuIntervalBufferTest, DetectsDuplicateAndNonMonotonicTimestamps) {
  ImuIntervalBuffer buffer;
  EXPECT_TRUE(buffer.add(measurement(2.0)));
  EXPECT_FALSE(buffer.add(measurement(2.0)));
  EXPECT_FALSE(buffer.add(measurement(1.0)));
  EXPECT_EQ(1U, buffer.statistics().duplicate_timestamps);
  EXPECT_EQ(1U, buffer.statistics().non_monotonic_timestamps);
}

}  // namespace
}  // namespace sphere_vio
