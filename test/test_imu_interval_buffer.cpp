#include <gtest/gtest.h>

#include "sphere_vio/imu_interval_buffer.hpp"

namespace sphere_vio {
namespace {

ImuMeasurement measurement(double timestamp) {
  ImuMeasurement value;
  value.timestamp = timestamp;
  return value;
}

TEST(ImuIntervalBufferTest, ExtractsClosedIntervalWithSynthesizedBoundaries) {
  ImuIntervalBuffer buffer;
  buffer.add(measurement(1.0));
  buffer.add(measurement(1.5));
  buffer.add(measurement(2.0));
  buffer.add(measurement(2.5));
  const std::vector<ImuMeasurement> values = buffer.extract(1.0, 2.0);
  // The rewritten buffer keeps both interval endpoints so MSCKF integration
  // never loses the left boundary sample from the previous image interval.
  ASSERT_EQ(3U, values.size());
  EXPECT_DOUBLE_EQ(1.0, values[0].timestamp);
  EXPECT_DOUBLE_EQ(1.5, values[1].timestamp);
  EXPECT_DOUBLE_EQ(2.0, values[2].timestamp);
  EXPECT_EQ(1U, buffer.size());
}

TEST(ImuIntervalBufferTest, InterpolatesEndpointsBetweenSamples) {
  ImuIntervalBuffer buffer;
  ImuMeasurement first = measurement(1.0);
  first.acceleration = Eigen::Vector3d(1.0, 0.0, 0.0);
  ImuMeasurement second = measurement(2.0);
  second.acceleration = Eigen::Vector3d(3.0, 0.0, 0.0);
  buffer.add(first);
  buffer.add(second);
  const std::vector<ImuMeasurement> values = buffer.extract(1.25, 1.75);
  ASSERT_EQ(2U, values.size());
  EXPECT_DOUBLE_EQ(1.25, values[0].timestamp);
  EXPECT_NEAR(1.5, values[0].acceleration.x(), 1e-12);
  EXPECT_DOUBLE_EQ(1.75, values[1].timestamp);
  EXPECT_NEAR(2.5, values[1].acceleration.x(), 1e-12);
  EXPECT_GE(buffer.statistics().boundary_interpolations, 2U);
}

TEST(ImuIntervalBufferTest, DetectsDuplicateAndNonMonotonicTimestamps) {
  ImuIntervalBuffer buffer;
  EXPECT_TRUE(buffer.add(measurement(2.0)));
  EXPECT_FALSE(buffer.add(measurement(2.0)));
  EXPECT_FALSE(buffer.add(measurement(1.0)));
  EXPECT_EQ(1U, buffer.statistics().duplicate_timestamps);
  EXPECT_EQ(1U, buffer.statistics().non_monotonic_timestamps);
}

TEST(ImuIntervalBufferTest, ConsecutiveIntervalsKeepSharedBoundary) {
  ImuIntervalBuffer buffer;
  buffer.add(measurement(1.0));
  buffer.add(measurement(1.5));
  buffer.add(measurement(2.0));
  buffer.add(measurement(2.5));
  buffer.add(measurement(3.0));

  const std::vector<ImuMeasurement> first = buffer.extract(1.0, 2.0);
  ASSERT_EQ(3U, first.size());
  EXPECT_DOUBLE_EQ(1.0, first[0].timestamp);
  EXPECT_DOUBLE_EQ(2.0, first[2].timestamp);

  // 第二段与前一段共享 2.0 这个左边界。若上一帧弹出边界样本时没有保留副本，
  // 这里会丢掉 2.0，导致 2.0->2.5 段被 2.5->3.0 错误覆盖（这正是时间轴修复
  // 要消除的旧 bug）。
  const std::vector<ImuMeasurement> second = buffer.extract(2.0, 3.0);
  ASSERT_EQ(3U, second.size());
  EXPECT_DOUBLE_EQ(2.0, second[0].timestamp);
  EXPECT_DOUBLE_EQ(2.5, second[1].timestamp);
  EXPECT_DOUBLE_EQ(3.0, second[2].timestamp);
}

TEST(ImuIntervalBufferTest, LargeGapFallsBackToInterpolatedEndpoints) {
  ImuIntervalBuffer buffer;
  buffer.add(measurement(1.0));
  buffer.add(measurement(1.5));
  buffer.add(measurement(5.0));
  buffer.add(measurement(5.5));

  // 2.0 位于 [1.5, 5.0] 大间隙内：左边界在 1.5/5.0 之间插值，右边界保持
  // 最新值，避免滤波时钟在无样本的情况下凭空推进。
  const std::vector<ImuMeasurement> gap = buffer.extract(1.0, 2.0);
  ASSERT_EQ(3U, gap.size());
  EXPECT_DOUBLE_EQ(1.0, gap[0].timestamp);
  EXPECT_DOUBLE_EQ(1.5, gap[1].timestamp);
  EXPECT_DOUBLE_EQ(2.0, gap[2].timestamp);
  EXPECT_GE(buffer.statistics().boundary_holds, 1U);
}

}  // namespace
}  // namespace sphere_vio
