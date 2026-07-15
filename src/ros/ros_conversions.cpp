#include "sphere_vio/ros/ros_conversions.hpp"

namespace sphere_vio {

bool convertImuMessage(const sensor_msgs::Imu& message,
                       ImuMeasurement* measurement) {
  if (!measurement) return false;
  measurement->timestamp = message.header.stamp.toSec();
  measurement->acceleration << message.linear_acceleration.x,
      message.linear_acceleration.y, message.linear_acceleration.z;
  measurement->angular_velocity << message.angular_velocity.x,
      message.angular_velocity.y, message.angular_velocity.z;
  return true;
}

}  // namespace sphere_vio
