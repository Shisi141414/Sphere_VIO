#pragma once

#include <sensor_msgs/Imu.h>

#include "sphere_vio/common/types.hpp"

namespace sphere_vio {

bool convertImuMessage(const sensor_msgs::Imu& message,
                       ImuMeasurement* measurement);

}  // namespace sphere_vio
