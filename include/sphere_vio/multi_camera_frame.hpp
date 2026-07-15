#pragma once

#include <array>
#include <vector>

#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

namespace sphere_vio
{

/**
 * @brief 一组完成时间同步的四目图像及其对应 IMU 区间。
 *
 * imu_measurements 保存的时间范围为：
 *
 *     previous_stamp < imu_stamp <= stamp
 */
struct MultiCameraFrame
{
    ros::Time previous_stamp;
    ros::Time stamp;

    std::array<sensor_msgs::ImageConstPtr, 4> images;

    std::vector<sensor_msgs::ImuConstPtr> imu_measurements;
};

}  // namespace sphere_vio