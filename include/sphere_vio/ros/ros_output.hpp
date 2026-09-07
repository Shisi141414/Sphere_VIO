#pragma once

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>

#include "sphere_vio/backend/eskf.hpp"
#include "sphere_vio/backend/landmark_map.hpp"

namespace sphere_vio {

// ROS1 interaction layer: odometry, path, TF, and map-point output.
class RosOutput {
 public:
  RosOutput();

  void publish(const EskfState& state,
               const Eigen::Matrix<double, 15, 15>& covariance,
               const std::vector<BackendLandmark>& landmarks);

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher odometry_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher landmarks_publisher_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  nav_msgs::Path path_;
};

}  // namespace sphere_vio
