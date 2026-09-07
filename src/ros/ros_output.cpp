#include "sphere_vio/ros/ros_output.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointField.h>

namespace sphere_vio {
namespace {

geometry_msgs::Point toPoint(const Eigen::Vector3d& vector) {
  geometry_msgs::Point point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

geometry_msgs::Vector3 toVector3(const Eigen::Vector3d& vector) {
  geometry_msgs::Vector3 result;
  result.x = vector.x();
  result.y = vector.y();
  result.z = vector.z();
  return result;
}

geometry_msgs::Quaternion toQuaternion(const Eigen::Quaterniond& quaternion) {
  geometry_msgs::Quaternion result;
  result.w = quaternion.w();
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  return result;
}

sensor_msgs::PointCloud2 makePointCloud(
    const std::vector<BackendLandmark>& landmarks,
    const ros::Time& stamp) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = "world";
  cloud.height = 1U;
  cloud.width = static_cast<std::uint32_t>(landmarks.size());
  cloud.is_bigendian = false;
  cloud.point_step = 12U;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_dense = true;

  sensor_msgs::PointField field_x;
  field_x.name = "x";
  field_x.offset = 0U;
  field_x.datatype = sensor_msgs::PointField::FLOAT32;
  field_x.count = 1U;
  sensor_msgs::PointField field_y = field_x;
  field_y.name = "y";
  field_y.offset = 4U;
  sensor_msgs::PointField field_z = field_x;
  field_z.name = "z";
  field_z.offset = 8U;
  cloud.fields = {field_x, field_y, field_z};

  cloud.data.resize(cloud.row_step);
  for (std::size_t index = 0U; index < landmarks.size(); ++index) {
    float* point = reinterpret_cast<float*>(cloud.data.data() + index * 12U);
    point[0] = static_cast<float>(landmarks[index].point_w.x());
    point[1] = static_cast<float>(landmarks[index].point_w.y());
    point[2] = static_cast<float>(landmarks[index].point_w.z());
  }
  return cloud;
}

}  // namespace

RosOutput::RosOutput()
    : transform_broadcaster_() {
  odometry_publisher_ =
      node_handle_.advertise<nav_msgs::Odometry>("/sphere_vio/odometry", 10);
  path_publisher_ =
      node_handle_.advertise<nav_msgs::Path>("/sphere_vio/path", 10);
  landmarks_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          "/sphere_vio/landmarks", 10);
  path_.header.frame_id = "world";
}

void RosOutput::publish(
    const EskfState& state,
    const Eigen::Matrix<double, 15, 15>& covariance,
    const std::vector<BackendLandmark>& landmarks) {
  const ros::Time stamp(state.timestamp);

  nav_msgs::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = "world";
  odometry.child_frame_id = "body";
  odometry.pose.pose.position = toPoint(state.p_wb);
  odometry.pose.pose.orientation = toQuaternion(state.q_wb);
  odometry.twist.twist.linear = toVector3(state.v_wb);
  odometry.twist.twist.angular = toVector3(state.bias_gyro);
  // Keep a conservative diagonal covariance so RViz and downstream consumers
  // can render the output without a custom covariance layout.
  odometry.pose.covariance[0] = covariance(0, 0);
  odometry.pose.covariance[7] = covariance(1, 1);
  odometry.pose.covariance[14] = covariance(2, 2);
  odometry.twist.covariance[0] = covariance(6, 6);
  odometry.twist.covariance[7] = covariance(7, 7);
  odometry.twist.covariance[14] = covariance(8, 8);
  odometry_publisher_.publish(odometry);

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = "world";
  pose.pose = odometry.pose.pose;
  path_.header.stamp = stamp;
  path_.poses.push_back(pose);
  path_publisher_.publish(path_);

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "world";
  transform.child_frame_id = "body";
  transform.transform.translation.x = state.p_wb.x();
  transform.transform.translation.y = state.p_wb.y();
  transform.transform.translation.z = state.p_wb.z();
  transform.transform.rotation = toQuaternion(state.q_wb);
  transform_broadcaster_.sendTransform(transform);

  landmarks_publisher_.publish(makePointCloud(landmarks, stamp));
}

}  // namespace sphere_vio
