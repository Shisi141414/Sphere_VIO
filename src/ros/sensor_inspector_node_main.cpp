#include <ros/ros.h>

#include "sphere_vio/ros/sensor_inspector_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_inspector");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");
  sphere_vio::SensorInspectorNode inspector(node_handle, private_node_handle);
  ros::spin();
  return 0;
}
