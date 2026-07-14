# Sphere-VIO Reproduction

A ROS Noetic implementation and reproduction study of Sphere-VIO for a
four-camera omnidirectional fisheye camera system with an IMU.

## Development platforms

### Development computer

- Ubuntu 20.04.6 LTS
- x86_64
- ROS Noetic
- OpenCV 4.2.0
- Eigen 3.3.7
- GCC 9.4.0
- CMake 3.16.3

### Deployment device

- Ubuntu 20.04.6 LTS
- AArch64
- ARM Cortex-A55
- ROS Noetic

## Build

```bash
cd ~/catkin_ws

catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="sphere_vio"

source devel/setup.bash