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
```

## Deterministic offline rosbag runner

The offline runner reads the four configured image topics and one IMU topic
directly with the rosbag C++ API. It does not require `roscore`, live sensors,
or `rosbag play`.

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

rosrun sphere_vio sphere_vio_bag_runner \
  _config_file:=/root/catkin_ws/src/sphere_vio/config/offline.yaml \
  _bag_path:=/path/to/data.bag
```

Equivalent long options are available for environments where ROS remapping
arguments are inconvenient:

```bash
rosrun sphere_vio sphere_vio_bag_runner \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /path/to/data.bag
```

The first completed image frame establishes the left boundary of the first IMU
interval. Measurements at and before that timestamp are discarded. Each later
frame extracts IMU measurements in `(previous_image_time, current_image_time]`.

## Basic offline visualization

The raw-sensor visualizer displays synchronized images as `left/right` over
`bleft/bright`, together with image timing and per-frame IMU interval status.
It applies no camera projection and does not require `roscore`.

```bash
/root/catkin_ws/devel/lib/sphere_vio/sphere_vio_bag_visualizer \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /absolute/path/to/data.bag \
  --rate 1.0 \
  --imu-gap-warning 0.008
```

Use Space to pause or resume, N to advance one frame while paused, and Q or
Esc to quit.

Add the optional sparse Body-bearing ERP coverage panel with:

```bash
/root/catkin_ws/devel/lib/sphere_vio/sphere_vio_bag_visualizer \
  --config /root/catkin_ws/src/sphere_vio/config/offline.yaml \
  --bag /absolute/path/to/data.bag \
  --show-spherical-coverage
```

The panel uses `config/cameras.yaml` by default, plots only calibrated sparse
bearing samples, and performs no image stitching or depth estimation. Use
`--cameras FILE` to select another camera calibration explicitly.
