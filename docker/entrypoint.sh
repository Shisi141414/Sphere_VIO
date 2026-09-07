#!/usr/bin/env bash
set -e

# Every container command needs the ROS and catkin workspace setup scripts.
# Sourcing them here means "docker run ... rosrun sphere_vio ..." works
# without the user typing the source commands every time.
#
# catkin's setup.sh passes the current positional arguments to _setup_util.py,
# so clear "$@" before sourcing and restore the original command afterwards.
original_args=("$@")
set --

source /opt/ros/noetic/setup.bash
if [ -f /root/catkin_ws/devel/setup.bash ]; then
  source /root/catkin_ws/devel/setup.bash
fi

exec "${original_args[@]}"
