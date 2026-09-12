# Sphere-VIO is a ROS 1 Noetic catkin package.  Use an official ROS Noetic
# image so the build and the offline bag tools see the expected ABI.
FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS=/root/catkin_ws

# Use Tsinghua TUNA mirrors for Ubuntu/ROS. The default archive.ubuntu.com
# and packages.ros.org are international; on a China network they are slow,
# and a local proxy (e.g. Clash on 127.0.0.1:7897) may reset large downloads.
# Switching to a domestic mirror avoids both problems.
RUN sed -i 's|archive.ubuntu.com|mirrors.tuna.tsinghua.edu.cn|g; s|security.ubuntu.com|mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list \
    && find /etc/apt/sources.list.d -name '*.list' -exec sed -i 's|packages.ros.org|mirrors.tuna.tsinghua.edu.cn|g' {} +

# Keep the image deterministic and small: install only the dependencies used by
# package.xml, the CMakeLists.txt targets, and the OpenCV GUI tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ca-certificates \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    python3-numpy \
    libgtk-3-0 \
    libgl1 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libxkbcommon-x11-0 \
    xauth \
    ros-noetic-catkin \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    ros-noetic-rosbag \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${CATKIN_WS}
RUN mkdir -p src
COPY . src/sphere_vio

# catkin_make must run with the ROS environment loaded.  Build only this
# package to avoid pulling unrelated catkin packages into the workspace.
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES=\"sphere_vio\""

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR ${CATKIN_WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
