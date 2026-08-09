# Demo runtime for xgo_localization.
#
# Two targets, built and published separately:
#
#   core   linux/amd64 + linux/arm64 — the node and both SLAM backends. Enough
#          to replay a bag anywhere: Mac, Windows, Linux.
#   robot  linux/arm64 only — core plus the XGO drivers and the Pi camera stack.
#          Those depend on a Raspberry Pi libcamera build and cannot be built
#          for amd64, which is why the split exists.
#
# Everything upstream is pinned by commit SHA. Tags move; SHAs do not, and a
# pinned clone caches properly instead of needing a cache-busting hack.

FROM ros:jazzy-ros-base@sha256:da725acf8b0f9f30c683e33ffbdcd6482d077af96d6fdc7688c5f4f280b7d923 AS core

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Docker Desktop's storage driver can break apt signature checks
# ("At least one invalid signature was encountered"). Disabling the sandbox
# user sidesteps it and is harmless inside a container.
RUN echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/00no-sandbox

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        python3-pip \
        ros-jazzy-cartographer \
        ros-jazzy-cartographer-ros \
        ros-jazzy-rtabmap-slam \
        ros-jazzy-rtabmap-odom \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# --- the artifact -----------------------------------------------------------
# Fetched by SHA. Bump this one ARG to take a new version of the node; the
# change is visible in the diff and the image rebuilds from here down.
ARG NODE_SHA=ca8acfbb9743ec746065f5170422a6d475eaec09
RUN mkdir -p /ws/src/xgo_localization && cd /ws/src/xgo_localization && \
    git init -q && \
    git remote add origin https://github.com/luckyTamme/xgo_localization && \
    git fetch -q --depth 1 origin ${NODE_SHA} && \
    git checkout -q FETCH_HEAD && \
    rm -rf .git

RUN cd /ws && source /opt/ros/jazzy/setup.bash && \
    colcon build --merge-install && \
    rm -rf build log

# `docker exec` bypasses the entrypoint, so without this a shell into a running
# container has no ros2 on PATH — the first thing anyone tries when something
# looks wrong. Both files are needed: profile.d covers login shells
# (`bash -lc ...`), bash.bashrc covers interactive ones (`exec -it ... bash`).
RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      | tee /etc/profile.d/ros.sh >> /etc/bash.bashrc

COPY launch /demo/launch
COPY config /demo/config
COPY lichtblick /demo/lichtblick
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /demo
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]


# ============================================================================
FROM core AS robot
# Adds what only exists on the robot: the XGO driver over UART, the LiDAR
# driver, and the Pi camera stack. arm64 only.

# libcamera from the Raspberry Pi fork. The distro package crashes its IPA
# proxy on Pi Trixie kernels, so it is replaced rather than supplemented, and
# installed over /opt/ros/jazzy so camera_ros links against this copy.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake meson ninja-build pkg-config \
        libyaml-dev python3-yaml python3-ply python3-jinja2 \
        libgnutls28-dev openssl libdw-dev libunwind-dev \
        libudev-dev libevent-dev \
        ros-jazzy-camera-ros \
        ros-jazzy-image-transport-plugins \
        python3-serial \
    && rm -rf /var/lib/apt/lists/*

ARG LIBCAMERA_REF=v0.7.1+rpt20260429
RUN git clone --depth 1 --branch ${LIBCAMERA_REF} \
        https://github.com/raspberrypi/libcamera.git /tmp/libcamera && \
    cd /tmp/libcamera && \
    meson setup build --prefix=/opt/ros/jazzy \
        -Dpipelines=rpi/vc4 -Dipas=rpi/vc4 \
        -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled \
        -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled && \
    ninja -C build install && ldconfig && \
    rm -rf /tmp/libcamera

# wjwwood/cxx_serial — xgo2_ros links it and ROS 2 has no packaged equivalent
# of the ROS 1 serial library, so it goes in system-wide.
ARG CXX_SERIAL_SHA=869511ef94e08ea608d90a3ec4854d696f33fa29
RUN mkdir -p /tmp/cxx_serial && cd /tmp/cxx_serial && \
    git init -q && git remote add origin https://github.com/wjwwood/cxx_serial && \
    git fetch -q --depth 1 origin ${CXX_SERIAL_SHA} && git checkout -q FETCH_HEAD && \
    make CMAKE_FLAGS=-DCMAKE_INSTALL_PREFIX=/usr/local && \
    make install && ldconfig && \
    rm -rf /tmp/cxx_serial

# The XGO driver fork and the LiDAR driver, both pinned.
#
# Names to keep straight: the repository is xgo_ros, the ROS package inside it
# is xgo2_ros, and the executable is xgo2_ros_node.
#
# The fork exists because XGO Mini 2 firmware M-5.1.1 dropped the autofeedback
# streaming upstream relies on, which makes the upstream driver hang. It polls
# registers instead and publishes /xgo/applied_vel — the velocity this whole
# stack integrates.
ARG XGO_ROS_SHA=1f7003687ab33d053b8008beff509aebed37b76a
ARG LDLIDAR_SHA=bf668a89baf722a787dadc442860dcbf33a82f5a

# Two upstream quirks are patched right after the clones:
#   - xgo_ros' CMakeLists installs meshes/ and urdf/ directories that the
#     repository never committed, so they are created empty.
#   - ldlidar uses pthread_mutex_* without including <pthread.h>. gcc 11 pulled
#     it in transitively; gcc 13 on Noble does not.
RUN mkdir -p /ws/src/xgo_ros /ws/src/ldlidar_stl_ros2 && \
    cd /ws/src/xgo_ros && git init -q && \
      git remote add origin https://github.com/luckyTamme/xgo_ros && \
      git fetch -q --depth 1 origin ${XGO_ROS_SHA} && git checkout -q FETCH_HEAD && rm -rf .git && \
    cd /ws/src/ldlidar_stl_ros2 && git init -q && \
      git remote add origin https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2 && \
      git fetch -q --depth 1 origin ${LDLIDAR_SHA} && git checkout -q FETCH_HEAD && rm -rf .git && \
    mkdir -p /ws/src/xgo_ros/meshes /ws/src/xgo_ros/urdf && \
    sed -i '/^#include "log_module.h"$/a #include <pthread.h>' \
        /ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp

RUN cd /ws && source /opt/ros/jazzy/setup.bash && \
    apt-get update && \
    rosdep install --from-paths src -y --ignore-src \
        --skip-keys="libcamera ament_lint_auto ament_lint_common" && \
    colcon build --merge-install && \
    rm -rf build log /var/lib/apt/lists/*
