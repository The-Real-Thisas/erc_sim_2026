FROM osrf/ros:humble-desktop AS base
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# 0. HTTPS for apt
RUN sed -i 's|http://archive.ubuntu.com|https://archive.ubuntu.com|g' /etc/apt/sources.list

# 1. Tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-vcstool python3-colcon-common-extensions python3-rosdep \
    python3-pip python3-venv git wget curl lsb-release gnupg2 ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# 2. ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    ros-humble-effort-controllers \
    ros-humble-realsense2-description \
    ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# Message definitions only: ros_gz_interfaces carries the Contacts message the
# contact sensors publish, and depends on nothing from Gazebo (geometry_msgs,
# std_msgs, rcl_interfaces and the rosidl runtime). Keeping the type means team
# code written against the Gazebo build's /contacts still compiles and runs.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

# Recording: the MCAP storage plugin, so `ros2 bag record -s mcap` writes a
# file Foxglove replays directly (the stock image records sqlite3 only), and
# the JPEG image transport, so a recorded camera is megabytes, not gigabytes.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-compressed-image-transport \
    && rm -rf /var/lib/apt/lists/*

# 3. MuJoCo backend
# mujoco_ros2_control is vendored in src/, so this installs only what building
# it needs: the MuJoCo library itself, the CMake package both its CMakeLists
# require, the GL headers its camera plugin links against, and the Python the
# URDF-to-MJCF converter imports. ros2_control_cmake would normally arrive with
# the binary mujoco_ros2_control_plugins, which is deliberately not installed
# here - without it both packages fail at find_package().
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mujoco-vendor \
    ros-humble-ros2-control-cmake \
    ros-humble-urdfdom-py \
    ros-humble-backward-ros \
    python3-pykdl \
    python3-numpy \
    libglfw3-dev \
    libegl-dev \
    libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# The converter runs from its own venv. Building it here rather than letting
# the converter create it on first launch keeps the promise that no network
# access is needed after cloning.
COPY src/mujoco_ros2_control/mujoco_ros2_control/scripts/requirements.txt /tmp/mjcf-requirements.txt
RUN python3 -m venv --system-site-packages /root/.ros/ros2_control/.venv \
    && source /root/.ros/ros2_control/.venv/bin/activate \
    && pip install --no-cache-dir --disable-pip-version-check \
        -r /tmp/mjcf-requirements.txt pycollada \
    && rm /tmp/mjcf-requirements.txt

# 4. Workspace directory (source is mounted from host at runtime)
ENV PYTHONDONTWRITEBYTECODE=1
RUN mkdir -p /opt/erc_ws/src

# 5. Entrypoint
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /opt/erc_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
