FROM osrf/ros:humble-desktop AS base
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# 0. HTTPS for apt
RUN sed -i 's|http://archive.ubuntu.com|https://archive.ubuntu.com|g' /etc/apt/sources.list

# 1. Tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-vcstool python3-colcon-common-extensions python3-rosdep \
    python3-pip python3-venv git wget curl lsb-release gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# 2. Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# 3. ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gzharmonic \
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
    ros-humble-diff-drive-controller \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    ros-humble-effort-controllers \
    ros-humble-realsense2-description \
    ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# 4. MuJoCo backend
# mujoco_ros2_control is vendored in src/, so this installs only what building
# it needs: the MuJoCo library itself, the CMake package both its CMakeLists
# require, the GL headers its camera plugin links against, and the Python the
# URDF-to-MJCF converter imports. ros2_control_cmake is normally pulled in by
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

# 5. Workspace directory (source is mounted from host at runtime)
ENV GZ_VERSION=harmonic
ENV PYTHONDONTWRITEBYTECODE=1
RUN mkdir -p /opt/erc_ws/src

# 6. Entrypoint
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /opt/erc_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
