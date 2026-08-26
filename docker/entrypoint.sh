#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# The workspace overlay, once it has been built.
if [ -f /opt/erc_ws/install/setup.bash ]; then
    source /opt/erc_ws/install/setup.bash
fi

# Gazebo mesh paths
export GZ_SIM_RESOURCE_PATH="\
/opt/erc_ws/install/erc_description/share:\
/opt/erc_ws/install/omni_base_description/share:\
/opt/erc_ws/install/tiago_pro_description/share:\
/opt/erc_ws/install/pal_sea_arm_description/share:\
/opt/erc_ws/install/tiago_pro_head_description/share:\
/opt/erc_ws/install/pal_pro_gripper_description/share:\
/opt/erc_ws/install/pal_gripper_description/share:\
/opt/erc_ws/install/pal_urdf_utils/share:\
/opt/ros/humble/share:\
${GZ_SIM_RESOURCE_PATH}"

# gz_ros2_control plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH="\
/opt/erc_ws/install/gz_ros2_control/lib:\
${GZ_SIM_SYSTEM_PLUGIN_PATH}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONDONTWRITEBYTECODE=1

# Defaults only. Setting ROS_LOCALHOST_ONLY=1 keeps DDS off the LAN, which
# matters on a busy network where the participant count alone can stall the
# simulation - but it also breaks a second machine, a host-side RViz outside
# this container's netns, and a real-robot bridge, so it stays opt-in and any
# value the user already exported wins.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"

exec "$@"
