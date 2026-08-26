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

# Keep DDS on the loopback interface. Without this every ROS 2 process on the
# LAN joins the discovery mesh, and the participant count alone is enough to
# stall the simulation on a busy network.
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

exec "$@"
