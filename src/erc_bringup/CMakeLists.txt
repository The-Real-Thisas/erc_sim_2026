cmake_minimum_required(VERSION 3.8)
project(erc_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
# generate_mujoco_world.py imports mesh_collision.py from its own directory,
# so the two must be installed side by side.
install(PROGRAMS
  scripts/generate_urdf.py
  scripts/gripper_command_clamp.py
  scripts/generate_mujoco_urdf.py
  scripts/generate_mujoco_world.py
  scripts/mesh_collision.py
  scripts/odom_relay.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
