# Emirates Robotics Competition 2026

Library Assistant Robot challenge: Autonomous book retrieval using a TIAGo Pro mobile manipulator.

<img src="docs/assets/erc_3d_env.png" width="300"/> <img src="docs/assets/tiago_pro.png" width="200"/>

## Prerequisites

- x86_64 (amd64) architecture. ARM-based hosts (e.g. Apple Silicon) are not supported
- Linux host (Ubuntu 22.04/24.04 recommended) with X11
- Docker Engine + Docker Compose v2
- Git
- NVIDIA GPU + nvidia-container-toolkit for GPU-accelerated rendering        # Rendering is slow but possible without NVIDIA GPU
- ~15GB free disk space

**Note:** The default `ROS_DOMAIN_ID` is `23`. If you are running multiple ROS 2 environments on the same network, ensure there are no domain ID conflicts.

## Quick start

All dependencies are vendored in this repository — there is no separate dependency-fetching step, and no network access is needed after cloning.

```bash
# — Host terminal —
./docker/up.sh --build                        # Build image & start container
./docker/attach.sh                            # Open shell inside container

# — Inside container (after attach) —
colcon build --symlink-install                # Build workspace packages (first run takes several minutes)
source install/setup.bash                     # Source the workspace
ros2 launch erc_bringup mujoco_simulation.launch.py   # Launch MuJoCo + robot
```

The simulator runs headless by default, so it needs no display. Add
`headless:=false` to the launch command to get the MuJoCo viewer window.

To open additional terminals into the running container:
```bash
# — Host terminal —                           # Navigate to the repository root (where "docker/", "src/", and "docs/" are located)
./docker/attach.sh                            # Do NOT run ./docker/up.sh again as this will kill your running container
```

The video below is a visual guide for the above steps.

https://github.com/user-attachments/assets/05af53b4-abcf-4add-b4d2-149e7cda9941


## Robot platform

TIAGo Pro by PAL Robotics — omnidirectional mobile manipulator.

**Hardware in simulation:**
- Omnidirectional mecanum base (4 mecanum wheels, full holonomic motion)
- Two 7-DOF arms (left and right) with PAL Pro grippers
- Pan-tilt head (2 DOF)
- Prismatic torso lift
- Intel RealSense D435 RGB-D camera (head-mounted)
- Two 270° LiDARs (front and rear, base-mounted)
- IMU (base)

## ROS 2 topics and controllers

### Mobile base

The base is holonomic and is driven through its wheels. `mecanum_drive_controller` turns `/cmd_vel` into the four wheel velocities, and `mujoco_ros2_control_plugins/BaseVelocityPlugin` turns the wheels' measured rotation back into a force-limited push on the floating base, capped at what four wheels at PAL's 6 Nm effort limit could develop (315 N, 147 Nm). Publish a standard `Twist` message to move the robot in any direction.

**Contact resists the base.** Driving into a shelf saturates that force and stalls the robot rather than carrying an arm through it. The wheels keep turning against the obstruction, so `/odom` — which `mecanum_drive_controller` dead-reckons from the wheel encoders, exactly as the real robot's controller does — drifts away from the truth while the robot is stuck. **`/odom` is not ground truth.** Over a 17 s, six-leg path it is accurate to about 0.4% of distance travelled; against an obstacle it diverges without bound, as wheel odometry does. Where the `odom` frame sits in the world is for a solution's own localiser to find, as on the real robot: nothing publishes a `world` → `odom` transform. If you need the true base pose, `/simulator/floating_base_state` reports it in world coordinates.

The mecanum roller pattern is not modelled as geometry. MuJoCo has no equivalent of the `fdir1` friction direction the competition's Gazebo build uses to point each wheel's grip along its 45° roller axis, and only a capsule's anisotropic friction frame follows the geom — which a spinning wheel cannot exploit. The wheel geoms therefore carry the robot's weight and collide with the world but do not resist sliding; the mecanum traction is supplied analytically from their rotation instead.

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscriber | Base velocity (linear.x, linear.y, angular.z) |
| `/odom` | `nav_msgs/Odometry` | Publisher | Wheel odometry |

**Examples:**
```bash
# Drive forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}" --rate 10

# Strafe left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.3}, angular: {z: 0.0}}" --rate 10

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.5}}" --rate 10

# Diagonal movement (forward + left)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.3}, angular: {z: 0.0}}" --rate 10
```

### Joint controllers

All joint controllers use `joint_trajectory_controller/JointTrajectoryController` via `ros2_control`. Send trajectories using the action interface or the topic shortcut.

| Controller | Topic | Joints |
|---|---|---|
| `arm_left_controller` | `/arm_left_controller/joint_trajectory` | `arm_left_1_joint` .. `arm_left_7_joint` |
| `arm_right_controller` | `/arm_right_controller/joint_trajectory` | `arm_right_1_joint` .. `arm_right_7_joint` |
| `gripper_left_controller_raw` | `/gripper_left_controller/joint_trajectory` | `gripper_left_finger_joint` |
| `gripper_right_controller_raw` | `/gripper_right_controller/joint_trajectory` | `gripper_right_finger_joint` |
| `head_controller` | `/head_controller/joint_trajectory` | `head_1_joint`, `head_2_joint` |
| `torso_controller` | `/torso_controller/joint_trajectory` | `torso_lift_joint` |
| `imu_sensor_broadcaster` | `/base_imu` | Base IMU (read-only) |
| `joint_state_broadcaster` | `/joint_states` | All joints (read-only) |

**Note on the grippers.** Command the un-suffixed `/gripper_{left,right}_controller/joint_trajectory`, not the `_raw` topic. A guard node rejects any point outside the finger joint's usable range of `0.000` (closed) to `0.069` (open); commanding `_raw` directly bypasses it, and the gripper jams permanently if driven past its limit.

**Examples:**
```bash
# — Left arm: move joint 1 to 0.5 rad —
ros2 topic pub --once /arm_left_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [arm_left_1_joint, arm_left_2_joint, arm_left_3_joint, arm_left_4_joint, \
  arm_left_5_joint, arm_left_6_joint, arm_left_7_joint], \
  points: [{positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# — Right arm: move joint 1 to -0.5 rad —
ros2 topic pub --once /arm_right_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [arm_right_1_joint, arm_right_2_joint, arm_right_3_joint, arm_right_4_joint, \
  arm_right_5_joint, arm_right_6_joint, arm_right_7_joint], \
  points: [{positions: [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# — Left gripper: close (0.0 = closed, 0.069 = fully open) —
ros2 topic pub --once /gripper_left_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_left_finger_joint], \
  points: [{positions: [0.0], time_from_start: {sec: 1}}]}"

# — Left gripper: open —
ros2 topic pub --once /gripper_left_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_left_finger_joint], \
  points: [{positions: [0.04], time_from_start: {sec: 1}}]}"

# — Right gripper: close —
ros2 topic pub --once /gripper_right_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_right_finger_joint], \
  points: [{positions: [0.0], time_from_start: {sec: 1}}]}"

# — Right gripper: open —
ros2 topic pub --once /gripper_right_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_right_finger_joint], \
  points: [{positions: [0.04], time_from_start: {sec: 1}}]}"

# — Head: pan left 0.5 rad, tilt down 0.3 rad —
ros2 topic pub --once /head_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [head_1_joint, head_2_joint], \
  points: [{positions: [0.5, -0.3], time_from_start: {sec: 1}}]}"

# — Torso: raise to 0.3 m —
ros2 topic pub --once /torso_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [torso_lift_joint], \
  points: [{positions: [0.3], time_from_start: {sec: 2}}]}"
```

### Sensors

| Topic | Type | Description |
|---|---|---|
| `/scan_front_raw` | `sensor_msgs/msg/LaserScan` | Front LiDAR |
| `/scan_rear_raw` | `sensor_msgs/msg/LaserScan` | Rear LiDAR |
| `/head_front_camera/head_front_camera/color/image_raw` | `sensor_msgs/msg/Image` | Head RGB camera |
| `/head_front_camera/head_front_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Head RGB camera info |
| `/head_front_camera/head_front_camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Head depth camera (float32) |
| `/head_front_camera/head_front_camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Head depth camera info |
| `/head_front_camera/head_front_camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | |
| `/base_imu` | `sensor_msgs/msg/Imu` | Base IMU |
| `/spectator/color` | `sensor_msgs/msg/Image` | Fixed arena view |
| `/spectator/depth` | `sensor_msgs/msg/Image` | |
| `/spectator/camera_info` | `sensor_msgs/msg/CameraInfo` | |
| `/model_states` | `mujoco_ros2_control_msgs/msg/FreeJointStateArray` | Ground-truth pose and twist of every free body, in the world frame |
| `/contacts` | `ros_gz_interfaces/msg/Contacts` | Every contact on the robot — base, torso, head, both arms, both grippers |
| `/bin_contacts` | `ros_gz_interfaces/msg/Contacts` | Every contact on the collection bin |

**Contacts.** Both topics carry one entry per colliding pair, with the contact points, normals, penetration depths and world-frame wrenches in parallel arrays. `/bin_contacts` is the signal that a book has been placed: a book in the bin shows up as a pair against `bin_floor`. The two collision entities are named the way Gazebo named them — a robot link reports `<link>_collision`, and arena geometry reports its own name (`bin_floor`, `book_col_3_row_4_red_geom`, `erc_shelf_collision_12`). Grasp force is also readable from the `effort` field of `/joint_states`, and object placement from `/model_states`. The base's own contacts are reported, and unlike the old kinematic drive they do slow it down — see **Mobile base** above.

### Sensor specifications

| Sensor | Parameter | Value |
|---|---|---|
| **Head RGB camera** | Resolution | 640 × 360 px |
| | Horizontal FOV | 1.518 rad (87°) |
| | Vertical FOV | 0.981 rad (56.19°) |
| | Update rate | 30 Hz |
| **Head depth camera** | Resolution | 640 × 360 px |
| | Depth range | 0.2 – 8.0 m |
| | Update rate | 30 Hz |
| **Front / Rear LiDAR** | Model | SICK TIM551 |
| | FOV | ~270° |
| | Range | 0.05 – 25.0 m |
| | Samples | 818 (0.33°/step) |
| | Update rate | 10 Hz |
| **Base IMU** | Update rate | 250 Hz |
| **Contacts** | `/contacts` update rate | 30 Hz |
| | `/bin_contacts` update rate | 10 Hz |


## Software stack

| Component | Version |
|---|---|
| ROS 2 | Humble |
| Physics engine | MuJoCo |
| DDS | CycloneDDS |
| Robot description | PAL Robotics (vendored, see above) |
| Controller framework | ros2_control + mujoco_ros2_control |

## Vendored dependencies

| Package | Upstream | Branch |
|---|---|---|
| `launch_pal` | https://github.com/pal-robotics/launch_pal | `master` |
| `pal_urdf_utils` | https://github.com/pal-robotics/pal_urdf_utils | `humble-devel` |
| `pal_gripper` | https://github.com/pal-robotics/pal_gripper | `humble-devel` |
| `pal_pro_gripper` | https://github.com/pal-robotics/pal_pro_gripper | `humble-devel` |
| `tiago_pro_robot` | https://github.com/pal-robotics/tiago_pro_robot | `humble-devel` |
| `pal_sea_arm` | https://github.com/pal-robotics/pal_sea_arm | `humble-devel` |
| `tiago_pro_head_robot` | https://github.com/pal-robotics/tiago_pro_head_robot | `humble-devel` |
| `omni_base_robot` | https://github.com/pal-robotics/omni_base_robot | `humble-devel` |
| `mujoco_ros2_control` | https://github.com/pal-robotics-forks/mujoco_ros2_control | `main` (two local patches, see below) |

`mujoco_ros2_control` is vendored with two patches to `BaseVelocityPlugin`, both of which apply to the base and neither of which changes its default behaviour: `hold_pose_on_idle`, which latches the pose of an idle kinematically-driven base so articulation reaction torques cannot wander it, and `drive_mode: traction`, which is what this simulator actually runs — the base is servoed from the measured rotation of its wheels with a force cap instead of having a velocity written into it. `drive_mode` defaults to `kinematic`, so an existing configuration behaves exactly as it did upstream.

## URDF generation

The robot URDF is generated from PAL's xacro sources and saved to `src/erc_description/urdf/tiago_pro.urdf` which is accessible on the host for inspection. Because the workspace is built with `--symlink-install`, the URDF is picked up immediately when created with no rebuild required. The simulator-specific retarget — MuJoCo actuators, gripper loop closures, camera placement and gravity compensation — is applied on top of that file at launch, so the URDF stays the single description that both the simulator and any real-robot tooling start from. The head camera's optics are stated only in that URDF, in its `head_front_camera_link` sensor block, and read back out at launch, so the resolution and field of view below can be checked against the file.

## Arena

The arena is generated as MJCF at launch. `ERC_SEED` (or `seed:=`) fixes the book colours and the shelf number order; the same seed always produces the same arena. Pass `scene:=/path/to/scene.xml` to load a different MJCF scene instead.

## Reporting issues

If you notice a bug, an error in the documentation, or anything in the code that needs changing, please submit an issue on the repository rather than reporting it elsewhere. This keeps all known issues visible and traceable for both participants and organisers.

<img src="docs/assets/issue_submission.png" width="500"/>

1. In the "Title" field, type a title for your issue.
2. In the comment body field, type a description of your issue. To cross-reference a related discussion, paste the discussion's URL into the issue description.

Before opening a new issue, search existing issues to check whether it has already been reported. When describing your issue, include:

- Steps to reproduce the problem
- Expected vs. actual behaviour
- Relevant terminal output or logs
- Your host environment (OS, GPU, `ROS_DOMAIN_ID` if changed from default)

## Troubleshooting

**CycloneDDS serialization warnings** (`serdata.cpp` errors about null-terminated strings) — these are harmless noise from large message serialization. They don't affect functionality.

**Nothing on any topic for the first few seconds** — the robot description is converted to MJCF before the simulator starts, which takes a few seconds. Controllers are spawned on a 5-second timer.

**Controllers not activating** — check that `erc_bringup` was built: `colcon build --symlink-install --packages-select erc_bringup && source install/setup.bash`

**Base not strafing (lateral movement)** — ensure you're publishing to `/cmd_vel` with `linear.y` set, and check that `mecanum_drive_controller` is active (`ros2 control list_controllers`). Nothing moves the base if that controller is not running: the wheels stay still and the base servo has nothing to follow.

**Build errors after mixing build flags** — always build with `--symlink-install`. Mixing symlink and non-symlink builds leaves stale artifacts; recover with `rm -rf build/ install/ log/` and rebuild.

**Rebuild after Dockerfile changes** — run `./docker/up.sh --build` to rebuild the image.

**DDS discovery storms on a busy network** — the container leaves `ROS_LOCALHOST_ONLY` unset so a second machine or a host-side RViz can see the topics. On a crowded LAN this can stall the simulation; export `ROS_LOCALHOST_ONLY=1` before launching to confine DDS to loopback.
