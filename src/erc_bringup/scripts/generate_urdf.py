#!/usr/bin/env python3
"""
Generate the competition URDF for the TIAGo Pro robot.

Calls PAL's xacro with the competition configuration and applies the only
patch the robot itself needs: the wheels are removed from <ros2_control> as
commanded joints and re-added as state-only. The result is written to
erc_description/urdf/tiago_pro.urdf.

This file describes the ROBOT, not the simulator. The MuJoCo-specific
retarget - hardware plugin, actuators, four-bar loop closures, camera,
gravity compensation - is applied on top of this by generate_mujoco_urdf.py
at launch, so this URDF stays the single description both the simulator and
any real-robot tooling start from.

Run inside the container after building:
    ros2 run erc_bringup generate_urdf.py

Use --help to see all available configuration options and their valid values.

Examples:
    # Default competition config (dual-arm, pro grippers)
    ros2 run erc_bringup generate_urdf.py

    # Single left arm only
    ros2 run erc_bringup generate_urdf.py --arm_type_right no-arm

    # Navigation-only testing, no arms
    ros2 run erc_bringup generate_urdf.py --arm_type_left no-arm --arm_type_right no-arm
"""

import argparse
import os
import subprocess

from ament_index_python.packages import get_package_share_directory

# The base is driven as a whole body by the simulator, not through the wheel
# joints, so nothing should command them. They stay in <ros2_control> as
# state-only entries purely so joint_state_broadcaster can publish their
# positions for RViz.
WHEEL_JOINTS = ('wheel_front_right_joint', 'wheel_front_left_joint',
                'wheel_rear_right_joint', 'wheel_rear_left_joint')
WHEEL_POSITIONS = {'wheel_front_right_joint': 'front_right',
                   'wheel_front_left_joint': 'front_left',
                   'wheel_rear_right_joint': 'rear_right',
                   'wheel_rear_left_joint': 'rear_left'}


def generate_urdf(**xacro_args):
    """Generate and patch the TIAGo Pro URDF. Returns the URDF string."""

    tiago_dir = get_package_share_directory('tiago_pro_description')
    bringup_dir = get_package_share_directory('erc_bringup')

    controller_cfg = os.path.join(
        bringup_dir, 'config', 'controller_manager_cfg.yaml')

    # ── Generate URDF via xacro ──
    xacro_path = os.path.join(tiago_dir, 'robots', 'tiago_pro.urdf.xacro')
    cmd = ['xacro', xacro_path]
    for key, val in xacro_args.items():
        cmd.append(f'{key}:={val}')
    urdf = subprocess.check_output(cmd).decode('utf-8')

    # ── Swap PAL's controller config with ours ──
    pal_cfg = os.path.join(
        tiago_dir, 'ros2_control', 'gazebo_controller_manager_cfg.yaml')
    urdf = urdf.replace(pal_cfg, controller_cfg)

    # ── Wheels: commanded -> state-only ──
    for wheel in WHEEL_JOINTS:
        position = WHEEL_POSITIONS[wheel]
        joint_block = (
            f'<joint name="{wheel}">\n'
            f'      <command_interface name="velocity"/>\n'
            f'      <state_interface name="position"/>\n'
            f'    </joint>\n'
            f'    <transmission name="wheel_{position}_trans">\n'
            f'      <plugin>transmission_interface/SimpleTransmission</plugin>\n'
            f'      <actuator name="wheel_{position}_actuator" role="actuator1"/>\n'
            f'      <joint name="{wheel}" role="joint1">\n'
            f'        <mechanical_reduction>1.0</mechanical_reduction>\n'
            f'      </joint>\n'
            f'    </transmission>\n'
        )
        if joint_block in urdf:
            urdf = urdf.replace(joint_block, '', 1)
        else:
            print(f'WARNING: could not strip {wheel} from <ros2_control> — '
                  f'block format may have changed.')

    state_only = ''
    for wheel in WHEEL_JOINTS:
        state_only += (
            f'    <joint name="{wheel}">\n'
            f'      <state_interface name="position"/>\n'
            f'      <state_interface name="velocity"/>\n'
            f'    </joint>\n'
        )
    urdf = urdf.replace('</ros2_control>', state_only + '  </ros2_control>', 1)

    return urdf


def main():
    parser = argparse.ArgumentParser(
        description='Generate the ERC 2026 TIAGo Pro URDF.')

    # ── Robot configuration ──
    parser.add_argument('--arm_type_left', default='tiago-pro-s',
                        choices=['tiago-pro-s', 'no-arm'],
                        help='Left arm type (default: tiago-pro-s)')
    parser.add_argument('--arm_type_right', default='tiago-pro-s',
                        choices=['tiago-pro-s', 'no-arm'],
                        help='Right arm type (default: tiago-pro-s)')
    parser.add_argument('--end_effector_left', default='pal-pro-gripper',
                        choices=['pal-pro-gripper', 'no-end-effector'],
                        help='Left end effector (default: pal-pro-gripper)')
    parser.add_argument('--end_effector_right', default='pal-pro-gripper',
                        choices=['pal-pro-gripper', 'no-end-effector'],
                        help='Right end effector (default: pal-pro-gripper)')
    parser.add_argument('--ft_sensor_left', default=None,
                        choices=['no-ft-sensor'],
                        help='Force torque sensor left arm [no-ft-sensor]')
    parser.add_argument('--ft_sensor_right', default=None,
                        choices=['no-ft-sensor'],
                        help='Force torque sensor right arm [no-ft-sensor]')
    parser.add_argument('--wrist_model_left', default=None,
                        help='Wrist model left arm [wrist-2010, wrist-2017]')
    parser.add_argument('--wrist_model_right', default=None,
                        help='Wrist model right arm [wrist-2010, wrist-2017]')
    parser.add_argument('--camera_model', default=None,
                        help='Head camera model [realsense-d435i, realsense-d435]')
    parser.add_argument('--laser_model', default=None,
                        help='LiDAR model [sick-571, sick-561, sick-tim, no-laser]')
    parser.add_argument('--base_type', default=None,
                        help='Base type [omni_base]')

    # ── Boolean flags ──
    parser.add_argument('--has_wrist_camera', action='store_true',
                        help='Mount a wrist camera link on the end effector. Adds '
                             'the link only - nothing renders it, so it is a '
                             'kinematic placeholder [flag, default: off]')

    args = parser.parse_args()

    xacro_args = {
        'arm_type_left': args.arm_type_left,
        'arm_type_right': args.arm_type_right,
        'end_effector_left': args.end_effector_left,
        'end_effector_right': args.end_effector_right,
        'has_teleop_arms': 'False',
        'is_public_sim': 'True',
        'use_sim_time': 'True',
        # PAL's arg name is about their plugin choice, but it also selects the
        # command-interface set: their 'classic' default adds velocity and
        # effort interfaces alongside position, and a position-controlled
        # simulator rejects both. Keep it on the position-only branch.
        'gazebo_version': 'gazebo',
    }

    optional = {
        'ft_sensor_left': args.ft_sensor_left,
        'ft_sensor_right': args.ft_sensor_right,
        'wrist_model_left': args.wrist_model_left,
        'wrist_model_right': args.wrist_model_right,
        'camera_model': args.camera_model,
        'laser_model': args.laser_model,
        'base_type': args.base_type,
    }
    for key, val in optional.items():
        if val is not None:
            xacro_args[key] = val

    if args.has_wrist_camera:
        xacro_args['has_wrist_camera'] = 'True'

    print('[generate_urdf] Configuration:')
    for key, val in xacro_args.items():
        print(f'  {key}: {val}')

    urdf = generate_urdf(**xacro_args)

    # Prefer the source tree so the URDF is visible and editable on the host.
    # With --symlink-install the share directory is a symlink to it anyway; the
    # fallback only matters for a non-symlink build, where the file lands in the
    # install space and the next colcon build would overwrite it.
    share_urdf = os.path.join(
        get_package_share_directory('erc_description'), 'urdf')
    source_urdf = os.path.realpath(os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..', '..', 'erc_description', 'urdf'))
    if os.path.isdir(os.path.dirname(source_urdf)):
        output_dir = source_urdf
    else:
        output_dir = share_urdf
        print('[generate_urdf] WARNING: writing into the install space; a '
              'rebuild will overwrite this. Build with --symlink-install.')
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'tiago_pro.urdf')

    with open(output_path, 'w') as f:
        f.write(urdf)

    print(f'[generate_urdf] Written to {os.path.realpath(output_path)}')
    print(f'[generate_urdf] URDF size: {len(urdf):,} bytes')


if __name__ == '__main__':
    main()
