#!/usr/bin/env python3
"""
Generate the MuJoCo-flavoured URDF for the ERC TIAGo Pro dev simulator.

Starts from the competition's pregenerated URDF (erc_description/urdf/
tiago_pro.urdf) so the robot the dev simulator runs is byte-derived from the
one the competition ships: same links, masses, joint limits, meshes and
ros2_control joint set. Applies only what MuJoCo needs:

  1. Strip every <gazebo> element (plugins, sensors, friction tags).
  2. Swap the gz_ros2_control hardware plugin for
     mujoco_ros2_control/MujocoSystemInterface reading the MJCF from a topic.
  3. Embed a <mujoco_inputs> block for the URDF->MJCF converter:
       - one MuJoCo <position> actuator per commanded joint, gains taken from
         PAL's own tiago_pro_mujoco pids.yaml (p -> kp, u_clamp -> forcerange)
       - <equality> joint couplings replacing the URDF <mimic> four-bar
         constraints that MJCF conversion drops
       - the head camera as a MuJoCo RGB-D camera on the optical frame
         (matched to the competition's D435 retarget: 640x360, 56 deg vfov)
  4. Add the MuJoCo camera frame (optical frame rotated pi about X, MuJoCo
     cameras look along -Z).

Run inside the mujoco container after building:
    ros2 run tiger_mujoco generate_mujoco_urdf.py -o /tmp/tiger_mujoco.urdf
"""

import argparse
import re
import sys

from ament_index_python.packages import get_package_share_directory

# PAL's own MuJoCo gains for the TIAGo Pro. kp and the force clamp come from
# tiago_pro_simulation (humble-devel, 1.19.0) tiago_pro_mujoco/config/pids.yaml
# (p -> kp, u_clamp -> forcerange); kv is PAL's own MuJoCo damping ladder from
# pal_sea_arm_description/mujoco/mj_tags.xacro, which is tuned per arm joint
# rather than derived. kv=None means PAL ships no damping for that joint, and
# the actuator falls back to critical damping.
#                                 kp        clamp    kv
PAL_GAINS = {
    'torso_lift_joint':           (10000.0, 2000.0, None),
    'head_1_joint':               (2000.0,  5.197,  None),
    'head_2_joint':               (2000.0,  2.77,   None),
    'arm_left_1_joint':           (1000.0,  43.0,   50.0),
    'arm_left_2_joint':           (1000.0,  43.0,   60.0),
    'arm_left_3_joint':           (1000.0,  26.0,   50.0),
    'arm_left_4_joint':           (500.0,   26.0,   20.0),
    'arm_left_5_joint':           (1000.0,  26.0,   20.0),
    'arm_left_6_joint':           (1000.0,  26.0,   20.0),
    'arm_left_7_joint':           (500.0,   26.0,   20.0),
    'arm_right_1_joint':          (1000.0,  43.0,   50.0),
    'arm_right_2_joint':          (1000.0,  43.0,   60.0),
    'arm_right_3_joint':          (1000.0,  26.0,   50.0),
    'arm_right_4_joint':          (500.0,   26.0,   20.0),
    'arm_right_5_joint':          (1000.0,  26.0,   20.0),
    'arm_right_6_joint':          (1000.0,  26.0,   20.0),
    'arm_right_7_joint':          (500.0,   26.0,   20.0),
    # Grippers: kp raised from PAL's 100 so the position servo can develop
    # the full 8 N force limit within the screw's travel; the clamp itself
    # stays PAL's number, so max pinch is unchanged in principle and the
    # grip force is decided by the force-targeted close in the pick script.
    'gripper_left_finger_joint':  (300.0,   8.0,    None),
    'gripper_right_finger_joint': (300.0,   8.0,    None),
}

# The gripper is a four-bar linkage: the fingertip is pinned to BOTH the inner
# finger (a URDF joint) and the outer finger (a loop the URDF cannot express,
# so it fakes it with a <mimic> ratio). MuJoCo can close the loop for real, so
# the fingertip's <connect> replaces its mimic equality rather than joining it:
# pinch force then reacts through the outer finger as it does on the hardware,
# instead of being carried entirely by a scripted joint ratio.
# Anchors are in the fingertip body frame, from PAL's own call site
# (pal_sea_arm_description/robots/pal_sea_arm.urdf.xacro).
FOUR_BAR_ANCHOR = {'left': '0.014 -0.001 0', 'right': '0.014 -0.0015 0'}

GZ_PLUGIN = '<plugin>gz_ros2_control/GazeboSimSystem</plugin>'


def strip_gazebo_blocks(urdf: str) -> str:
    n = urdf.count('<gazebo')
    urdf = re.sub(r'[ \t]*<gazebo( [^>]*)?>.*?</gazebo>\n?', '', urdf, flags=re.S)
    if '<gazebo' in urdf:
        sys.exit('ERROR: gazebo blocks survived stripping')
    print(f'stripped {n} gazebo blocks')
    return urdf


def strip_transmissions(urdf: str) -> str:
    # gz_ros2_control never implements <transmission> (all reductions are 1.0
    # anyway), but mujoco_ros2_control demands a matching MuJoCo actuator per
    # transmission actuator. Stripping them reproduces the Gazebo behaviour.
    n = urdf.count('<transmission')
    urdf = re.sub(r'[ \t]*<transmission( [^>]*)?>.*?</transmission>\n?', '',
                  urdf, flags=re.S)
    print(f'stripped {n} transmissions')
    return urdf


def joint_limits(urdf: str):
    """name -> (lower, upper) for every joint with a <limit>."""
    lims = {}
    for m in re.finditer(r'<joint name="([^"]+)" type="[^"]+">(.*?)</joint>', urdf, re.S):
        lm = re.search(r'<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)"', m.group(2))
        if lm:
            lims[m.group(1)] = (float(lm.group(1)), float(lm.group(2)))
    return lims


def mimic_table(urdf: str):
    """(mimic_joint, driven_joint, multiplier) parsed from ros2_control params."""
    block = urdf[urdf.index('<ros2_control'):urdf.index('</ros2_control>')]
    out = []
    for m in re.finditer(r'<joint name="([^"]+)">(.*?)</joint>', block, re.S):
        mm = re.search(r'<param name="mimic">([^<]+)</param>', m.group(2))
        mult = re.search(r'<param name="multiplier">([^<]+)</param>', m.group(2))
        if mm:
            out.append((m.group(1), mm.group(1).strip(),
                        float(mult.group(1)) if mult else 1.0))
    return out


def root_link(urdf: str) -> str:
    """The URDF link that is never a joint's child - the MJCF's root body."""
    links = set(re.findall(r'<link name="([^"]+)"', urdf))
    children = set(re.findall(r'<child link="([^"]+)"', urdf))
    roots = links - children
    if len(roots) != 1:
        sys.exit(f'expected exactly one root link, found {sorted(roots)}')
    return roots.pop()


def build_mujoco_inputs(urdf: str, spawn_xyz: str, spawn_yaw: str) -> str:
    lims = joint_limits(urdf)
    root_body = root_link(urdf)
    actuators = []
    for joint, (kp, clamp, kv) in PAL_GAINS.items():
        if joint not in lims:
            print(f'WARNING: {joint} has no limits in URDF, skipping actuator')
            continue
        lo, hi = lims[joint]
        damping = f'kv="{kv:g}"' if kv is not None else 'dampratio="1.0"'
        actuators.append(
            f'        <position name="{joint}" joint="{joint}" kp="{kp:g}" '
            f'{damping} ctrlrange="{lo:.6g} {hi:.6g}" '
            f'forcerange="{-clamp:g} {clamp:g}"/>')

    # PAL's real arm controllers apply gravity feedforward in firmware, so the
    # PID clamps (26/43 Nm) are headroom on top of gravity, not inclusive of
    # it. Mirror that with MuJoCo gravity compensation on every upper-body
    # link; contact/grasp forces stay real and a carried object is NOT
    # compensated (its weight loads the arm as on the real robot).
    gravcomp = []
    for m in re.finditer(r'<link name="((?:torso_lift|head_[12]|arm_(?:left|right)_\w*|gripper_(?:left|right)\w*)_link)"', urdf):
        gravcomp.append(
            f'        <modify_element type="body" name="{m.group(1)}" gravcomp="1.0"/>')
    gravcomp = sorted(set(gravcomp))

    equalities = []
    superseded = 0
    for mimic, driven, mult in mimic_table(urdf):
        # Both halves of the four-bar go, not just the fingertip: with the
        # outer finger still slaved to the screw by a ratio the loop is
        # over-determined, and the solver back-drives the screw (measured
        # 4.5 mm of tracking error and 5.4% jaw-gap slope error). Dropping
        # both and letting <connect> close the loop reproduces the measured
        # jaw-gap law exactly.
        if re.match(r'gripper_(left|right)_(fingertip|outer_finger)_(left|right)_joint$',
                    mimic):
            superseded += 1
            continue
        equalities.append(
            f'        <joint joint1="{mimic}" joint2="{driven}" '
            f'polycoef="0 {mult:g} 0 0 0" '
            f'solimp="0.95 0.99 0.001" solref="0.005 1"/>')

    connects, excludes = [], []
    for grip in ('left', 'right'):
        for finger in ('left', 'right'):
            tip = f'gripper_{grip}_fingertip_{finger}_link'
            outer = f'gripper_{grip}_outer_finger_{finger}_link'
            inner = f'gripper_{grip}_inner_finger_{finger}_link'
            if f'name="{tip}"' not in urdf:
                continue
            connects.append(
                f'        <connect name="four_bar_{grip}_{finger}" '
                f'body1="{tip}" body2="{outer}" active="true" '
                f'anchor="{FOUR_BAR_ANCHOR[finger]}" '
                f'solimp="0.95 0.99 0.001" solref="0.005 1"/>')
            # The links of a closed loop overlap at their shared pivot, so
            # they must not also collide there.
            excludes.append(f'        <exclude body1="{inner}" body2="{outer}"/>')
            excludes.append(f'        <exclude body1="{tip}" body2="{outer}"/>')
    equalities.extend(connects)
    print(f'{len(actuators)} actuators, {len(equalities)} equalities '
          f'({len(connects)} four-bar loop closures replacing {superseded} '
          f'mimics), {len(excludes)} contact excludes')

    return f'''  <mujoco_inputs>
    <raw_inputs>
      <!-- multiccd: without it every convex mesh pair yields a single
           contact point - each fingertip touched the book at exactly one
           point (free pivot in the grasp), and the flat book tunnelled
           10 mm into the bin's thin coacd floor sheets (measured
           2026-08-26). -->
      <option integrator="implicitfast" cone="elliptic" impratio="10">
        <flag multiccd="enable"/>
      </option>
      <!-- znear is a fraction of the scene's statistic extent (~2 m), so 0.1
           puts the near plane at ~0.2 m: the D435's minimum range, and far
           enough that the camera does not render its own housing. -->
      <visual>
        <map znear="0.1" zfar="30"/>
      </visual>
      <default>
        <default class="visual">
          <geom group="2" type="mesh" contype="0" conaffinity="0"/>
        </default>
        <default class="collision">
          <!-- contype 1 / conaffinity 2 disables robot self-collision while
               keeping robot-vs-world contact (world geoms are 1/1): the
               competition Gazebo model runs with self-collision off, and with
               it on the arms' zero pose presses into the chassis and gets
               pushed out to a dangling pose that fouls the table. -->
          <geom group="3" type="mesh" contype="1" conaffinity="2"/>
        </default>
      </default>
      <actuator>
{chr(10).join(actuators)}
      </actuator>
      <equality>
{chr(10).join(equalities)}
      </equality>
      <contact>
{chr(10).join(excludes)}
      </contact>
    </raw_inputs>
    <processed_inputs>
      <!-- The converter treats the site as a REP-103 optical frame and applies
           the optical->MuJoCo rotation itself. -->
      <camera site="head_front_camera_color_optical_frame" name="head_front_camera"
              fovy="56" mode="fixed" resolution="640 360"/>
      <!-- The base is driven kinematically (BaseVelocityPlugin writes the
           freejoint's planar qvel), so wheel-ground friction only fights
           strafe/rotation. Near-zero friction mirrors the competition's own
           mu2=0 lateral-slip patch. Geoms are addressed by (mesh, class). -->
      <!-- priority=1 makes the wheel's friction win outright over the floor's
           (default combination is element-wise max, so lowering only the
           wheel would do nothing). -->
      <modify_element type="geom" mesh="wheel_link" class="collision" friction="0.05 0.001 0.0001" priority="1"/>
      <modify_element type="geom" mesh="wheel_link_reflected" class="collision" friction="0.05 0.001 0.0001" priority="1"/>
      <!-- Rubber pad on paper ~1.2; torsional/rolling let the pinch resist
           the book pivoting about the grasp axis (active because the book
           is condim 6 and pair condim/friction take the max). -->
      <modify_element type="geom" mesh="fingertip" class="collision" friction="1.2 0.015 0.002"/>
{chr(10).join(gravcomp)}
      <!-- Where the robot starts. The root body carries the free joint, so its
           pos/quat are the floating base's initial qpos; erc_world.sdf spawns
           tiago_pro yawed 90 degrees at the start zone. Do NOT lift it off the
           floor the way the Gazebo spawn does: the base plugin latches the pose
           whenever cmd_vel is stale, so a robot spawned in the air stays there. -->
      <modify_element type="body" name="{root_body}" pos="{spawn_xyz}" euler="0 0 {spawn_yaw}"/>
    </processed_inputs>
  </mujoco_inputs>
'''


CAMERA_FRAME = ''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--output', required=True)
    ap.add_argument('--headless', default='true', choices=['true', 'false'])
    ap.add_argument('--camera-rate', default='15.0')
    ap.add_argument('--sim-speed-factor', default='-1.0',
                    help='-1 follows the viewer slowdown setting')
    ap.add_argument('--spawn-xyz', default='0 0 0',
                    help='floating base start position, MJCF order')
    ap.add_argument('--spawn-yaw', default='0',
                    help='floating base start yaw in radians')
    args = ap.parse_args()

    src = get_package_share_directory('erc_description') + '/urdf/tiago_pro.urdf'
    urdf = open(src).read()

    urdf = strip_gazebo_blocks(urdf)
    urdf = strip_transmissions(urdf)

    hardware = f'''<plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model_topic">/mujoco_robot_description</param>
      <param name="headless">{args.headless}</param>
      <param name="camera_publish_rate">{args.camera_rate}</param>
      <param name="sim_speed_factor">{args.sim_speed_factor}</param>'''
    if GZ_PLUGIN not in urdf:
        sys.exit('ERROR: gz_ros2_control plugin block not found')
    urdf = urdf.replace(GZ_PLUGIN, hardware, 1)

    inputs = build_mujoco_inputs(urdf, args.spawn_xyz, args.spawn_yaw)
    urdf = urdf.replace('</robot>', CAMERA_FRAME + inputs + '</robot>', 1)

    with open(args.output, 'w') as f:
        f.write(urdf)
    print(f'wrote {args.output}')


if __name__ == '__main__':
    main()
