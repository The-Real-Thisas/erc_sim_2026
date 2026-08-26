#!/usr/bin/env python3
"""
Generate the MuJoCo-flavoured URDF for the ERC TIAGo Pro dev simulator.

Starts from the competition's pregenerated URDF (erc_description/urdf/
tiago_pro.urdf) so the robot the dev simulator runs is byte-derived from the
one the competition ships: same links, masses, joint limits, meshes and
ros2_control joint set. Applies only what MuJoCo needs:

  1. Strip every <gazebo> element PAL's xacro emits (plugins, sensors,
     friction tags) - MuJoCo reads none of them - along with the transmissions
     and the two laser housings (see strip_laser_housings).
  2. Swap the hardware plugin PAL's xacro emits for
     mujoco_ros2_control/MujocoSystemInterface reading the MJCF from a topic.
  3. Embed a <mujoco_inputs> block for the URDF->MJCF converter:
       - one MuJoCo <position> actuator per commanded joint, gains taken from
         PAL's own tiago_pro_mujoco pids.yaml (p -> kp, u_clamp -> forcerange)
       - <equality> joint couplings replacing the URDF <mimic> four-bar
         constraints that MJCF conversion drops
       - the head camera as a MuJoCo RGB-D camera on the optical frame
         (matched to the competition's D435 retarget: 640x360, 56 deg vfov)
The camera needs no frame of its own: the converter treats the named site as a
REP-103 optical frame and applies the optical->MuJoCo rotation itself.

Run inside the container after building:
    ros2 run erc_bringup generate_mujoco_urdf.py -o /tmp/erc_mujoco.urdf
"""

import argparse
import math
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

# PAL's xacro emits one of several simulator hardware plugins depending on its
# own arguments, so match whichever one landed rather than a fixed name.
HARDWARE_PLUGIN_RE = re.compile(r'<plugin>[^<]*(?:GazeboSystem|GazeboSimSystem|'
                                r'MujocoSystem\w*)</plugin>')

# The two base SICK scanners, matched to what the robot description declares:
# 818 samples across 270 degrees at 10 Hz, 0.05-25 m. The lidar extension casts
# rays in the site's own frame - x is azimuth zero, z is the scan normal - and
# the converter emits one site per URDF link, so each scanner rides its own
# link frame.
LASERS = (
    ('scan_front_raw', 'base_front_laser_link'),
    ('scan_rear_raw', 'base_rear_laser_link'),
)
# These are hand-copied from the <gazebo><sensor type="gpu_lidar"> blocks that
# strip_gazebo_blocks deletes below, so nothing can cross-check them at runtime.
# They match the shipped description char-for-char today; if PAL changes the
# scanner spec, this is where it has to be changed too.
LASER_SAMPLES = 818
LASER_MIN_ANGLE = -2.3387411976724017
LASER_MAX_ANGLE = 2.356194490192345
LASER_MIN_RANGE = 0.05
LASER_MAX_RANGE = 25.0
LASER_RATE = 10.0

# The base IMU. mujoco_ros2_control builds a ros2_control IMU sensor out of
# three MJCF sensors whose names share a base and take these suffixes, and
# imu_sensor_broadcaster turns that into a sensor_msgs/Imu.
IMU_SENSOR = 'base_imu_sensor'
IMU_SITE = 'base_imu_link'

# The head camera, matched to an Intel RealSense D435 depth mode: a native 16:9
# depth resolution at the datasheet's 87 deg horizontal field of view. MuJoCo
# specifies a camera by its VERTICAL fov, which follows from these.
#
# This is the ONLY definition of the head camera's optics. The robot
# description carries PAL's stock camera too, but that lives inside <gazebo>
# sensor blocks which are stripped below and which nothing reads, so passing
# --camera_model to generate_urdf.py will NOT change what the simulated camera
# does. Change it here.
CAM_W, CAM_H = 640, 360
CAM_HFOV_RAD = 1.5184364492350666        # 87 deg


def strip_gazebo_blocks(urdf: str) -> str:
    n = urdf.count('<gazebo')
    urdf = re.sub(r'[ \t]*<gazebo( [^>]*)?>.*?</gazebo>\n?', '', urdf, flags=re.S)
    if '<gazebo' in urdf:
        sys.exit('ERROR: gazebo blocks survived stripping')
    print(f'stripped {n} gazebo blocks')
    return urdf


def strip_laser_housings(urdf: str) -> str:
    """Remove the scanners' own housing geometry.

    The lidar extension casts its rays with no self-exclusion, and mj_multiRay
    filters by geom group and the static flag - not by contype/conaffinity - so
    a housing left in the model is hit by its own scanner whatever its collision
    flags say. Measured: every one of the 818 rays returned the housing mesh at
    20.6 mm. Both the visual and the collision element have to go, because the
    converter emits a collision geom from the visual mesh as well.

    The housings are 8 cm nubs sitting inside the base's own collision envelope,
    so the robot's contact behaviour is unchanged; they simply stop being drawn.
    """
    n = 0
    for _name, site in LASERS:
        m = re.search(r'(<link name="' + re.escape(site) + r'"[^>]*>)(.*?)(</link>)',
                      urdf, re.S)
        if not m:
            # A legitimate configuration: generate_urdf.py --laser_model
            # no-laser produces a robot with no scanners at all, and the LASERS
            # loop below skips them for the same reason.
            print(f'note: no <link name="{site}">; this robot has no scanner there')
            continue
        body, count = re.subn(r'[ \t]*<(visual|collision)(?:\s[^>]*)?>.*?</\1>\n?',
                              '', m.group(2), flags=re.S)
        # Half-stripping is worse than not stripping: whatever is left is
        # opaque to the scanner's own rays and every one of the 818 samples
        # comes back as the housing, which looks like a plausible scan.
        if count != 2:
            sys.exit(f'ERROR: expected one <visual> and one <collision> in '
                     f'{site}, removed {count}. Refusing to ship a scanner '
                     f'that would read its own housing.')
        urdf = urdf[:m.start()] + m.group(1) + body + m.group(3) + urdf[m.end():]
        n += count
    print(f'stripped {n} laser housing visual/collision elements')
    return urdf


def strip_transmissions(urdf: str) -> str:
    # mujoco_ros2_control demands a matching MuJoCo actuator for every
    # transmission actuator, so leaving these in would mean inventing actuators
    # that drive nothing. Every joint that is actually commanded gets a
    # <position> actuator on the joint itself instead, which bypasses the
    # transmission - including the gripper, whose PalGripperTransmission is the
    # one block here with a reduction other than 1.0.
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


def head_camera_fovy() -> float:
    """Vertical FOV in degrees, from the horizontal FOV and the aspect ratio."""
    return math.degrees(
        2 * math.atan(math.tan(CAM_HFOV_RAD / 2) * CAM_H / CAM_W))


def build_mujoco_inputs(urdf: str, spawn_xyz: str, spawn_yaw: str,
                        fovy: float) -> str:
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

    lidar_instances, sensors = [], []
    for name, site in LASERS:
        if f'name="{site}"' not in urdf:
            print(f'WARNING: {site} not in URDF, skipping {name}')
            continue
        lidar_instances.append(
            f'          <instance name="{name}">\n'
            f'            <config key="resolution" value="{LASER_SAMPLES} 1"/>\n'
            f'            <config key="azimuth_range" '
            f'value="{LASER_MIN_ANGLE:.10g} {LASER_MAX_ANGLE:.10g}"/>\n'
            f'            <config key="elevation_range" value="0.0"/>\n'
            f'            <config key="min_range" value="{LASER_MIN_RANGE:g}"/>\n'
            f'            <config key="max_range" value="{LASER_MAX_RANGE:g}"/>\n'
            f'            <config key="update_rate" value="{LASER_RATE:g}"/>\n'
            f'            <config key="async" value="0"/>\n'
            f'          </instance>')
        sensors.append(
            f'        <plugin name="{name}" instance="{name}" objtype="site" '
            f'objname="{site}"/>')
    if f'name="{IMU_SITE}"' in urdf:
        sensors.append(
            f'        <framequat name="{IMU_SENSOR}_quat" objtype="site" '
            f'objname="{IMU_SITE}"/>')
        sensors.append(f'        <gyro name="{IMU_SENSOR}_gyro" site="{IMU_SITE}"/>')
        sensors.append(
            f'        <accelerometer name="{IMU_SENSOR}_accel" site="{IMU_SITE}"/>')

    # An <extension> naming mujoco.plugin.lidar makes the engine plugin a hard
    # requirement of the model: if it is not loaded, compilation fails outright
    # with "plugin mujoco.plugin.lidar not found" and the whole simulator dies.
    # So emit it only when a scanner actually made it in.
    lidar_extension = ''
    if lidar_instances:
        lidar_extension = ('      <extension>\n'
                           '        <plugin plugin="mujoco.plugin.lidar">\n'
                           + chr(10).join(lidar_instances) + '\n'
                           '        </plugin>\n'
                           '      </extension>\n')

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
               arms' zero pose otherwise presses into the chassis and gets
               pushed out to a dangling pose that fouls the table. -->
          <geom group="3" type="mesh" contype="1" conaffinity="2"/>
        </default>
      </default>
      <actuator>
{chr(10).join(actuators)}
      </actuator>
{lidar_extension}      <sensor>
{chr(10).join(sensors)}
      </sensor>
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
              fovy="{fovy:.6g}" mode="fixed" resolution="{CAM_W} {CAM_H}"/>
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
           pos/quat are the floating base's initial qpos; the arena spec starts
           the robot yawed 90 degrees on the start zone. Do NOT lift it off the
           floor: the base plugin latches the pose whenever cmd_vel is stale,
           so a robot spawned in the air would simply stay there. -->
      <modify_element type="body" name="{root_body}" pos="{spawn_xyz}" euler="0 0 {spawn_yaw}"/>
    </processed_inputs>
  </mujoco_inputs>
'''


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

    fovy = head_camera_fovy()

    urdf = strip_gazebo_blocks(urdf)
    urdf = strip_transmissions(urdf)
    urdf = strip_laser_housings(urdf)

    hardware = f'''<plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model_topic">/mujoco_robot_description</param>
      <param name="headless">{args.headless}</param>
      <param name="camera_publish_rate">{args.camera_rate}</param>
      <param name="sim_speed_factor">{args.sim_speed_factor}</param>'''
    if not HARDWARE_PLUGIN_RE.search(urdf):
        sys.exit('ERROR: no simulator hardware plugin found in <ros2_control>, '
                 'so there is nothing to swap for the MuJoCo one')
    urdf = HARDWARE_PLUGIN_RE.sub(lambda _m: hardware, urdf, count=1)

    imu_iface = ''
    if f'name="{IMU_SITE}"' in urdf:
        interfaces = ''.join(
            f'      <state_interface name="{n}"/>\n' for n in (
                'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w',
                'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z',
                'linear_acceleration.x', 'linear_acceleration.y',
                'linear_acceleration.z'))
        imu_iface = (f'    <sensor name="{IMU_SENSOR}">\n'
                     f'      <param name="mujoco_type">imu</param>\n'
                     f'{interfaces}'
                     f'    </sensor>\n')
        urdf = urdf.replace('</ros2_control>', imu_iface + '  </ros2_control>', 1)

    inputs = build_mujoco_inputs(urdf, args.spawn_xyz, args.spawn_yaw, fovy)
    urdf = urdf.replace('</robot>', inputs + '</robot>', 1)

    with open(args.output, 'w') as f:
        f.write(urdf)
    print(f'wrote {args.output}')


if __name__ == '__main__':
    main()
