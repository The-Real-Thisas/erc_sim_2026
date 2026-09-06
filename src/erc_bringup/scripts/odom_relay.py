#!/usr/bin/env python3
"""
Publish the competition's odometry interface: /odom + TF odom -> base_footprint,
plus the static world -> odom transform.

The odometry itself is NOT ground truth. mecanum_drive_controller integrates it
from the four wheel encoders, exactly as gz-sim-mecanum-drive-system does in the
competition's Gazebo build and as the real robot's own controller does, so it
slips and drifts whenever the wheels turn further than the base actually moved -
driving into an obstacle being the obvious case. This node only republishes that
odometry under the competition's topic and frame names and broadcasts its
transform; it must never be "corrected" against /simulator/floating_base_state,
which is the ground truth a solution is not supposed to have.

Wheel odometry starts at identity wherever the robot was spawned, so the odom
frame is NOT the world frame - and /model_states reports object poses in world
coordinates. This node therefore also publishes a static world -> odom transform,
latched from the spawn pose on /simulator/floating_base_state. Convert through TF
rather than assuming the two frames coincide.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


WHEEL_ODOM_TOPIC = '/mecanum_drive_controller/odometry'

# The competition's MecanumDrive plugin publishes /odom and its transform at
# odom_publish_frequency 50; the controller here integrates at the control
# rate, 250 Hz, so the relay keeps every fifth. At several times real time the
# extra four were 1500 transforms a second on /tf, drowning the 20 Hz joint
# transforms in every listener's queue (measured 2026-09-06).
ODOM_RATE = 50.0


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.origin = None  # (x, y, yaw) of the odom frame in world
        self.last_odom = None  # stamp of the last odometry relayed
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(
            Odometry, WHEEL_ODOM_TOPIC, self.cb, 10)
        # Ground truth, used for one thing only: to find out where the robot was
        # spawned so the world -> odom transform can be published. The
        # subscription is dropped as soon as that is known.
        self.spawn_sub = self.create_subscription(
            Odometry, '/simulator/floating_base_state', self.spawn_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def spawn_cb(self, msg: Odometry):
        if self.origin is not None:
            return
        p = msg.pose.pose.position
        self.origin = (p.x, p.y, quat_to_yaw(msg.pose.pose.orientation))
        self.publish_world_to_odom(msg.header.stamp)
        self.destroy_subscription(self.spawn_sub)
        self.spawn_sub = None

    def cb(self, msg: Odometry):
        """Republish the wheel odometry under the competition's names."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_odom is not None and stamp - self.last_odom < 1.0 / ODOM_RATE - 1e-6 \
                and stamp >= self.last_odom:
            return
        self.last_odom = stamp
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)


    def publish_world_to_odom(self, stamp):
        """Where the odom frame sits in the world, latched once at startup."""
        ox, oy, oyaw = self.origin
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.translation.x = ox
        t.transform.translation.y = oy
        t.transform.rotation.z = math.sin(oyaw / 2.0)
        t.transform.rotation.w = math.cos(oyaw / 2.0)
        self.static_tf.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomRelay())


if __name__ == '__main__':
    main()
