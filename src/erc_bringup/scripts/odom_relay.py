#!/usr/bin/env python3
"""
Relay MuJoCo's floating-base odometry onto the competition interface:
/simulator/floating_base_state (world frame) -> /odom + TF odom->base_footprint.

Like the Gazebo MecanumDrive plugin, odom is pinned to where the robot was
when this node started, not to the world origin: the first received pose
defines the odom frame, so /odom starts at identity however the robot was
spawned. Consumers that want world coordinates should go through TF rather
than assuming odom and world coincide.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.origin = None  # (x, y, yaw) of the odom frame in world
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(
            Odometry, '/simulator/floating_base_state', self.cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q)
        if self.origin is None:
            # Latch the spawn pose so /odom starts at identity, matching what
            # the Gazebo MecanumDrive plugin reports. The competition spawns
            # the robot yawed 90 degrees, so pinning at (0,0,0) instead would
            # make /odom disagree with the Gazebo backend by that 90 degrees
            # from the very first message.
            self.origin = (p.x, p.y, yaw)
        ox, oy, oyaw = self.origin
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = p.x - ox, p.y - oy
        x, y = c * dx - s * dy, s * dx + c * dy
        dyaw = yaw - oyaw

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.z = math.sin(dyaw / 2.0)
        out.pose.pose.orientation.w = math.cos(dyaw / 2.0)
        # nav_msgs/Odometry defines twist in the CHILD frame, and that is what
        # the Gazebo plugin publishes. MuJoCo reports the free joint's raw
        # qvel, which is world-frame, so rotate the linear part into the base
        # frame; the yaw rate is the same in both.
        out.twist = msg.twist
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        cy, sy = math.cos(-yaw), math.sin(-yaw)
        out.twist.twist.linear.x = cy * vx - sy * vy
        out.twist.twist.linear.y = sy * vx + cy * vy
        self.pub.publish(out)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = out.pose.pose.orientation.z
        t.transform.rotation.w = out.pose.pose.orientation.w
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomRelay())


if __name__ == '__main__':
    main()
