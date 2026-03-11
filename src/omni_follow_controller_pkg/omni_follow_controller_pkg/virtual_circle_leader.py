#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster

from .common import wrap_pi, yaw_to_quat


class VirtualCircleLeader(Node):
    def __init__(self) -> None:
        super().__init__('virtual_circle_leader')

        self.robot_ns = self.declare_parameter('robot_ns', 'robot1').value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.child_frame_id = self.declare_parameter('child_frame_id', f'{self.robot_ns}/base_link').value
        self.center_x = float(self.declare_parameter('center_x', 0.0).value)
        self.center_y = float(self.declare_parameter('center_y', 0.0).value)
        self.radius = float(self.declare_parameter('radius', 2.0).value)
        self.angular_speed = float(self.declare_parameter('angular_speed', 0.25).value)  # rad/s
        self.start_angle = float(self.declare_parameter('start_angle', 0.0).value)
        self.update_rate_hz = float(self.declare_parameter('update_rate_hz', 50.0).value)
        self.publish_path = bool(self.declare_parameter('publish_path', True).value)
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)
        self.path_points = int(self.declare_parameter('path_points', 240).value)

        odom_topic = f'/{self.robot_ns}/sensor_odom'
        path_topic = f'/{self.robot_ns}/reference_path'

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 1)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(self.update_rate_hz, 1e-3), self.on_timer)

        if self.publish_path:
            self.publish_reference_path()

        self.get_logger().info(
            f'Publishing virtual circular leader on {odom_topic} '
            f'(R={self.radius:.3f} m, w={self.angular_speed:.3f} rad/s)'
        )

    def publish_reference_path(self) -> None:
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        poses: List[PoseStamped] = []
        for i in range(max(8, self.path_points)):
            theta = self.start_angle + 2.0 * math.pi * i / self.path_points
            x = self.center_x + self.radius * math.cos(theta)
            y = self.center_y + self.radius * math.sin(theta)
            yaw = wrap_pi(theta + math.pi / 2.0) if self.angular_speed >= 0.0 else wrap_pi(theta - math.pi / 2.0)

            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quat(yaw)
            poses.append(ps)

        path.poses = poses
        self.path_pub.publish(path)

    def on_timer(self) -> None:
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds / 1e9

        theta = self.start_angle + self.angular_speed * t
        x = self.center_x + self.radius * math.cos(theta)
        y = self.center_y + self.radius * math.sin(theta)

        if self.angular_speed >= 0.0:
            yaw = wrap_pi(theta + math.pi / 2.0)
        else:
            yaw = wrap_pi(theta - math.pi / 2.0)

        linear_speed = abs(self.radius * self.angular_speed)
        angular_z = self.angular_speed

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(yaw)

        # Body-frame twist: forward speed only for a tangent-moving virtual leader.
        odom.twist.twist.linear.x = linear_speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = VirtualCircleLeader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
