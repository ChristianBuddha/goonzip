#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .common import clamp, wrap_pi, yaw_from_quat


class FormationFollowerController(Node):
    """
    Relative-state controller for one follower in a real-leader formation.

    The control law is the same as omni_follow_controller_pkg, but the
    default inputs are filtered odometry topics from real robots:

      leader   -> /<leader_ns>/odometry/filtered
      follower -> /<follower_ns>/odometry/filtered
      command  -> /<follower_ns>/cmd_vel
    """

    def __init__(self) -> None:
        super().__init__('formation_follower_controller')

        self.leader_ns = str(self.declare_parameter('leader_ns', 'robot1').value)
        self.follower_ns = str(self.declare_parameter('follower_ns', 'robot2').value)
        self.odom_topic_name = str(self.declare_parameter('odom_topic_name', 'odometry/filtered').value)
        self.cmd_topic_name = str(self.declare_parameter('cmd_topic_name', 'cmd_vel').value)

        self.l_star = float(self.declare_parameter('l_star', 0.5).value)
        self.d_star = float(self.declare_parameter('d_star', math.pi / 2.0).value)
        self.gammaL_star = float(self.declare_parameter('gammaL_star', self.d_star).value)

        self.k1 = float(self.declare_parameter('k1', 1.0).value)
        self.k2 = float(self.declare_parameter('k2', 1.0).value)
        self.k_psi = float(self.declare_parameter('k_psi', 1.0).value)
        self.psi_offset = float(self.declare_parameter('psi_offset', 0.0).value)
        self.align_follower_yaw = bool(self.declare_parameter('align_follower_yaw', True).value)
        self.l_min = float(self.declare_parameter('l_min', 0.25).value)
        self.p1_max = float(self.declare_parameter('p1_max', 1.0).value)
        self.p2_max = float(self.declare_parameter('p2_max', 2.0).value)
        self.vx_max = float(self.declare_parameter('vx_max', 0.2).value)
        self.vy_max = float(self.declare_parameter('vy_max', 0.2).value)
        self.wz_max = float(self.declare_parameter('wz_max', 0.4).value)
        self.control_rate_hz = float(self.declare_parameter('control_rate_hz', 50.0).value)
        self.transform_leader_to_follower_frame = bool(
            self.declare_parameter('transform_leader_to_follower_frame', True).value
        )
        self.tf_lookup_timeout_sec = float(self.declare_parameter('tf_lookup_timeout_sec', 0.05).value)

        leader_odom_override = str(self.declare_parameter('leader_odom_topic', '').value)
        follower_odom_override = str(self.declare_parameter('follower_odom_topic', '').value)
        cmd_override = str(self.declare_parameter('cmd_topic', '').value)

        leader_odom_topic = self._resolve_topic(leader_odom_override, self.leader_ns, self.odom_topic_name)
        follower_odom_topic = self._resolve_topic(follower_odom_override, self.follower_ns, self.odom_topic_name)
        cmd_topic = self._resolve_topic(cmd_override, self.follower_ns, self.cmd_topic_name)

        self._leader_odom: Optional[Odometry] = None
        self._follower_odom: Optional[Odometry] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_tf_warn_ns = 0

        self.create_subscription(Odometry, leader_odom_topic, self.on_leader_odom, 10)
        self.create_subscription(Odometry, follower_odom_topic, self.on_follower_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.control_rate_hz, 1e-3), self.on_timer)

        self.get_logger().info(
            f'Formation follower {self.follower_ns}: {leader_odom_topic} -> {cmd_topic} '
            f'(l*={self.l_star:.3f}, d*={self.gammaL_star:.3f} rad)'
        )

    @staticmethod
    def _resolve_topic(explicit_topic: str, namespace: str, leaf_name: str) -> str:
        explicit_topic = explicit_topic.strip()
        if explicit_topic:
            return explicit_topic if explicit_topic.startswith('/') else f'/{explicit_topic}'

        namespace = namespace.strip().strip('/')
        leaf_name = leaf_name.strip().strip('/')
        if namespace:
            return f'/{namespace}/{leaf_name}'
        return f'/{leaf_name}'

    def on_leader_odom(self, msg: Odometry) -> None:
        self._leader_odom = msg

    def on_follower_odom(self, msg: Odometry) -> None:
        self._follower_odom = msg

    def transform_leader_pose_to_frame(
        self, leader: Odometry, target_frame: str
    ) -> Optional[tuple[float, float, float]]:
        source_frame = leader.header.frame_id
        if not source_frame:
            return None
        if source_frame == target_frame:
            return (
                leader.pose.pose.position.x,
                leader.pose.pose.position.y,
                yaw_from_quat(leader.pose.pose.orientation),
            )

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=max(self.tf_lookup_timeout_sec, 0.0)),
            )
        except TransformException as ex:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > 1_000_000_000:
                self.get_logger().warn(f'Failed TF {source_frame} -> {target_frame}: {ex}')
                self._last_tf_warn_ns = now_ns
            return None

        yaw_tf = yaw_from_quat(tf.transform.rotation)
        c = math.cos(yaw_tf)
        s = math.sin(yaw_tf)

        x_src = leader.pose.pose.position.x
        y_src = leader.pose.pose.position.y
        x_tgt = (c * x_src) - (s * y_src) + tf.transform.translation.x
        y_tgt = (s * x_src) + (c * y_src) + tf.transform.translation.y

        psi_src = yaw_from_quat(leader.pose.pose.orientation)
        psi_tgt = wrap_pi(psi_src + yaw_tf)
        return x_tgt, y_tgt, psi_tgt

    def on_timer(self) -> None:
        if self._leader_odom is None or self._follower_odom is None:
            return

        leader = self._leader_odom
        follower = self._follower_odom

        target_frame = follower.header.frame_id
        if not target_frame:
            return

        if self.transform_leader_to_follower_frame:
            leader_in_follower_frame = self.transform_leader_pose_to_frame(leader, target_frame)
            if leader_in_follower_frame is None:
                return
            xL, yL, psiL = leader_in_follower_frame
        else:
            xL = leader.pose.pose.position.x
            yL = leader.pose.pose.position.y
            psiL = yaw_from_quat(leader.pose.pose.orientation)

        vL = leader.twist.twist.linear.x
        psi_dot_L = leader.twist.twist.angular.z

        xF = follower.pose.pose.position.x
        yF = follower.pose.pose.position.y
        psiF = yaw_from_quat(follower.pose.pose.orientation)

        dx = xF - xL
        dy = yF - yL
        l = math.hypot(dx, dy)
        l_eff = max(l, self.l_min)
        theta_line = math.atan2(dy, dx)

        gammaL = wrap_pi(theta_line - psiL)
        gammaF = wrap_pi(theta_line - psiF)

        F1 = -vL * math.cos(gammaL)
        F2 = (vL * math.sin(gammaL) / l_eff) - psi_dot_L

        e_l = self.l_star - l
        e_gamma = wrap_pi(self.gammaL_star - gammaL)
        p1 = clamp(self.k1 * e_l, -self.p1_max, self.p1_max)
        p2 = clamp(self.k2 * e_gamma, -self.p2_max, self.p2_max)

        q1 = p1 - F1
        q2 = p2 - F2

        c = math.cos(gammaF)
        s = math.sin(gammaF)

        vx = c * q1 - (l_eff * s) * q2
        vy = s * q1 + (l_eff * c) * q2

        vx = clamp(vx, -self.vx_max, self.vx_max)
        vy = clamp(vy, -self.vy_max, self.vy_max)

        wz = 0.0
        if self.align_follower_yaw:
            psi_target = wrap_pi(psiL + self.psi_offset)
            e_psi = wrap_pi(psi_target - psiF)
            wz = clamp(self.k_psi * e_psi, -self.wz_max, self.wz_max)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = FormationFollowerController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
