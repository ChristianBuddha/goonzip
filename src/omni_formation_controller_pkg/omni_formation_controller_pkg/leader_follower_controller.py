#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .common import clamp, wrap_pi, yaw_from_quat


class LeaderFollowerController(Node):
    def __init__(self) -> None:
        super().__init__('leader_follower_controller')

        self.leader_pose_topic = str(
            self.declare_parameter('leader_pose_topic', '/robot1/amcl_pose').value
        )
        self.follower_pose_topic = str(
            self.declare_parameter('follower_pose_topic', '/robot2/amcl_pose').value
        )
        self.leader_odom_topic = str(
            self.declare_parameter('leader_odom_topic', '/robot1/odometry/filtered').value
        )
        self.follower_cmd_vel_topic = str(
            self.declare_parameter('follower_cmd_vel_topic', '/robot2/cmd_vel').value
        )

        self.l_star = float(self.declare_parameter('l_star', 0.25).value)
        self.gammaL_star = float(
            self.declare_parameter('gammaL_star', math.pi).value
        )
        self.k1 = float(self.declare_parameter('k1', 1.0).value)
        self.k2 = float(self.declare_parameter('k2', 1.0).value)
        self.k_psi = float(self.declare_parameter('k_psi', 1.0).value)
        self.psi_offset = float(self.declare_parameter('psi_offset', 0.0).value)
        self.align_follower_yaw = bool(
            self.declare_parameter('align_follower_yaw', True).value
        )
        self.l_min = float(self.declare_parameter('l_min', 0.25).value)
        self.p1_max = float(self.declare_parameter('p1_max', 1.5).value)
        self.p2_max = float(self.declare_parameter('p2_max', 2.5).value)
        self.vx_max = float(self.declare_parameter('vx_max', 0.4).value)
        self.vy_max = float(self.declare_parameter('vy_max', 0.4).value)
        self.wz_max = float(self.declare_parameter('wz_max', 0.4).value)
        self.control_rate_hz = float(
            self.declare_parameter('control_rate_hz', 50.0).value
        )
        self.transform_leader_to_follower_frame = bool(
            self.declare_parameter('transform_leader_to_follower_frame', True).value
        )
        self.tf_lookup_timeout_sec = float(
            self.declare_parameter('tf_lookup_timeout_sec', 0.05).value
        )
        self.status_log_period_sec = float(
            self.declare_parameter('status_log_period_sec', 1.0).value
        )
        self.log_cmd = bool(self.declare_parameter('log_cmd', False).value)

        self._leader_pose: Optional[PoseWithCovarianceStamped] = None
        self._follower_pose: Optional[PoseWithCovarianceStamped] = None
        self._leader_odom: Optional[Odometry] = None
        self._last_tf_warn_ns = 0
        self._last_status_log_ns = 0
        self._last_cmd_log_ns = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        amcl_pose_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.leader_pose_topic,
            self.on_leader_pose,
            amcl_pose_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.follower_pose_topic,
            self.on_follower_pose,
            amcl_pose_qos,
        )
        self.create_subscription(
            Odometry, self.leader_odom_topic, self.on_leader_odom, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.follower_cmd_vel_topic, 10)
        self.timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1e-3), self.on_timer
        )

        self.get_logger().info(
            'Starting formation controller: '
            f'leader_pose={self.leader_pose_topic}, '
            f'follower_pose={self.follower_pose_topic}, '
            f'leader_odom={self.leader_odom_topic}, '
            f'cmd={self.follower_cmd_vel_topic}'
        )

    def on_leader_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._leader_pose = msg

    def on_follower_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._follower_pose = msg

    def on_leader_odom(self, msg: Odometry) -> None:
        self._leader_odom = msg

    def _should_log_now(self, last_log_ns: int) -> bool:
        period_ns = int(max(self.status_log_period_sec, 0.1) * 1e9)
        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - last_log_ns) >= period_ns

    def transform_pose_to_frame(
        self, x_src: float, y_src: float, yaw_src: float, source_frame: str, target_frame: str
    ) -> Optional[tuple[float, float, float]]:
        if not source_frame or not target_frame:
            return None
        if source_frame == target_frame:
            return x_src, y_src, yaw_src

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=max(self.tf_lookup_timeout_sec, 0.0)),
            )
        except TransformException as ex:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > 1_000_000_000:
                self.get_logger().warn(
                    f'Failed TF {source_frame} -> {target_frame}: {ex}'
                )
                self._last_tf_warn_ns = now_ns
            return None

        yaw_tf = yaw_from_quat(tf_msg.transform.rotation)
        cos_yaw = math.cos(yaw_tf)
        sin_yaw = math.sin(yaw_tf)

        x_tgt = (cos_yaw * x_src) - (sin_yaw * y_src) + tf_msg.transform.translation.x
        y_tgt = (sin_yaw * x_src) + (cos_yaw * y_src) + tf_msg.transform.translation.y
        yaw_tgt = wrap_pi(yaw_src + yaw_tf)
        return x_tgt, y_tgt, yaw_tgt

    def on_timer(self) -> None:
        missing = []
        if self._leader_pose is None:
            missing.append('leader_pose')
        if self._follower_pose is None:
            missing.append('follower_pose')
        if missing:
            now_ns = self.get_clock().now().nanoseconds
            if self._should_log_now(self._last_status_log_ns):
                self.get_logger().warn(
                    'Waiting for required inputs: ' + ', '.join(missing)
                )
                self._last_status_log_ns = now_ns
            return

        leader_pose = self._leader_pose
        follower_pose = self._follower_pose
        leader_odom = self._leader_odom

        leader_frame = leader_pose.header.frame_id
        follower_frame = follower_pose.header.frame_id
        if not leader_frame or not follower_frame:
            return

        x_leader = leader_pose.pose.pose.position.x
        y_leader = leader_pose.pose.pose.position.y
        psi_leader = yaw_from_quat(leader_pose.pose.pose.orientation)

        if self.transform_leader_to_follower_frame:
            transformed = self.transform_pose_to_frame(
                x_leader,
                y_leader,
                psi_leader,
                leader_frame,
                follower_frame,
            )
            if transformed is None:
                return
            x_leader, y_leader, psi_leader = transformed
        elif leader_frame != follower_frame:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > 1_000_000_000:
                self.get_logger().warn(
                    f'Leader frame {leader_frame} and follower frame {follower_frame} differ; '
                    'enable transform_leader_to_follower_frame or use a common frame.'
                )
                self._last_tf_warn_ns = now_ns
            return

        x_follower = follower_pose.pose.pose.position.x
        y_follower = follower_pose.pose.pose.position.y
        psi_follower = yaw_from_quat(follower_pose.pose.pose.orientation)

        dx = x_follower - x_leader
        dy = y_follower - y_leader
        distance = math.hypot(dx, dy)
        distance_eff = max(distance, self.l_min)
        theta_line = math.atan2(dy, dx)

        gammaL = wrap_pi(theta_line - psi_leader)
        gammaF = wrap_pi(theta_line - psi_follower)

        if leader_odom is None:
            v_leader = 0.0
            psi_dot_leader = 0.0
            now_ns = self.get_clock().now().nanoseconds
            if self._should_log_now(self._last_status_log_ns):
                self.get_logger().warn(
                    'leader_odom not received yet; using zero feedforward.'
                )
                self._last_status_log_ns = now_ns
        else:
            v_leader = leader_odom.twist.twist.linear.x
            psi_dot_leader = leader_odom.twist.twist.angular.z

        f1 = -v_leader * math.cos(gammaL)
        f2 = (v_leader * math.sin(gammaL) / distance_eff) - psi_dot_leader

        e_l = self.l_star - distance
        e_gamma = wrap_pi(self.gammaL_star - gammaL)
        p1 = clamp(self.k1 * e_l, -self.p1_max, self.p1_max)
        p2 = clamp(self.k2 * e_gamma, -self.p2_max, self.p2_max)

        q1 = p1 - f1
        q2 = p2 - f2

        cos_gamma = math.cos(gammaF)
        sin_gamma = math.sin(gammaF)

        vx = cos_gamma * q1 - (distance_eff * sin_gamma) * q2
        vy = sin_gamma * q1 + (distance_eff * cos_gamma) * q2

        cmd = Twist()
        cmd.linear.x = clamp(vx, -self.vx_max, self.vx_max)
        cmd.linear.y = clamp(vy, -self.vy_max, self.vy_max)

        if self.align_follower_yaw:
            psi_target = wrap_pi(psi_leader + self.psi_offset)
            e_psi = wrap_pi(psi_target - psi_follower)
            cmd.angular.z = clamp(self.k_psi * e_psi, -self.wz_max, self.wz_max)

        self.cmd_pub.publish(cmd)

        if self.log_cmd and self._should_log_now(self._last_cmd_log_ns):
            self.get_logger().info(
                f'distance={distance:.3f}, gammaL={gammaL:.3f}, gammaF={gammaF:.3f}, '
                f'cmd=({cmd.linear.x:.3f}, {cmd.linear.y:.3f}, {cmd.angular.z:.3f})'
            )
            self._last_cmd_log_ns = self.get_clock().now().nanoseconds


def main() -> None:
    rclpy.init()
    node = LeaderFollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
