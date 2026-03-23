from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'leader_pose_topic', default_value='/robot1/amcl_pose'
            ),
            DeclareLaunchArgument(
                'follower_pose_topic', default_value='/robot2/amcl_pose'
            ),
            DeclareLaunchArgument(
                'leader_odom_topic', default_value='/robot1/odometry/filtered'
            ),
            DeclareLaunchArgument(
                'follower_cmd_vel_topic', default_value='/robot2/cmd_vel'
            ),
            DeclareLaunchArgument('l_star', default_value='0.25'),
            DeclareLaunchArgument('gammaL_star', default_value='3.141592653589793'),
            DeclareLaunchArgument('k1', default_value='1.0'),
            DeclareLaunchArgument('k2', default_value='1.0'),
            DeclareLaunchArgument('k_psi', default_value='1.0'),
            DeclareLaunchArgument('psi_offset', default_value='0.0'),
            DeclareLaunchArgument('align_follower_yaw', default_value='true'),
            DeclareLaunchArgument(
                'transform_leader_to_follower_frame', default_value='true'
            ),
            DeclareLaunchArgument('tf_lookup_timeout_sec', default_value='0.05'),
            DeclareLaunchArgument('vx_max', default_value='0.4'),
            DeclareLaunchArgument('vy_max', default_value='0.4'),
            DeclareLaunchArgument('wz_max', default_value='0.4'),
            DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
            Node(
                package='omni_formation_controller_pkg',
                executable='leader_follower_controller',
                name='leader_follower_controller',
                output='screen',
                parameters=[
                    {
                        'leader_pose_topic': LaunchConfiguration('leader_pose_topic'),
                        'follower_pose_topic': LaunchConfiguration('follower_pose_topic'),
                        'leader_odom_topic': LaunchConfiguration('leader_odom_topic'),
                        'follower_cmd_vel_topic': LaunchConfiguration(
                            'follower_cmd_vel_topic'
                        ),
                        'l_star': LaunchConfiguration('l_star'),
                        'gammaL_star': LaunchConfiguration('gammaL_star'),
                        'k1': LaunchConfiguration('k1'),
                        'k2': LaunchConfiguration('k2'),
                        'k_psi': LaunchConfiguration('k_psi'),
                        'psi_offset': LaunchConfiguration('psi_offset'),
                        'align_follower_yaw': LaunchConfiguration(
                            'align_follower_yaw'
                        ),
                        'transform_leader_to_follower_frame': LaunchConfiguration(
                            'transform_leader_to_follower_frame'
                        ),
                        'tf_lookup_timeout_sec': LaunchConfiguration(
                            'tf_lookup_timeout_sec'
                        ),
                        'vx_max': LaunchConfiguration('vx_max'),
                        'vy_max': LaunchConfiguration('vy_max'),
                        'wz_max': LaunchConfiguration('wz_max'),
                        'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                    }
                ],
            ),
        ]
    )
