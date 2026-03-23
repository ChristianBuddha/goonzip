from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _amcl_multi_launch_source():
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare('nexus_base_ros'), 'launch', 'amcl_multi.launch.py']
        )
    )


def _formation_controller_launch_source():
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [
                FindPackageShare('omni_formation_controller_pkg'),
                'launch',
                'formation_controller.launch.py',
            ]
        )
    )


def generate_launch_description():
    controller = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_controller')),
        actions=[
            TimerAction(
                period=LaunchConfiguration('controller_start_delay_sec'),
                actions=[
                    IncludeLaunchDescription(
                        _formation_controller_launch_source(),
                        launch_arguments={
                            'leader_pose_topic': '/robot1/amcl_pose',
                            'follower_pose_topic': '/robot2/amcl_pose',
                            'leader_odom_topic': '/robot1/odometry/filtered',
                            'follower_cmd_vel_topic': '/robot2/cmd_vel',
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
                        }.items(),
                    )
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map', description='Absolute path to map yaml file'
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('amcl_log_level', default_value='info'),
            DeclareLaunchArgument('use_initial_poses', default_value='true'),
            DeclareLaunchArgument('enable_controller', default_value='true'),
            DeclareLaunchArgument('controller_start_delay_sec', default_value='4.0'),
            DeclareLaunchArgument('robot1_initial_x', default_value='0.020460318663828497'),
            DeclareLaunchArgument('robot1_initial_y', default_value='1.7073198976839097'),
            DeclareLaunchArgument('robot1_initial_qz', default_value='-0.6666470222544838'),
            DeclareLaunchArgument('robot1_initial_qw', default_value='0.7453735625303796'),
            DeclareLaunchArgument('robot2_initial_x', default_value='-0.6817325744234557'),
            DeclareLaunchArgument('robot2_initial_y', default_value='1.9861121185603547'),
            DeclareLaunchArgument('robot2_initial_qz', default_value='-0.7786198435287842'),
            DeclareLaunchArgument('robot2_initial_qw', default_value='0.627495927686556'),
            DeclareLaunchArgument('l_star', default_value='0.60'),
            DeclareLaunchArgument('gammaL_star', default_value='3.141592653589793'),
            DeclareLaunchArgument('k1', default_value='0.8'),
            DeclareLaunchArgument('k2', default_value='0.8'),
            DeclareLaunchArgument('k_psi', default_value='0.8'),
            DeclareLaunchArgument('psi_offset', default_value='0.0'),
            DeclareLaunchArgument('align_follower_yaw', default_value='true'),
            DeclareLaunchArgument(
                'transform_leader_to_follower_frame', default_value='true'
            ),
            DeclareLaunchArgument('tf_lookup_timeout_sec', default_value='0.1'),
            DeclareLaunchArgument('vx_max', default_value='0.20'),
            DeclareLaunchArgument('vy_max', default_value='0.20'),
            DeclareLaunchArgument('wz_max', default_value='0.30'),
            DeclareLaunchArgument('control_rate_hz', default_value='30.0'),
            IncludeLaunchDescription(
                _amcl_multi_launch_source(),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'robot_names': 'robot1,robot2',
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                    'log_level': LaunchConfiguration('amcl_log_level'),
                    'use_initial_poses': LaunchConfiguration('use_initial_poses'),
                    'robot1_initial_x': LaunchConfiguration('robot1_initial_x'),
                    'robot1_initial_y': LaunchConfiguration('robot1_initial_y'),
                    'robot1_initial_qz': LaunchConfiguration('robot1_initial_qz'),
                    'robot1_initial_qw': LaunchConfiguration('robot1_initial_qw'),
                    'robot2_initial_x': LaunchConfiguration('robot2_initial_x'),
                    'robot2_initial_y': LaunchConfiguration('robot2_initial_y'),
                    'robot2_initial_qz': LaunchConfiguration('robot2_initial_qz'),
                    'robot2_initial_qw': LaunchConfiguration('robot2_initial_qw'),
                }.items(),
            ),
            controller,
        ]
    )
