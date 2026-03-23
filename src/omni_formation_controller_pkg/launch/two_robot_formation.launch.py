from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _full_launch_source():
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare('nexus_base_ros'), 'launch', 'full.launch.py']
        )
    )


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
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    robot1 = IncludeLaunchDescription(
        _full_launch_source(),
        launch_arguments={
            'robot_ns': 'robot1',
            'serial_port': LaunchConfiguration('robot1_serial_port'),
            'serial_baud': LaunchConfiguration('robot1_serial_baud'),
            'lidar_serial_port': LaunchConfiguration('robot1_lidar_serial_port'),
            'lidar_serial_baudrate': LaunchConfiguration(
                'robot1_lidar_serial_baudrate'
            ),
            'ekf_use_sim_time': use_sim_time,
            'imu_driver': LaunchConfiguration('robot1_imu_driver'),
        }.items(),
    )

    robot2 = IncludeLaunchDescription(
        _full_launch_source(),
        launch_arguments={
            'robot_ns': 'robot2',
            'serial_port': LaunchConfiguration('robot2_serial_port'),
            'serial_baud': LaunchConfiguration('robot2_serial_baud'),
            'lidar_serial_port': LaunchConfiguration('robot2_lidar_serial_port'),
            'lidar_serial_baudrate': LaunchConfiguration(
                'robot2_lidar_serial_baudrate'
            ),
            'ekf_use_sim_time': use_sim_time,
            'imu_driver': LaunchConfiguration('robot2_imu_driver'),
        }.items(),
    )

    localization = TimerAction(
        period=LaunchConfiguration('localization_start_delay_sec'),
        actions=[
            IncludeLaunchDescription(
                _amcl_multi_launch_source(),
                launch_arguments={
                    'map': map_yaml,
                    'robot_names': 'robot1,robot2',
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'log_level': LaunchConfiguration('amcl_log_level'),
                }.items(),
            )
        ],
    )

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
                            'control_rate_hz': LaunchConfiguration(
                                'control_rate_hz'
                            ),
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
            DeclareLaunchArgument('enable_controller', default_value='true'),
            DeclareLaunchArgument(
                'localization_start_delay_sec', default_value='2.0'
            ),
            DeclareLaunchArgument(
                'controller_start_delay_sec', default_value='5.0'
            ),
            DeclareLaunchArgument('amcl_log_level', default_value='info'),
            DeclareLaunchArgument('robot1_serial_port', default_value='/dev/ttyNEXUS1'),
            DeclareLaunchArgument('robot1_serial_baud', default_value='115200'),
            DeclareLaunchArgument(
                'robot1_lidar_serial_port', default_value='/dev/ttyLIDAR1'
            ),
            DeclareLaunchArgument(
                'robot1_lidar_serial_baudrate', default_value='256000'
            ),
            DeclareLaunchArgument('robot1_imu_driver', default_value='bno055'),
            DeclareLaunchArgument('robot2_serial_port', default_value='/dev/ttyNEXUS2'),
            DeclareLaunchArgument('robot2_serial_baud', default_value='115200'),
            DeclareLaunchArgument(
                'robot2_lidar_serial_port', default_value='/dev/ttyLIDAR2'
            ),
            DeclareLaunchArgument(
                'robot2_lidar_serial_baudrate', default_value='256000'
            ),
            DeclareLaunchArgument('robot2_imu_driver', default_value='bno055'),
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
            robot1,
            robot2,
            localization,
            controller,
        ]
    )
