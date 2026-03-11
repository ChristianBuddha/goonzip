from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    leader_frame = LaunchConfiguration('leader_frame')
    leader_center_x = LaunchConfiguration('leader_center_x')
    leader_center_y = LaunchConfiguration('leader_center_y')
    leader_radius = LaunchConfiguration('leader_radius')
    leader_angular_speed = LaunchConfiguration('leader_angular_speed')
    leader_update_rate_hz = LaunchConfiguration('leader_update_rate_hz')
    l_star = LaunchConfiguration('l_star')
    transform_leader_to_follower_frame = LaunchConfiguration('transform_leader_to_follower_frame')
    tf_lookup_timeout_sec = LaunchConfiguration('tf_lookup_timeout_sec')

    leader = Node(
        package='omni_follow_controller_pkg',
        executable='virtual_circle_leader',
        name='virtual_circle_leader',
        output='screen',
        parameters=[{
            'robot_ns': 'robot1',
            'frame_id': leader_frame,
            'child_frame_id': 'robot1/base_link',
            'center_x': leader_center_x,
            'center_y': leader_center_y,
            'radius': leader_radius,
            'angular_speed': leader_angular_speed,
            'update_rate_hz': leader_update_rate_hz,
            'publish_tf': True,
        }]
    )

    follower = Node(
        package='omni_follow_controller_pkg',
        executable='omni_follower_controller',
        name='omni_follower_controller',
        output='screen',
        remappings=[
            ('/robot2/sensor_odom', '/robot2/odometry/filtered'),
        ],
        parameters=[{
            'leader_ns': 'robot1',
            'follower_ns': 'robot2',
            'l_star': l_star,
            'gammaL_star': 2.88893,  # follower trails behind leader
            'k1': 1.0,
            'k2': 1.0,
            'k_psi': 1.0,
            'align_follower_yaw': True,
            'psi_offset': 0.0,
            'transform_leader_to_follower_frame': transform_leader_to_follower_frame,
            'tf_lookup_timeout_sec': tf_lookup_timeout_sec,
            'vx_max': 0.4,
            'vy_max': 0.4,
            'wz_max': 0.4,
            'control_rate_hz': 50.0,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('leader_frame', default_value='map'),
        DeclareLaunchArgument('leader_center_x', default_value='0.0'),
        DeclareLaunchArgument('leader_center_y', default_value='0.0'),
        DeclareLaunchArgument('leader_radius', default_value='1.0'),
        DeclareLaunchArgument('leader_angular_speed', default_value='0.1'),
        DeclareLaunchArgument('leader_update_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('l_star', default_value='0.25'),
        DeclareLaunchArgument('transform_leader_to_follower_frame', default_value='true'),
        DeclareLaunchArgument('tf_lookup_timeout_sec', default_value='0.05'),
        leader,
        follower,
    ])
