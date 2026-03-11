from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    robot_ns = LaunchConfiguration("robot_ns")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    imu_frame = LaunchConfiguration("imu_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    base_footprint_frame = LaunchConfiguration("base_footprint_frame")

    return LaunchDescription([
        DeclareLaunchArgument("robot_ns", default_value="robot1"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyNEXUS"),
        DeclareLaunchArgument("serial_baud", default_value="115200"),
        DeclareLaunchArgument("lidar_serial_port", default_value="/dev/ttyLIDAR"),
        DeclareLaunchArgument("lidar_serial_baudrate", default_value="256000"),
        DeclareLaunchArgument("odom_frame", default_value=[robot_ns, "/odom"]),
        DeclareLaunchArgument("base_frame", default_value=[robot_ns, "/base_link"]),
        DeclareLaunchArgument("imu_frame", default_value=[robot_ns, "/imu_link"]),
        DeclareLaunchArgument("laser_frame", default_value=[robot_ns, "/laser"]),
        DeclareLaunchArgument("base_footprint_frame", default_value=[robot_ns, "/base_footprint"]),

        # =========================
        # EKF
        # =========================
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("nexus_base_ros"),
                "config",
                "ekf.yaml",
            ]),
        ),
        DeclareLaunchArgument("ekf_use_sim_time", default_value="false"),
        DeclareLaunchArgument("imu_driver", default_value="bno055"),

        # =========================
        # Base bringup (모터, IMU, TF 등)
        # =========================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("nexus_base_ros"),
                    "launch",
                    "nexus_bringup.launch.py",
                ])
            ),
            launch_arguments={
                "robot_ns": robot_ns,
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baud": LaunchConfiguration("serial_baud"),
                "imu_driver": LaunchConfiguration("imu_driver"),
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "imu_frame": imu_frame,
                "laser_frame": laser_frame,
                "base_footprint_frame": base_footprint_frame,
            }.items(),
        ),

        # =========================
        # EKF node
        # =========================
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            namespace=robot_ns,
            output="screen",
            parameters=[
                LaunchConfiguration("ekf_params_file"),
                {"use_sim_time": LaunchConfiguration("ekf_use_sim_time")},
                {
                    "odom_frame": odom_frame,
                    "base_link_frame": base_frame,
                    "world_frame": odom_frame,
                },
            ],
        ),

        # =========================
        # LiDAR (1.5초 지연 실행)
        # =========================
        TimerAction(
            period=1.5,
            actions=[
                GroupAction(
                    actions=[
                        PushRosNamespace(robot_ns),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                PathJoinSubstitution([
                                    FindPackageShare("sllidar_ros2"),
                                    "launch",
                                    "sllidar_a2m12_launch.py",
                                ])
                            ),
                            launch_arguments={
                                "channel_type": "serial",
                                "serial_port": LaunchConfiguration("lidar_serial_port"),
                                "serial_baudrate": LaunchConfiguration("lidar_serial_baudrate"),
                                "frame_id": laser_frame,
                                "inverted": "false",
                                "angle_compensate": "true",
                                "scan_mode": "Sensitivity",
                            }.items(),
                        ),
                    ]
                )
            ],
        ),
    ])
