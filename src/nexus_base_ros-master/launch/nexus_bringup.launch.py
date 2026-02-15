from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription 
from launch.conditions import IfCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare 
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyNEXUS"),
        DeclareLaunchArgument("serial_baud", default_value="115200"),
        DeclareLaunchArgument("tx_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("cmd_timeout_ms", default_value="250"),
        DeclareLaunchArgument("use_joystick", default_value="true"),
        DeclareLaunchArgument("axis_linear_x", default_value="3"),
        DeclareLaunchArgument("axis_linear_y", default_value="2"),
        DeclareLaunchArgument("axis_angular_z", default_value="0"),
        DeclareLaunchArgument("max_vel_x", default_value="0.2"),
        DeclareLaunchArgument("max_vel_y", default_value="0.2"),
        DeclareLaunchArgument("max_vel_th", default_value="0.2"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.0"),
        DeclareLaunchArgument("imu_roll", default_value="0.0"),
        DeclareLaunchArgument("imu_pitch", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="0.0"),
        DeclareLaunchArgument("laser_x", default_value="0.0"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.0"),
        DeclareLaunchArgument("laser_roll", default_value="0.0"),
        DeclareLaunchArgument("laser_pitch", default_value="0.0"),
        DeclareLaunchArgument("laser_yaw", default_value="3.141592653"),
        DeclareLaunchArgument(
            "bno055_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("bno055"),
                "params",
                "bno055_params_i2c.yaml",
            ]),
        ),

        # Node(
            # package="nexus_base_ros",
            # executable="nexus_teleop_joy",
            # name="teleop_joy",
            # condition=IfCondition(LaunchConfiguration("use_joystick")),
            # parameters=[{
                # "axis_linear_x": LaunchConfiguration("axis_linear_x"),
                # "axis_linear_y": LaunchConfiguration("axis_linear_y"),
                # "axis_angular_z": LaunchConfiguration("axis_angular_z"),
                # "max_vel_x": LaunchConfiguration("max_vel_x"),
                # "max_vel_y": LaunchConfiguration("max_vel_y"),
                # "max_vel_th": LaunchConfiguration("max_vel_th"),
            # }],
            # output="screen",
        # ),

        Node(
            package="nexus_base_ros",
            executable="nexus_serial_bridge",
            name="nexus_serial_bridge",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baud": LaunchConfiguration("serial_baud"),
                "tx_rate_hz": LaunchConfiguration("tx_rate_hz"),
                "cmd_timeout_ms": LaunchConfiguration("cmd_timeout_ms"),
                "frame_id": LaunchConfiguration("base_frame"),
            }],
            output="screen",
        ),

        Node(
            package="nexus_base_ros",
            executable="nexus_odometry",
            name="nexus_odometry",
            parameters=[{
                "frame_id": LaunchConfiguration("odom_frame"),
                "child_frame_id": LaunchConfiguration("base_frame"),
                "publish_tf": LaunchConfiguration("publish_tf"),
            }],
            output="screen",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_imu",
            arguments=[
                LaunchConfiguration("imu_x"),
                LaunchConfiguration("imu_y"),
                LaunchConfiguration("imu_z"),
                LaunchConfiguration("imu_yaw"),
                LaunchConfiguration("imu_pitch"),
                LaunchConfiguration("imu_roll"),
                LaunchConfiguration("base_frame"),
                "imu_link",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_laser",
            arguments=[
                LaunchConfiguration("laser_x"),
                LaunchConfiguration("laser_y"),
                LaunchConfiguration("laser_z"),
                LaunchConfiguration("laser_yaw"),
                LaunchConfiguration("laser_pitch"),
                LaunchConfiguration("laser_roll"),
                LaunchConfiguration("base_frame"),
                "laser",
            ],
        ),
    
        Node(
            package="bno055",
            executable="bno055",
            name="bno055",
            parameters=[LaunchConfiguration("bno055_params")],
            output="screen",
        ),
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_footprint",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                LaunchConfiguration("base_frame"),
                "base_footprint",
            ],
        ),
        
    ])