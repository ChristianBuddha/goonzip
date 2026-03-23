import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _yaw_from_quat_z_w(qz, qw):
    qz = float(qz)
    qw = float(qw)
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def _create_nodes(context, *args, **kwargs):
    map_yaml = LaunchConfiguration("map").perform(context)
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    autostart = _as_bool(LaunchConfiguration("autostart").perform(context))
    log_level = LaunchConfiguration("log_level").perform(context)
    robot_names_raw = LaunchConfiguration("robot_names").perform(context)
    use_initial_poses = _as_bool(
        LaunchConfiguration("use_initial_poses").perform(context)
    )

    robot_names = [
        name.strip().strip("/")
        for name in robot_names_raw.split(",")
        if name.strip()
    ]
    if not robot_names:
        raise RuntimeError("robot_names is empty. Example: robot1,robot2")

    initial_pose_by_robot = {
        "robot1": {
            "x": LaunchConfiguration("robot1_initial_x").perform(context),
            "y": LaunchConfiguration("robot1_initial_y").perform(context),
            "qz": LaunchConfiguration("robot1_initial_qz").perform(context),
            "qw": LaunchConfiguration("robot1_initial_qw").perform(context),
        },
        "robot2": {
            "x": LaunchConfiguration("robot2_initial_x").perform(context),
            "y": LaunchConfiguration("robot2_initial_y").perform(context),
            "qz": LaunchConfiguration("robot2_initial_qz").perform(context),
            "qw": LaunchConfiguration("robot2_initial_qw").perform(context),
        },
    }

    actions = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "yaml_filename": map_yaml,
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": ["map_server"]},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]

    for robot_ns in robot_names:
        amcl_params = {
            "use_sim_time": use_sim_time,
            "alpha1": 0.3,
            "alpha2": 0.3,
            "alpha3": 0.3,
            "alpha4": 0.3,
            "alpha5": 0.3,
            "base_frame_id": f"{robot_ns}/base_link",
            "odom_frame_id": f"{robot_ns}/odom",
            "global_frame_id": "map",
            "map_topic": "/map",
            "update_min_a": 0.0,
            "update_min_d": 0.0,
            "robot_model_type": "nav2_amcl::OmniMotionModel",
            "scan_topic": "scan",
            "laser_model_type": "likelihood_field",
            "max_beams": 60,
            "laser_likelihood_max_dist": 2.0,
            "laser_max_range": 100.0,
            "laser_min_range": -1.0,
            "sigma_hit": 0.2,
            "z_hit": 0.5,
            "z_rand": 0.5,
            "z_short": 0.05,
            "z_max": 0.05,
            "lambda_short": 0.1,
            "min_particles": 500,
            "max_particles": 2000,
            "pf_err": 0.05,
            "pf_z": 0.99,
            "resample_interval": 1,
            "recovery_alpha_fast": 0.0,
            "recovery_alpha_slow": 0.0,
            "random_seed": -1,
            "tf_broadcast": True,
            "transform_tolerance": 1.0,
            "set_initial_pose": False,
            "always_reset_initial_pose": False,
        }

        if use_initial_poses and robot_ns in initial_pose_by_robot:
            pose = initial_pose_by_robot[robot_ns]
            amcl_params["set_initial_pose"] = True
            amcl_params["initial_pose"] = {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": 0.0,
                "yaw": _yaw_from_quat_z_w(pose["qz"], pose["qw"]),
            }

        actions.extend(
            [
                Node(
                    package="nav2_amcl",
                    executable="amcl",
                    name="amcl",
                    namespace=robot_ns,
                    output="screen",
                    parameters=[amcl_params],
                    arguments=["--ros-args", "--log-level", log_level],
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_localization",
                    namespace=robot_ns,
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"autostart": autostart},
                        {"node_names": ["amcl"]},
                    ],
                    arguments=["--ros-args", "--log-level", log_level],
                ),
            ]
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Absolute path to map yaml file",
            ),
            DeclareLaunchArgument("robot_names", default_value="robot1,robot2"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("use_initial_poses", default_value="true"),
            DeclareLaunchArgument(
                "robot1_initial_x",
                default_value="0.8255554974077502",
            ),
            DeclareLaunchArgument(
                "robot1_initial_y",
                default_value="1.629472944906972",
            ),
            DeclareLaunchArgument(
                "robot1_initial_qz",
                default_value="-0.7849336466220255",
            ),
            DeclareLaunchArgument(
                "robot1_initial_qw",
                default_value="0.6195798337588541",
            ),
            DeclareLaunchArgument(
                "robot2_initial_x",
                default_value="-0.6817325744234557",
            ),
            DeclareLaunchArgument(
                "robot2_initial_y",
                default_value="1.9861121185603547",
            ),
            DeclareLaunchArgument(
                "robot2_initial_qz",
                default_value="-0.7786198435287842",
            ),
            DeclareLaunchArgument(
                "robot2_initial_qw",
                default_value="0.627495927686556",
            ),
            OpaqueFunction(function=_create_nodes),
        ]
    )
