from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _create_nodes(context, *args, **kwargs):
    map_yaml = LaunchConfiguration("map").perform(context)
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    autostart = _as_bool(LaunchConfiguration("autostart").perform(context))
    log_level = LaunchConfiguration("log_level").perform(context)
    robot_names_raw = LaunchConfiguration("robot_names").perform(context)

    robot_names = [
        name.strip().strip("/")
        for name in robot_names_raw.split(",")
        if name.strip()
    ]
    if not robot_names:
        raise RuntimeError("robot_names is empty. Example: robot1,robot2")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    actions = []

    for robot_ns in robot_names:
        amcl_params = {
            "use_sim_time": use_sim_time,
            "alpha1": 0.2,
            "alpha2": 0.2,
            "alpha3": 0.2,
            "alpha4": 0.2,
            "alpha5": 0.2,
            "base_frame_id": f"{robot_ns}/base_footprint",
            "odom_frame_id": f"{robot_ns}/odom",
            "global_frame_id": "map",
            "update_min_a": 0.0,
            "update_min_d": 0.0,
            "robot_model_type": "nav2_amcl::OmniMotionModel",
            # Relative topic -> /<robot_ns>/scan due node namespace
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
        }

        actions.extend(
            [
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    namespace=robot_ns,
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "yaml_filename": map_yaml,
                        }
                    ],
                    remappings=remappings,
                    arguments=["--ros-args", "--log-level", log_level],
                ),
                Node(
                    package="nav2_amcl",
                    executable="amcl",
                    name="amcl",
                    namespace=robot_ns,
                    output="screen",
                    parameters=[amcl_params],
                    remappings=remappings,
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
                        {"node_names": ["map_server", "amcl"]},
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
            OpaqueFunction(function=_create_nodes),
        ]
    )
