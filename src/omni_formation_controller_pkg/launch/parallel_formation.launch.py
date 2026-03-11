import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _parse_gamma(token: str) -> float:
    token = token.strip().lower()
    if token == 'left':
        return math.pi / 2.0
    if token == 'right':
        return -math.pi / 2.0
    if token == 'front':
        return 0.0
    if token in ('rear', 'back', 'behind'):
        return math.pi
    return float(token)


def _create_nodes(context, *args, **kwargs):
    leader_ns = LaunchConfiguration('leader_ns').perform(context).strip().strip('/')
    follower_specs_raw = LaunchConfiguration('follower_specs').perform(context)
    odom_topic_name = LaunchConfiguration('odom_topic_name').perform(context)
    cmd_topic_name = LaunchConfiguration('cmd_topic_name').perform(context)

    params = {
        'leader_ns': leader_ns,
        'odom_topic_name': odom_topic_name,
        'cmd_topic_name': cmd_topic_name,
        'k1': float(LaunchConfiguration('k1').perform(context)),
        'k2': float(LaunchConfiguration('k2').perform(context)),
        'k_psi': float(LaunchConfiguration('k_psi').perform(context)),
        'align_follower_yaw': _as_bool(LaunchConfiguration('align_follower_yaw').perform(context)),
        'psi_offset': float(LaunchConfiguration('psi_offset').perform(context)),
        'l_min': float(LaunchConfiguration('l_min').perform(context)),
        'p1_max': float(LaunchConfiguration('p1_max').perform(context)),
        'p2_max': float(LaunchConfiguration('p2_max').perform(context)),
        'vx_max': float(LaunchConfiguration('vx_max').perform(context)),
        'vy_max': float(LaunchConfiguration('vy_max').perform(context)),
        'wz_max': float(LaunchConfiguration('wz_max').perform(context)),
        'control_rate_hz': float(LaunchConfiguration('control_rate_hz').perform(context)),
        'transform_leader_to_follower_frame': _as_bool(
            LaunchConfiguration('transform_leader_to_follower_frame').perform(context)
        ),
        'tf_lookup_timeout_sec': float(LaunchConfiguration('tf_lookup_timeout_sec').perform(context)),
    }

    nodes = []
    for spec in [item.strip() for item in follower_specs_raw.split(',') if item.strip()]:
        parts = [part.strip() for part in spec.split(':')]
        if len(parts) != 3:
            raise RuntimeError(
                'Invalid follower_specs entry. Use <follower_ns>:<distance_m>:<left|right|front|rear|angle_rad>'
            )

        follower_ns, distance_text, gamma_text = parts
        node_params = dict(params)
        node_params.update(
            {
                'follower_ns': follower_ns.strip('/'),
                'l_star': float(distance_text),
                'gammaL_star': _parse_gamma(gamma_text),
            }
        )
        nodes.append(
            Node(
                package='omni_formation_controller_pkg',
                executable='formation_follower_controller',
                name=f'formation_follower_controller_{follower_ns.strip("/")}',
                output='screen',
                parameters=[node_params],
            )
        )

    if not nodes:
        raise RuntimeError('follower_specs is empty. Example: robot2:0.5:left')

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('leader_ns', default_value='robot1'),
            DeclareLaunchArgument('follower_specs', default_value='robot2:0.5:left'),
            DeclareLaunchArgument('odom_topic_name', default_value='odometry/filtered'),
            DeclareLaunchArgument('cmd_topic_name', default_value='cmd_vel'),
            DeclareLaunchArgument('k1', default_value='1.0'),
            DeclareLaunchArgument('k2', default_value='1.0'),
            DeclareLaunchArgument('k_psi', default_value='1.0'),
            DeclareLaunchArgument('align_follower_yaw', default_value='true'),
            DeclareLaunchArgument('psi_offset', default_value='0.0'),
            DeclareLaunchArgument('l_min', default_value='0.25'),
            DeclareLaunchArgument('p1_max', default_value='1.0'),
            DeclareLaunchArgument('p2_max', default_value='2.0'),
            DeclareLaunchArgument('vx_max', default_value='0.2'),
            DeclareLaunchArgument('vy_max', default_value='0.2'),
            DeclareLaunchArgument('wz_max', default_value='0.4'),
            DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
            DeclareLaunchArgument('transform_leader_to_follower_frame', default_value='true'),
            DeclareLaunchArgument('tf_lookup_timeout_sec', default_value='0.05'),
            OpaqueFunction(function=_create_nodes),
        ]
    )
