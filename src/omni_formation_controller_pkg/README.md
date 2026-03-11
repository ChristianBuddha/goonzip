# omni_formation_controller_pkg

ROS 2 Python package for follower-side omni formation control with a real leader robot.

## Current model

- `robot1` is the real leader.
- `robot2`, `robot3`, ... are followers.
- Each follower runs the same relative-state controller used in `omni_follow_controller_pkg`.
- The formation slot for each follower is defined by:
  - `l_star`: desired distance to leader
  - `gammaL_star` or `d_star`: desired angle of the leader->follower line measured in the leader frame

This keeps the implementation close to the existing controller while allowing multiple followers by launching multiple controller instances.

## Default topics

- Leader state: `/<leader_ns>/odometry/filtered`
- Follower state: `/<follower_ns>/odometry/filtered`
- Command output: `/<follower_ns>/cmd_vel`

The controller uses the odometry message pose and twist directly. TF is used to transform the leader pose into the follower odom frame when the robots are localized independently.

## Formation meaning

- `gammaL_star =  pi`   -> follower behind leader
- `gammaL_star =  0`    -> follower in front of leader
- `gammaL_star =  pi/2` -> follower on leader's left
- `gammaL_star = -pi/2` -> follower on leader's right

For a left-side parallel formation with `0.5 m` spacing:

- `l_star = 0.5`
- `gammaL_star = pi/2`

## Build

```bash
colcon build --packages-select omni_formation_controller_pkg
source install/setup.bash
```

## Run

Default launch for one follower on the left side of `robot1`:

```bash
ros2 launch omni_formation_controller_pkg parallel_formation.launch.py
```

Default follower spec:

```text
robot2:0.5:left
```

You can add more followers with `follower_specs`:

```bash
ros2 launch omni_formation_controller_pkg parallel_formation.launch.py \
  follower_specs:="robot2:0.5:left,robot3:0.5:right"
```

Each entry is:

```text
<follower_ns>:<distance_m>:<left|right|front|rear|angle_rad>
```

## Key parameters

Controller node:

- `leader_ns`
- `follower_ns`
- `odom_topic_name`
- `cmd_topic_name`
- `l_star`
- `gammaL_star` or `d_star`
- `k1`, `k2`, `k_psi`
- `align_follower_yaw`
- `psi_offset`
- `vx_max`, `vy_max`, `wz_max`
- `transform_leader_to_follower_frame`
- `tf_lookup_timeout_sec`

Launch defaults:

- `leader_ns = robot1`
- `follower_specs = robot2:0.5:left`
- `odom_topic_name = odometry/filtered`
- `cmd_topic_name = cmd_vel`
- `vx_max = 0.2`
- `vy_max = 0.2`
- `wz_max = 0.4`

## Notes

- This package only commands followers. The leader path remains external, for example Nav2 on `robot1`.
- The current implementation assumes each robot already publishes TF and filtered odometry.
- Obstacle avoidance is not handled in this package yet.
