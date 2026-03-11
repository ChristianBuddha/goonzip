# omni_follow_controller_pkg

ROS 2 Python package for **omnidirectional follower control** using the relative-state feedback-linearization idea discussed earlier.

## What is included

- `virtual_circle_leader`
  - Publishes a **virtual leader** under namespace `/robot1`
  - Topic: `/robot1/sensor_odom`
  - Optional visualization topic: `/robot1/reference_path`
  - Motion: circular trajectory with tangent heading

- `omni_follower_controller`
  - Subscribes to `/robot1/sensor_odom` as the virtual reference
  - Subscribes to `/robot2/sensor_odom` as the real follower state
  - Publishes `/robot2/cmd_vel`
  - Can transform leader pose into follower odom frame via TF (`transform_leader_to_follower_frame`)
  - Uses body-frame omni commands `linear.x = vx`, `linear.y = vy`, `angular.z = wz`

## Assumed topic structure

This matches your existing `nexus_odometry.cpp` / `nexus_serial_bridge.cpp` style when each robot is launched in a namespace:

- `/robot1/sensor_odom`
- `/robot2/sensor_odom`
- `/robot2/cmd_vel`

## Control model

State:

x = [l, gamma_L]^T

- `l`: distance from leader to follower
- `gamma_L`: angle(line leader->follower) - leader yaw

Relative angle at follower:

gamma_F = angle(line leader->follower) - follower yaw

For an omni robot with body-frame input u = [vx, vy]^T:

x_dot = F(x) + G(x) u

F(x) = [ -vL cos(gamma_L), vL sin(gamma_L)/l - psi_dot_L ]^T

G(x) = [[ cos(gamma_F),  sin(gamma_F)],
        [ sin(gamma_F)/l, -cos(gamma_F)/l ]]

Controller:

u = G(x)^(-1) (p - F(x))

p = [ k1 (l* - l), k2 (gammaL* - gamma_L) ]^T

## Recommended formation meaning

Because `gamma_L` is defined from **leader heading** to the **leader->follower line**:

- `gammaL_star = pi`   : follower trails behind leader
- `gammaL_star = 0`    : follower stays in front of leader
- `gammaL_star = pi/2` : follower stays to leader's left
- `gammaL_star = -pi/2`: follower stays to leader's right

## Build

Put this package in your ROS 2 workspace `src/` and build:

```bash
colcon build --packages-select omni_follow_controller_pkg
source install/setup.bash
```

## Run

### 1) Start your real follower robot stack in namespace `robot2`
That means your existing odometry/serial nodes should end up exposing:

- `/robot2/sensor_odom`
- `/robot2/cmd_vel`

### 2) Launch virtual leader + follower controller

```bash
ros2 launch omni_follow_controller_pkg virtual_circle_follow.launch.py
```

## Important parameters

### Virtual leader

- `robot_ns` (default `robot1`)
- `center_x`, `center_y`
- `radius`
- `angular_speed`

### Follower controller

- `leader_ns` (default `robot1`)
- `follower_ns` (default `robot2`)
- `l_star`
- `gammaL_star`
- `k1`, `k2`
- `k_psi`
- `align_follower_yaw`
- `vx_max`, `vy_max`, `wz_max`
- `l_min`
- `transform_leader_to_follower_frame` (default `true`)
- `tf_lookup_timeout_sec` (default `0.05`)

## Notes

- The controller clamps `l` with `l_min` to avoid numerical blow-up from `1/l`.
- It also saturates `vx`, `vy`, `wz`.
- `angular.z` is optional; here it is used to align the follower yaw to the leader yaw.
- If your topic names differ, edit the `leader_ns` / `follower_ns` parameters or modify the subscriptions directly.
