# omni_formation_controller_pkg

ROS 2 Python package for two-robot leader-follower formation driving with:

- one shared `map_server`
- one AMCL instance per robot namespace
- one leader-follower controller using filtered odometry topics

## Included

- `leader_follower_controller`
  - subscribes to `/robot1/odometry/filtered` by default
  - subscribes to `/robot2/odometry/filtered` by default
  - publishes `/robot2/cmd_vel`

- `formation_controller.launch.py`
  - controller only

- `two_robot_formation.launch.py`
  - `nexus_base_ros/full.launch.py` for `robot1`
  - `nexus_base_ros/full.launch.py` for `robot2`
  - `nexus_base_ros/amcl_multi.launch.py` with one shared map server
  - optional controller start

## Build

```bash
colcon build --packages-select omni_formation_controller_pkg
source install/setup.bash
```

## Run

```bash
ros2 launch omni_formation_controller_pkg two_robot_formation.launch.py \
  map:=/home/hs/trash/odom_revise_cap/my_map.yaml
```

Set the initial pose of `robot1` and `robot2` manually in RViz first. Until both AMCL transforms are available, the controller will wait for TF.
