# Runner simulation

This package contains the first simulation model for the diploma localisation study.

The default world is a small warehouse with walls and shelves. Use `rviz:=true` to open a 2D RViz view of `/runner_01_lidar_front` and `/runner_01_lidar_rear`.

The default Runner is a tall rectangular chassis:

- width: 0.6 m (Y)
- length: 1.0 m (X)
- height: 1.5 m (Z)
- front-right LiDAR and rear-left LiDAR: 1.6 m above ground, outside the chassis volume
- maximum linear velocity: 2.0 m/s
- maximum angular velocity: pi/2 rad/s

Each model is spawned with a unique name such as `runner_01`. Sensor measurements are
namespaced below that model name. Ground truth is published separately as
`/<runner_name>/ground_truth/odometry` and is not used by the sensor plugins.

## LiDAR ICP and EKF

The package also builds `lidar_localization_node`. For each robot the launch
file starts one node that consumes the front LiDAR and wheel odometry. It keeps
the previous scan as a local reference, estimates the scan-to-scan rigid
transform with 2D ICP, and fuses the resulting pose measurement with the
nonlinear differential-drive model using an EKF. The node publishes:

- `/<runner_name>/icp/odometry` - ICP-only local pose estimate
- `/<runner_name>/ekf/odometry` - EKF pose estimate
- `/<runner_name>/lidar_fused` - front and rear LiDAR fused in the robot `base_link` frame

The front and rear scans are transformed using their calibrated positions and
orientations, then merged into one 360-degree scan centered at the robot. ICP
and EKF use this fused scan. The original front and rear topics remain
available for comparison.

The initial pose is `(0, 0, 0)` in a local `<runner_name>/odom` frame. Ground
truth is not subscribed to by the localization node. This scan-to-scan reference
is an initial implementation surface; a known map can later replace the
previous scan without changing the EKF interface.

The launch default is `localization_mode:=single`: full sensor bridges and
ICP/EKF run only for `runner_01`, while every other robot publishes only its
ground-truth odometry. Use `localization_mode:=all` to restore sensor bridges
and localization nodes for every robot.

## Build

```bash
cd ~/Diploma/diploma_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select runner_sim
source install/setup.bash
```

## Run one Runner

```bash
ros2 launch runner_sim runner_sim.launch.py robot_count:=1
```

```bash
ros2 launch runner_sim runner_sim.launch.py robot_count:=1 rviz:=true
```

The RViz configuration contains only the fused displays `lidar_runner_01`
through `lidar_runner_06`. With `localization_mode:=all`, the fused scans for
all requested robots are available:

```bash
ros2 launch runner_sim runner_sim.launch.py \
	robot_count:=6 localization_mode:=all rviz:=true
```

RViz uses the `world` fixed frame. A separate visualization-only node converts
ground-truth odometry into `world -> runner_XX/base_link` TF transforms so
fused scans from multiple robots can be displayed together. Ground truth is
not used as an input to ICP or EKF.

All robots publish their overlays together on `/visualization_marker_array` as
a `visualization_msgs/msg/MarkerArray`. The ground-truth footprint and center are
green, while the EKF estimate is red. The marker node is visualization-only;
its ground-truth subscription does not feed the localization algorithm.

Use `world:=runner_world.sdf` to run the empty-plane world instead.

The wheel dimensions and exact sensor characteristics are intentionally centralized
in `models/runner.urdf.xacro` so measurements from the real robot can replace the
initial assumptions without changing the launch architecture.
