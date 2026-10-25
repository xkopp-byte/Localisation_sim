# Runner simulation

This package contains the first simulation model for the diploma localisation study.

The default world is a small warehouse with walls and shelves. Use `rviz:=true` to open a 2D RViz view of `/runner_01_lidar_front` and `/runner_01_lidar_rear`.

The default Runner is a tall rectangular chassis:

- width: 0.6 m (Y)
- length: 1.0 m (X)
- height: 1.5 m (Z)
- front-right LiDAR and rear-left LiDAR: 0.85 m above ground (below the 1.5 m body, so other runners are visible), outside the chassis volume
- maximum linear velocity: 2.0 m/s
- maximum angular velocity: pi/2 rad/s

Each model is spawned with a unique name such as `runner_01`. Sensor measurements are
namespaced below that model name. Ground truth is published separately as
`/<runner_name>/ground_truth/odometry` and is not used by the sensor plugins.

## LiDAR ICP and EKF

`lidar_localization_node` fuses the front and rear LiDAR into one 360-degree
scan in `base_link` (points on the robot's own body are removed), estimates the
scan-to-scan motion with point-to-line ICP and fuses it in an EKF with state
`[x, y, yaw, vx, vy, wz]`. The EKF mode is selected with `ekf_mode`:

- `ICP` - LiDAR odometry only
- `IMU_ICP` - LiDAR odometry + IMU gyro

Wheel odometry is not used by the filter. Outputs:

- `/<runner_name>/icp/odometry` - integrated ICP odometry
- `/<runner_name>/ekf/odometry` - EKF estimate with pose and twist covariance
- `/<runner_name>/lidar_fused` - fused scan

Runners are driven from ground truth by `motion_controller_node`; the EKF never
steers them. Mission CSV coordinates are in the `world` frame. `runner_01`'s
95 % position-uncertainty ellipse is shown in RViz (`/visualization_marker_array`).
See `mission_03_baseline.csv` and `mission_03_interference.csv` for the
multi-runner experiment.

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

## Mission movement

Mission execution is disabled by default. The helper script shows an arrow-key
menu containing `No mission` and all CSV files in `missions/`. Select an entry
with Up/Down and press Enter.

To run the included mission for three robots directly:

```bash
ros2 launch runner_sim runner_sim.launch.py \
	robot_count:=3 localization_mode:=all run_mission:=true \
	mission_file:=/absolute/path/to/mission_01.csv
```

The CSV is installed at `missions/mission_01.csv` and uses global target
positions:

```csv
runner_number,x_m,y_m,circle_radius_m,speed_percent,hard_point
1,-2,2,0,50,true
1,-2,3,0,50,true
```

`x_m` and `y_m` are absolute coordinates in the world frame. For a linear
command the runner first rotates toward the target and then drives straight to
it. `speed_percent` is a percentage of the configured maximum linear speed.
`hard_point` is explicit; `true` requires the runner to stop at the point before
the next command. A positive or negative `circle_radius_m` requests continuous
circular motion in the corresponding direction. Circular commands are rejected
if their endpoint is not compatible with the requested radius.

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
