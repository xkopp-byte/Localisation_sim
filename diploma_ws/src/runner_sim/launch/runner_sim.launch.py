import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Spawn formation: rows of three runners facing +X (towards the north wall at x = 11).
# Runner footprint is 1.0 m long x 0.6 m wide. Columns are 1.2 m apart, so neighbours can turn
# on the spot at the same time (turn radius ~0.58 m); rows keep a 0.3 m gap.
# The columns sit in the free lane between regal4/regal5 (y <= -0.95) and the racks at
# y >= 2.95, so every runner can drive straight forward to the wall without hitting anything.
# Seen from behind (looking towards +X):
#   X6  X4  X5   <- x = -0.7   (y = 2.2, 1.0, -0.2)
#   X3  X1  X2   <- x = -2.0
RUNNER_LENGTH = 1.0
RUNNER_WIDTH = 0.6
FORMATION_GAP = 0.3
FORMATION_ORIGIN = (-2.0, 1.0)  # X1, middle column of the first row
FORMATION_YAW = 0.0
FORMATION_SLOT_OFFSETS = (0, -1, 1)  # middle, right, left (left is +Y when facing +X)
FORMATION_COLUMN_SPACING = 1.2
FORMATION_ROW_SPACING = RUNNER_LENGTH + FORMATION_GAP


def formation_pose(index):
    row, slot = divmod(index, 3)
    x = FORMATION_ORIGIN[0] + row * FORMATION_ROW_SPACING
    y = FORMATION_ORIGIN[1] + FORMATION_SLOT_OFFSETS[slot] * FORMATION_COLUMN_SPACING
    return round(x, 3), round(y, 3)


def make_gui_after_spawns(spawns, gz_extra_args):
    """Open the Gazebo GUI only after every runner exists in the server.

    A GUI that is already running misses some runners when several are created close
    together; a GUI started afterwards loads the complete scene.
    """
    if '-s' in gz_extra_args.split():
        return []
    gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')
    remaining = {spawn for spawn in spawns}

    def on_spawn_exit(event, context):
        remaining.discard(event.action)
        return [gui] if not remaining else []

    return [RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=on_spawn_exit)) for spawn in spawns]


def make_runner_actions(context):
    count = int(LaunchConfiguration('robot_count').perform(context))
    localization_mode = LaunchConfiguration('localization_mode').perform(context)
    run_mission = LaunchConfiguration('run_mission').perform(context).lower() == 'true'
    mission_file = LaunchConfiguration('mission_file').perform(context)
    ekf_mode = LaunchConfiguration('ekf_mode').perform(context).upper()
    if ekf_mode not in ('ICP', 'IMU_ICP'):
        raise RuntimeError(f"ekf_mode must be ICP or IMU_ICP, got '{ekf_mode}'")
    model = PathJoinSubstitution([FindPackageShare('runner_sim'), 'models', 'runner.urdf.xacro'])
    actions = []
    spawns = []

    for index in range(count):
        name = f'runner_{index + 1:02d}'
        x, y = formation_pose(index)
        description = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', model, ' robot_name:=', name]),
                'use_sim_time': True,
                'frame_prefix': f'{name}/',
            }],
            output='screen',
        )
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', name,
                '-topic', f'/{name}/robot_description',
                '-x', str(x),
                '-y', str(y),
                '-z', '0.02',
                '-Y', str(FORMATION_YAW),
            ],
            output='screen',
        )
        visualization_tf = Node(
            package='runner_sim',
            executable='ground_truth_visualization_tf_node',
            namespace=name,
            name='ground_truth_visualization_tf_node',
            parameters=[{
                'use_sim_time': True,
                'robot_name': name,
                'ground_truth_topic': f'/{name}/ground_truth/odometry',
            }],
            output='screen',
        )
        localization_visualization = Node(
            package='runner_sim',
            executable='localization_visualization_node',
            namespace=name,
            name='localization_visualization_node',
            parameters=[{
                'use_sim_time': True,
                'robot_name': name,
                'ground_truth_topic': f'/{name}/ground_truth/odometry',
                'estimate_topic': f'/{name}/ekf/odometry',
                'marker_topic': '/visualization_marker_array',
                'initial_x': x,
                'initial_y': y,
                'initial_yaw': FORMATION_YAW,
                'show_covariance_ellipse': name == 'runner_01',
            }],
            output='screen',
        )
        motion_controller = Node(
            package='runner_sim',
            executable='motion_controller_node',
            namespace=name,
            name='motion_controller_node',
            parameters=[{
                'use_sim_time': True,
                'runner_number': index + 1,
                'odometry_topic': 'ground_truth/odometry',
                'command_topic': 'mission_command',
                'status_topic': 'mission_status',
                'cmd_vel_topic': 'cmd_vel',
            }],
            condition=IfCondition(str(run_mission).lower()),
            output='screen',
        )
        front_lidar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{name}_front_lidar_tf',
            arguments=[
                '--x', '0.56', '--y', '-0.36', '--z', '0.85',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', f'{name}/base_link',
                '--child-frame-id', f'{name}/base_link/front_right_lidar',
            ],
            output='screen',
        )
        rear_lidar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{name}_rear_lidar_tf',
            arguments=[
                '--x', '-0.56', '--y', '0.36', '--z', '0.85',
                '--yaw', '3.141592653589793', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', f'{name}/base_link',
                '--child-frame-id', f'{name}/base_link/rear_left_lidar',
            ],
            output='screen',
        )
        bridge_arguments = [
            f'/{name}/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ]
        localization_enabled = localization_mode == 'all' or name == 'runner_01'
        if run_mission:
            bridge_arguments.extend([
                f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{name}/wheel/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ])
        if localization_enabled:
            bridge_arguments.extend([
                f'/{name}_lidar_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{name}_lidar_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{name}/sensors/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ])

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge',
            arguments=bridge_arguments,
            output='screen',
        )
        localization = Node(
            package='runner_sim',
            executable='lidar_localization_node',
            namespace=name,
            name='lidar_localization_node',
            parameters=[{
                'use_sim_time': True,
                'robot_name': name,
                'front_scan_topic': f'/{name}_lidar_front',
                'rear_scan_topic': f'/{name}_lidar_rear',
                'fused_scan_topic': f'/{name}/lidar_fused',
                'imu_topic': f'/{name}/sensors/imu/data',
                'ekf_mode': ekf_mode,
                'icp_odom_topic': f'/{name}/icp/odometry',
                'ekf_odom_topic': f'/{name}/ekf/odometry',
            }],
            output='screen',
        )
        spawns.append(spawn)
        actions.extend([
            description, spawn, visualization_tf, localization_visualization,
            front_lidar_tf, rear_lidar_tf, bridge,
        ])
        if localization_enabled:
            actions.append(localization)
        if run_mission:
            actions.append(motion_controller)

    actions.extend(make_gui_after_spawns(spawns, LaunchConfiguration('gz_extra_args').perform(context)))

    if run_mission:
        actions.append(Node(
            package='runner_sim',
            executable='mission_manager_node',
            name='mission_manager_node',
            parameters=[{
                'use_sim_time': True,
                'mission_file': mission_file,
                'max_runner_number': count,
            }],
            output='screen',
        ))

    return actions


def generate_launch_description():
    package_share = FindPackageShare('runner_sim')
    world = PathJoinSubstitution([
        package_share,
        'worlds',
        LaunchConfiguration('world'),
    ])
    rviz_config = PathJoinSubstitution([package_share, 'config', 'runner_lidar.rviz'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        # Server only; the GUI is started by make_gui_after_spawns once all runners exist.
        launch_arguments={'gz_args': [world, ' -r -s ', LaunchConfiguration('gz_extra_args')]}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='1', description='Number of Runner instances, spawned in rows of three.'),
        DeclareLaunchArgument(
            'localization_mode', default_value='single',
            description='single: sensors and ICP/EKF only for runner_01; all: enable them for every robot.'),
        DeclareLaunchArgument(
            'run_mission', default_value='false',
            description='Start mission manager and motion controllers.'),
        DeclareLaunchArgument(
            'mission_file',
            default_value='',
            description='CSV mission file used when run_mission is true.'),
        DeclareLaunchArgument(
            'ekf_mode', default_value='ICP',
            description='ICP: EKF fuses LiDAR odometry only; IMU_ICP: LiDAR odometry + IMU gyro.'),
        DeclareLaunchArgument(
            'gz_extra_args', default_value='',
            description='Extra Gazebo arguments, e.g. "-s --headless-rendering" for a run without GUI.'),
        DeclareLaunchArgument('world', default_value='warehouse.sdf', description='World file in the runner_sim worlds directory.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz with the runner_01 LiDAR displays.'),
        gazebo,
        Node(
            package='rviz2',
            executable='rviz2',
            name='runner_lidar_rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        # Simulation time for all nodes with use_sim_time (TF, RViz, mission delays).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
        OpaqueFunction(function=make_runner_actions),
    ])
