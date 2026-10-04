from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def make_runner_actions(context):
    count = int(LaunchConfiguration('robot_count').perform(context))
    x_spacing = float(LaunchConfiguration('spawn_spacing').perform(context))
    localization_mode = LaunchConfiguration('localization_mode').perform(context)
    model = PathJoinSubstitution([FindPackageShare('runner_sim'), 'models', 'runner.urdf.xacro'])
    actions = []

    for index in range(count):
        name = f'runner_{index + 1:02d}'
        x = (index - (count - 1) / 2.0) * x_spacing
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
                '-y', '0.0',
                '-z', '0.02',
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
                'initial_y': 0.0,
                'initial_yaw': 0.0,
            }],
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
        if localization_enabled:
            bridge_arguments = [
                f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{name}_lidar_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{name}_lidar_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{name}/sensors/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'/{name}/wheel/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                *bridge_arguments,
            ]

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
                'wheel_odom_topic': f'/{name}/wheel/odometry',
                'icp_odom_topic': f'/{name}/icp/odometry',
                'ekf_odom_topic': f'/{name}/ekf/odometry',
            }],
            output='screen',
        )
        actions.extend([
            description, spawn, visualization_tf, localization_visualization,
            front_lidar_tf, rear_lidar_tf, bridge,
        ])
        if localization_enabled:
            actions.append(localization)

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
        launch_arguments={'gz_args': [world, ' -r']}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='1', description='Number of Runner instances (1 to 6).'),
        DeclareLaunchArgument('spawn_spacing', default_value='2.0', description='X spacing between initial robot poses.'),
        DeclareLaunchArgument(
            'localization_mode', default_value='single',
            description='single: sensors and ICP/EKF only for runner_01; all: enable them for every robot.'),
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
        OpaqueFunction(function=make_runner_actions),
    ])
