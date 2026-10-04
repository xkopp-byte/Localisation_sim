cmake_minimum_required(VERSION 3.8)
project(runner_sim)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(lidar_localization_node src/lidar_localization_node.cpp)
target_compile_features(lidar_localization_node PUBLIC cxx_std_17)
target_include_directories(lidar_localization_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(lidar_localization_node rclcpp sensor_msgs nav_msgs)

add_executable(ground_truth_visualization_tf_node src/ground_truth_visualization_tf_node.cpp)
target_compile_features(ground_truth_visualization_tf_node PUBLIC cxx_std_17)
ament_target_dependencies(
  ground_truth_visualization_tf_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(localization_visualization_node src/localization_visualization_node.cpp)
target_compile_features(localization_visualization_node PUBLIC cxx_std_17)
ament_target_dependencies(
  localization_visualization_node
  geometry_msgs
  nav_msgs
  rclcpp
  visualization_msgs
)

install(TARGETS
  lidar_localization_node
  ground_truth_visualization_tf_node
  localization_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
