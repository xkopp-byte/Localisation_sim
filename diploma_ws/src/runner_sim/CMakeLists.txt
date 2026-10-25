cmake_minimum_required(VERSION 3.8)
project(runner_sim)

# ICP runs at the LiDAR rate; an unoptimised Eigen build is far too slow for that.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionCommand.msg"
  "msg/MissionStatus.msg"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(lidar_localization_node src/lidar_localization_node.cpp)
target_compile_features(lidar_localization_node PUBLIC cxx_std_17)
target_include_directories(lidar_localization_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(lidar_localization_node rclcpp sensor_msgs nav_msgs)
target_compile_options(lidar_localization_node PRIVATE -Wall -Wextra)

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

add_executable(mission_manager_node src/mission_manager_node.cpp)
target_compile_features(mission_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(mission_manager_node rclcpp)

add_executable(motion_controller_node src/motion_controller_node.cpp)
target_compile_features(motion_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(motion_controller_node geometry_msgs nav_msgs rclcpp)

install(TARGETS
  lidar_localization_node
  ground_truth_visualization_tf_node
  localization_visualization_node
  mission_manager_node
  motion_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(lidar_localization_node "${cpp_typesupport_target}")
target_link_libraries(mission_manager_node "${cpp_typesupport_target}")
target_link_libraries(motion_controller_node "${cpp_typesupport_target}")

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY missions
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
