cmake_minimum_required(VERSION 3.8)
project(turtlebot3_multi_robot)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Generate custom interface messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# Build Nav2 ORCA Controller Plugin Library
add_library(nav2_orca_planner SHARED
  src/orca_local_planner.cpp
)

target_include_directories(nav2_orca_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_orca_planner
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

# Link generated message target to the C++ library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(nav2_orca_planner "${cpp_typesupport_target}")

# Build IsTopicActive BT Plugin
add_library(nav2_is_topic_active_condition_bt_node SHARED
  src/is_topic_active_condition.cpp
)
target_include_directories(nav2_is_topic_active_condition_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nav2_is_topic_active_condition_bt_node
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

# Pluginlib registration
pluginlib_export_plugin_description_file(nav2_core plugins/nav2_orca_planner_plugin.xml)

# Install Python scripts
install(PROGRAMS
  nodes/orca_p2p_node.py
  nodes/baseline_p2p_node.py
  nodes/benchmark_suite.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ Libraries and Headers
install(TARGETS nav2_orca_planner nav2_is_topic_active_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install Directories
install(DIRECTORY
  launch
  config
  params
  behavior_trees
  map
  rviz
  worlds
  models
  urdf
  plugins
  web
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(nav2_orca_planner nav2_is_topic_active_condition_bt_node)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  std_msgs
  behaviortree_cpp_v3
)

ament_package()
