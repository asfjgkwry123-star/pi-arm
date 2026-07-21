cmake_minimum_required(VERSION 3.8)
project(pi_arm_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pi_arm_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(pi_arm_manager_node src/manager_node.cpp)
target_include_directories(pi_arm_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pi_arm_manager_node
  control_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning_interface pi_arm_interfaces
  rclcpp rclcpp_action trajectory_msgs)

install(TARGETS pi_arm_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_state_aggregator test/test_state_aggregator.cpp)
  target_include_directories(test_state_aggregator PRIVATE include)
  ament_target_dependencies(test_state_aggregator pi_arm_interfaces)
endif()

ament_export_include_directories(include)
ament_package()
