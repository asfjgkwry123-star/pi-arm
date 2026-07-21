cmake_minimum_required(VERSION 3.16)
project(pi_arm_hardware LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pi_arm_can REQUIRED)
find_package(pi_arm_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(pi_arm_hardware SHARED
  src/backend.cpp
  src/pi_arm_system.cpp
)
target_include_directories(pi_arm_hardware
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(pi_arm_hardware
  hardware_interface
  pi_arm_can
  pi_arm_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_compile_options(pi_arm_hardware PRIVATE -Wall -Wextra -Wpedantic)

pluginlib_export_plugin_description_file(hardware_interface pi_arm_hardware_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS pi_arm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_backend test/test_backend.cpp)
  if(TARGET test_backend)
    target_link_libraries(test_backend pi_arm_hardware)
    ament_target_dependencies(test_backend pi_arm_can)
    target_compile_options(test_backend PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(pi_arm_hardware)
ament_export_dependencies(hardware_interface pi_arm_can pi_arm_interfaces pluginlib rclcpp)
ament_package()
