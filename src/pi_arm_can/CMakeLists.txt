cmake_minimum_required(VERSION 3.16)
project(pi_arm_can LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

add_library(pi_arm_can SHARED
  src/can_driver.cpp
  src/protocol.cpp
  src/transmission.cpp
)
add_library(pi_arm_can::pi_arm_can ALIAS pi_arm_can)
target_include_directories(pi_arm_can
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_compile_options(pi_arm_can PRIVATE -Wall -Wextra -Wpedantic)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS pi_arm_can
  EXPORT export_pi_arm_can
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pi_arm_can test/test_pi_arm_can.cpp)
  if(TARGET test_pi_arm_can)
    target_link_libraries(test_pi_arm_can pi_arm_can)
    target_compile_options(test_pi_arm_can PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_export_targets(export_pi_arm_can HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(pi_arm_can)
ament_package()
