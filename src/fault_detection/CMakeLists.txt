cmake_minimum_required(VERSION 3.5)
project(fault_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)

# Uncomment if the package has a setup.py. This macro ensures modules and global scripts get installed
# ament_python_install_package(${PROJECT_NAME})

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${interfaces_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(DroneInfoCompiler src/DroneInfoCompiler.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(DroneInfoCompiler
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  interfaces
)

install(TARGETS
  DroneInfoCompiler
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
