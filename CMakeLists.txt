cmake_minimum_required(VERSION 3.5)
project(mpc-rbt-simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enforce stricter compilation rules
add_compile_options(-Wall -Wextra -Wpedantic)

# Make sure all required dependencies are locatable
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver tiago_driver_plugin.xml)

# Define include directories for the compiler
include_directories(PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# Build and install libraries
add_library(${PROJECT_NAME} SHARED
  src/TiagoDriver.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# must export include if this library is used in other projects
install(DIRECTORY include/ DESTINATION include/)

# Install additional directories and files
install(DIRECTORY
  launch
  maps
  resources
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Set up testing
#TODO(linting): enable linting tests only if we want to enforce strict ROS code formatting, otherwise they will fail
if (BUILD_TESTING)
  #    find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  #    ament_lint_auto_find_test_dependencies()
  enable_testing()

  add_subdirectory(tests)
endif ()

# ROS exports
ament_export_include_directories(include)
ament_export_dependencies(
  pluginlib
  rclcpp
  webots_ros2_driver
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_export_libraries(${PROJECT_NAME})
ament_package()
