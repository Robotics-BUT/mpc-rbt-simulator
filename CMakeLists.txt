cmake_minimum_required(VERSION 3.5)
project(mpc-rbt-simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver tiago_driver_plugin.xml)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/TiagoDriver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

include_directories(include)

add_executable(localization src/Localization.cpp)
ament_target_dependencies(localization rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS
  localization
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  maps
  resources
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()