if (webots_ros2_driver_VERSION VERSION_LESS "2025.0.0")
  add_compile_definitions(WEBOTS_NODE_OLD_API)
endif()

ament_add_gtest(tiagoDriverTest tiagoDriverTest.cpp
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  APPEND_LIBRARY_DIRS ${PROJECT_NAME}
  TIMEOUT 900
)
target_link_libraries(tiagoDriverTest ${PROJECT_NAME})
