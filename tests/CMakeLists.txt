ament_add_gtest(tiagoDriverTest tiagoDriverTest.cpp
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        APPEND_LIBRARY_DIRS ${PROJECT_NAME}
        TIMEOUT 900
)
target_link_libraries(tiagoDriverTest ${PROJECT_NAME})

ament_add_gtest(localizationTest localizationTest.cpp
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        APPEND_LIBRARY_DIRS ${PROJECT_NAME}-solutions
        TIMEOUT 900
)
target_link_libraries(localizationTest ${PROJECT_NAME}-solutions)
