# Add test executable
add_executable(test_robot
    ${CMAKE_CURRENT_SOURCE_DIR}/test_robot.cpp
    ${CMAKE_SOURCE_DIR}/src/demo.cpp
    ${CMAKE_SOURCE_DIR}/src/simulation.cpp
)

set_target_properties(test_robot PROPERTIES 
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    CXX_STANDARD_REQUIRED ON
    COMPILE_DEFINITIONS PINOCCHIO_MODEL_DIR="${CMAKE_SOURCE_DIR}/share/robot_model"
)

# Include directories (same as your main project)
target_include_directories(test_robot PUBLIC
    ${CMAKE_SOURCE_DIR}/include/eigen_v3.4.0
    ${CMAKE_SOURCE_DIR}/include/pinocchio_v2.6.12
    ${CMAKE_SOURCE_DIR}/include/local
    ${CMAKE_SOURCE_DIR}/include/boost_v1.81.0
)

# Link libraries
target_link_libraries(test_robot
    stdc++fs
    ${CMAKE_SOURCE_DIR}/lib/libpinocchio.so
    ${CMAKE_SOURCE_DIR}/lib/libboost_filesystem.so
)

# Enable testing
enable_testing()

# Register the test
add_test(NAME robot_test COMMAND test_robot)
