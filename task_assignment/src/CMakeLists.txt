add_executable(robot_controller_assignment
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/simulation.cpp
)

set_target_properties(robot_controller_assignment PROPERTIES 
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    CXX_STANDARD_REQUIRED ON
    COMPILE_DEFINITIONS PINOCCHIO_MODEL_DIR="${CMAKE_SOURCE_DIR}/share/robot_model"
)

target_include_directories(robot_controller_assignment PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/eigen_v3.4.0
    ${CMAKE_SOURCE_DIR}/include/pinocchio_v2.6.12
    ${CMAKE_SOURCE_DIR}/include/local
    ${CMAKE_SOURCE_DIR}/include/boost_v1.81.0
) 

target_link_libraries(robot_controller_assignment 
    stdc++fs
    ${CMAKE_SOURCE_DIR}/lib/libpinocchio.so
    ${CMAKE_SOURCE_DIR}/lib/libboost_filesystem.so
)
