enable_testing()

# Helper function to create a test target
function(add_robot_test TEST_NAME TEST_SRC)
    add_executable(${TEST_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_SRC}
        ${ARGN}  # extra sources like simulation.cpp, demo.cpp, controller.cpp
    )

    set_target_properties(${TEST_NAME} PROPERTIES 
        CXX_STANDARD 17
        CXX_EXTENSIONS OFF
        CXX_STANDARD_REQUIRED ON
        COMPILE_DEFINITIONS PINOCCHIO_MODEL_DIR="${CMAKE_SOURCE_DIR}/share/robot_model"
    )

    target_include_directories(${TEST_NAME} PUBLIC
        ${CMAKE_SOURCE_DIR}/include/eigen_v3.4.0
        ${CMAKE_SOURCE_DIR}/include/pinocchio_v2.6.12
        ${CMAKE_SOURCE_DIR}/include/local
        ${CMAKE_SOURCE_DIR}/include/boost_v1.81.0
    )

    target_link_libraries(${TEST_NAME}
        stdc++fs
        ${CMAKE_SOURCE_DIR}/lib/libpinocchio.so
        ${CMAKE_SOURCE_DIR}/lib/libboost_filesystem.so
    )

    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

# --- Define tests ---
add_robot_test(test_robot_state test_robot_state.cpp
    ${CMAKE_SOURCE_DIR}/src/simulation.cpp
)

add_robot_test(test_controllers test_controllers.cpp
    ${CMAKE_SOURCE_DIR}/src/simulation.cpp
)

add_robot_test(test_demos test_demos.cpp
    ${CMAKE_SOURCE_DIR}/src/simulation.cpp
    ${CMAKE_SOURCE_DIR}/src/demo.cpp
# )

add_robot_test(robot_test test_robot.cpp
    ${CMAKE_SOURCE_DIR}/src/simulation.cpp
    ${CMAKE_SOURCE_DIR}/src/demo.cpp
)

# You can add more tests similarly