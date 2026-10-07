cmake_minimum_required(VERSION 3.16...3.28)

project(dqrobotics-interface-mujoco-tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Only the MuJoCo library is needed: skip its own tests, samples and simulate app
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
include(../dependencies.cmake)

include_directories(${CMAKE_SOURCE_DIR}/../include/)
add_library(dqrobotics-interface-mujoco
    ${CMAKE_SOURCE_DIR}/../src/dqrobotics/interfaces/mujoco/DQ_MujocoInterface.cpp
)
target_link_libraries(dqrobotics-interface-mujoco
    dqrobotics
    mujoco::mujoco
    ${TOOL_LIBS}
)

add_executable(test_interface test_interface.cpp)
target_link_libraries(test_interface dqrobotics-interface-mujoco)

# Each test runs in its own process, so a crash in one does not hide the others
enable_testing()
set(TESTS
    joint_positions_readable_after_set
    size_mismatch_throws
    site_follows_moving_body
    set_site_pose_on_worldbody_persists
    set_site_pose_on_moving_body
    joint_force_with_free_joint
    actuator_lookup_ignores_tendon_actuators
    load_error_includes_mujoco_message
    reset_restores_joints_and_sites
    load_model_twice
    destroy_without_loading
    two_interfaces_at_once
)
foreach(test_name IN LISTS TESTS)
    add_test(NAME ${test_name}
             COMMAND test_interface ${test_name}
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/models)
endforeach()

# Undefined behavior in the destructor does not always crash, so also check it with valgrind
find_program(VALGRIND valgrind)
if(VALGRIND)
    add_test(NAME destroy_without_loading_valgrind
             COMMAND ${VALGRIND} -q --error-exitcode=1 $<TARGET_FILE:test_interface> destroy_without_loading
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/models)
endif()
