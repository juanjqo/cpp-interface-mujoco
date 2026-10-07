cmake_minimum_required(VERSION 3.16...3.28)

project(dqrobotics-interface-mujoco-examples LANGUAGES CXX)

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

set(EXAMPLES
    joint_control
    site_poses
)
foreach(example IN LISTS EXAMPLES)
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} dqrobotics-interface-mujoco)
endforeach()

# Copy the models next to the executables, so the examples can be run from the build directory
add_custom_target(copy_models ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/models ${CMAKE_BINARY_DIR}
)
