cmake_minimum_required(VERSION  3.16...3.28)

project(test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
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

add_executable(test main.cpp)
target_link_libraries(test dqrobotics-interface-mujoco ${MUJOCO_LIBS})

set(XML_MODEL ${CMAKE_SOURCE_DIR}/../universal_robots_ur3e/)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory #copy_if_different
        ${XML_MODEL}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)
