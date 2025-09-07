cmake_minimum_required(VERSION  3.16...3.28)

project(test2 LANGUAGES CXX)

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
    ${MUJOCO_LIBS}
)

add_executable(test2 main.cpp)
target_link_libraries(test2 dqrobotics-interface-mujoco ${MUJOCO_LIBS})

set(XML_MODEL ${CMAKE_SOURCE_DIR}/../unitree_z1/)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory #copy_if_different
        ${XML_MODEL}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)
