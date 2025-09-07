cmake_minimum_required(VERSION  3.16...3.28)

project(test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
include(../dependencies.cmake)


add_executable(test main.cpp )


target_link_libraries(test PRIVATE
    ${MUJOCO_LIBS}
)


set(XML_MODEL ${CMAKE_SOURCE_DIR}/../example.xml)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${XML_MODEL}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)
