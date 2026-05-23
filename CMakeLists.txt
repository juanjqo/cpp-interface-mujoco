cmake_minimum_required(VERSION 3.22...3.28)
# https://cmake.org/cmake/help/latest/variable/BUILD_SHARED_LIBS.html
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics-interface-mujoco
        VERSION 1.0
        DESCRIPTION "An unofficial DQ Robotics interface for C++17 to interact with Mujoco."
        HOMEPAGE_URL "https://github.com/juanjqo/cpp-interface-mujoco"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(dependencies.cmake)

# strongly encouraged to enable this globally to avoid conflicts between
# -Wpedantic being enabled and -std=c++20 and -std=gnu++20 for example
# when compiling with PCH enabled
set(CMAKE_CXX_EXTENSIONS OFF)

# Apply -O3 for Release builds
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_library(${PROJECT_NAME} 
    src/dqrobotics/interfaces/mujoco/DQ_MujocoInterface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${MUJOCO_INCLUDE_DIR}  # Changed from MUJOCO_LIBS to MUJOCO_INCLUDE_DIR
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${TOOL_LIBS}
)
