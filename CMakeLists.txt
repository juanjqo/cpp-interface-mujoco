#---------------------------------------------------------------------------------------------------
# Build and install (run from the repository root)
#
#   1. Configure (Release is the default; /usr/local is the default install prefix)
#        cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
#      To build only the library, add: -DDQ_MUJOCO_BUILD_EXAMPLES=OFF -DDQ_MUJOCO_BUILD_TESTS=OFF
#
#   2. Build the library, examples and tests
#        cmake --build build -j$(nproc)
#
#   3. (Optional) Run the tests
#        ctest --test-dir build --output-on-failure
#
#   4. Install the library, its header and MuJoCo, then refresh the linker cache
#        sudo cmake --install build
#        sudo ldconfig
#
# Then, in another CMake project:
#   find_package(dqrobotics-interface-mujoco REQUIRED)
#   target_link_libraries(my_app PRIVATE dqrobotics_extensions::interface-mujoco)
#---------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.28)
# https://cmake.org/cmake/help/latest/variable/BUILD_SHARED_LIBS.html
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

if(WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics-interface-mujoco
        VERSION 1.0
        DESCRIPTION "An unofficial DQ Robotics interface for C++17 to interact with Mujoco."
        HOMEPAGE_URL "https://github.com/juanjqo/cpp-interface-mujoco"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# strongly encouraged to enable this globally to avoid conflicts between
# -Wpedantic being enabled and -std=c++20 and -std=gnu++20 for example
# when compiling with PCH enabled
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to an optimized build (-O3 -DNDEBUG) when no build type is given
if(PROJECT_IS_TOP_LEVEL AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(DQ_MUJOCO_BUILD_EXAMPLES "Build the examples" ${PROJECT_IS_TOP_LEVEL})
option(DQ_MUJOCO_BUILD_TESTS "Build the tests" ${PROJECT_IS_TOP_LEVEL})

include(dependencies.cmake)
include(GNUInstallDirs)

#---------------------------------------------------------------------------------------------------
# Library
#---------------------------------------------------------------------------------------------------
add_library(${PROJECT_NAME}
    src/dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.cpp
)
add_library(dqrobotics_extensions::interface-mujoco ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME interface-mujoco)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# The public header only exposes DQ Robotics (and Eigen through it).
# MuJoCo, GLFW and OpenGL are implementation details.
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    dqrobotics
    Eigen3::Eigen
    PRIVATE
    mujoco::mujoco
    ${TOOL_LIBS}
)

# Find the MuJoCo library installed next to this one
if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "@loader_path")
elseif(UNIX)
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

#---------------------------------------------------------------------------------------------------
# Install: provides find_package(dqrobotics-interface-mujoco) and the target dqrobotics_extensions::interface-mujoco
#---------------------------------------------------------------------------------------------------
include(CMakePackageConfigHelpers)
set(CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE dqrobotics_extensions::
    DESTINATION ${CONFIG_INSTALL_DIR}
)

configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CONFIG_INSTALL_DIR}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CONFIG_INSTALL_DIR}
)

#---------------------------------------------------------------------------------------------------
# Examples and tests
#---------------------------------------------------------------------------------------------------
if(DQ_MUJOCO_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(DQ_MUJOCO_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
