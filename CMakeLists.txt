#---------------------------------------------------------------------------------------------------
# Build and install (run from the repository root)
#
#   1. Configure (Release is the default; /usr/local is the default install prefix)
#        cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
#      To build only the library, add: -DDQ_MUJOCO_BUILD_EXAMPLES=OFF -DDQ_MUJOCO_BUILD_TESTS=OFF
#      To skip the robots library (embedded robot models), add: -DDQ_MUJOCO_BUILD_ROBOTS=OFF
#      To build without the GUI of the viewer (Dear ImGui), add: -DDQ_MUJOCO_BUILD_GUI=OFF
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
# Without sudo: use -DCMAKE_INSTALL_PREFIX=$HOME/opt in step 1, run step 4 as "cmake --install build"
# (no sudo, no ldconfig), and configure other projects with -DCMAKE_PREFIX_PATH=$HOME/opt.
#
# Then, in another CMake project:
#   find_package(dqrobotics-interface-mujoco REQUIRED)
#   target_link_libraries(my_app PRIVATE dqrobotics_extensions::interface-mujoco)
# and, to use the generic robot models (e.g., GenericQuadrupedRobot::description()):
#   target_link_libraries(my_app PRIVATE dqrobotics_extensions::interface-mujoco-generic-robots)
# The package also provides dq_mujoco_embed_model(), to embed models in a target. Models of real
# robots are in cpp-interface-mujoco-robots (https://github.com/juanjqo/cpp-interface-mujoco-robots).
#---------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.28)
# https://cmake.org/cmake/help/latest/variable/BUILD_SHARED_LIBS.html
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

if(WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics-interface-mujoco
        VERSION 1.1
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
option(DQ_MUJOCO_BUILD_ROBOTS "Build the generic robots library (generic robot models embedded in the library)" ON)
option(DQ_MUJOCO_BUILD_GUI "Build the GUI of the viewer (Dear ImGui, built into the library)" ON)

include(dependencies.cmake)
include(GNUInstallDirs)

#---------------------------------------------------------------------------------------------------
# Library
#---------------------------------------------------------------------------------------------------
add_library(${PROJECT_NAME}
    src/dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.cpp
    src/dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.cpp
    src/dqrobotics_extensions/interfaces/mujoco/DQ_MujocoCameras.cpp
)
add_library(dqrobotics_extensions::interface-mujoco ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME interface-mujoco)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# The public header only exposes DQ Robotics (and Eigen through it).
# MuJoCo, sas, GLFW and OpenGL are implementation details.
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    dqrobotics
    Eigen3::Eigen
    PRIVATE
    mujoco::mujoco
    marinholab::sas::core
    ${TOOL_LIBS}
)

if(DQ_MUJOCO_HAS_EGL)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DQ_MUJOCO_HAS_EGL)
endif()

# GUI of the viewer (set_gui_enabled()). Dear ImGui is an implementation detail, like MuJoCo's
# tinyxml2: its sources are built into this library (a separate static target would have to be
# exported in static builds), and its symbols are hidden, so that the programs that load this
# library and use their own ImGui do not bind to this copy. Each viewer thread has its own ImGui
# context (dq_mujoco_imconfig.h). Without the GUI, the public API is the same.
if(DQ_MUJOCO_BUILD_GUI)
    set(IMGUI_SOURCES
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    set(GUI_SOURCES
        src/dqrobotics_extensions/interfaces/mujoco/DQ_MujocoGui.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/generated/dq_mujoco_gui_font.cpp
    )
    # The font of the GUI (Roboto, in ImGui's sources), embedded as an array
    set(GUI_FONT ${imgui_SOURCE_DIR}/misc/fonts/Roboto-Medium.ttf)
    file(READ ${GUI_FONT} font_hex HEX)
    string(REPEAT "[0-9a-f]" 32 line_pattern)  # 16 bytes per line
    string(REGEX REPLACE "(${line_pattern})" "\\1\n" font_hex "${font_hex}")
    string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," font_hex "${font_hex}")
    file(SIZE ${GUI_FONT} font_size)
    file(CONFIGURE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated/dq_mujoco_gui_font.cpp
         CONTENT "// Generated by CMakeLists.txt from Roboto-Medium.ttf (Apache-2.0)\nextern const unsigned char dq_mujoco_gui_font[];\nextern const unsigned int dq_mujoco_gui_font_size;\nconst unsigned char dq_mujoco_gui_font[] = {\n${font_hex}};\nconst unsigned int dq_mujoco_gui_font_size = ${font_size};\n"
         @ONLY)
    unset(font_hex)

    target_sources(${PROJECT_NAME} PRIVATE ${IMGUI_SOURCES} ${GUI_SOURCES})
    if(NOT WIN32)
        set_source_files_properties(${IMGUI_SOURCES} PROPERTIES COMPILE_OPTIONS "-fvisibility=hidden;-w")
        set_source_files_properties(${GUI_SOURCES} PROPERTIES COMPILE_OPTIONS "-fvisibility=hidden")
    endif()
    target_include_directories(${PROJECT_NAME} PRIVATE
        $<BUILD_INTERFACE:${imgui_SOURCE_DIR}>
        $<BUILD_INTERFACE:${imgui_SOURCE_DIR}/backends>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/dqrobotics_extensions/interfaces/mujoco>
    )
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        DQ_MUJOCO_HAS_GUI
        IMGUI_USER_CONFIG="dq_mujoco_imconfig.h"
    )
endif()

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

# The licenses, with the documentation: this library's (LICENSE, the LGPL-3.0, and COPYING, the GPL-3.0
# that it supplements), and those of MuJoCo (Apache-2.0), which is installed with this library, and
# of the libraries built into it. Licenses of sources that were not fetched (e.g., a MuJoCo found on
# the system) are skipped.
install(FILES LICENSE COPYING DESTINATION ${CMAKE_INSTALL_DOCDIR})
set(BUNDLED_LICENSES  # <fetched dependency> <its license file>
    mujoco          LICENSE
    ccd             BSD-LICENSE
    lodepng         LICENSE
    marchingcubecpp README.md      # public domain or MIT, as stated in its README
    miniz           LICENSE
    qhull           COPYING.txt
    tinyobjloader   LICENSE
    tinyxml2        LICENSE.txt
    imgui           LICENSE.txt)
while(BUNDLED_LICENSES)
    list(POP_FRONT BUNDLED_LICENSES dependency license_file)
    FetchContent_GetProperties(${dependency} SOURCE_DIR source_dir)
    if(source_dir AND EXISTS ${source_dir}/${license_file})
        install(FILES ${source_dir}/${license_file} DESTINATION ${CMAKE_INSTALL_DOCDIR}/third_party/${dependency})
    endif()
endwhile()
# The font of the GUI (Roboto, from ImGui's sources) is embedded in the library
if(DQ_MUJOCO_BUILD_GUI)
    install(FILES cmake/third_party/roboto/NOTICE DESTINATION ${CMAKE_INSTALL_DOCDIR}/third_party/roboto)
endif()

# Generic robots library: dqrobotics_extensions::interface-mujoco-generic-robots (same export set)
if(DQ_MUJOCO_BUILD_ROBOTS)
    add_subdirectory(robots)
endif()

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
# The package also provides dq_mujoco_embed_model(), to embed models in other libraries (e.g., the
# robots of cpp-interface-mujoco-robots) and programs
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    cmake/DQMujocoEmbedModel.cmake
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
