if(UNIX AND NOT APPLE)
    add_compile_options(-Wno-format-truncation)
endif()

include(FetchContent)

# Only the MuJoCo library is needed: skip its own tests, samples and simulate app
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)

set(MUJOCO_VERSION "3.16.0")

# Prevent MuJoCo from creating Eigen3::Eigen target
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "Disable Python util to prevent Eigen3 target" FORCE)

FetchContent_Declare(
    mujoco
    URL https://github.com/google-deepmind/mujoco/archive/refs/tags/${MUJOCO_VERSION}.tar.gz
)

FetchContent_MakeAvailable(mujoco)

set(MUJOCO_INCLUDE_DIR ${mujoco_SOURCE_DIR}/include)
set(MUJOCO_LIB_DIR ${mujoco_SOURCE_DIR}/lib)

if(UNIX AND NOT APPLE)
    find_package(OpenGL REQUIRED)
    find_package(Eigen3 REQUIRED)  # Find it here after MuJoCo to avoid conflict
    set(TOOL_LIBS
        glfw
        OpenGL::GL
        Eigen3::Eigen
    )
    add_compile_options(-Wno-format-truncation
        -Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual -fPIC)
endif()

if(APPLE)
    find_package(Eigen3 REQUIRED)  # Find it here after MuJoCo
    include_directories(
        /usr/local/include/
        /opt/homebrew/include
        /opt/homebrew/include/GLFW
        ${mujoco_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIRS}
    )
    add_compile_options(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
    link_directories(/usr/local/lib/ /opt/homebrew/lib)
    set(TOOL_LIBS
        "-framework Cocoa"
        "-framework IOKit"
        "-framework CoreVideo"
        "-framework CoreFoundation"
        "-framework OpenGL"
        glfw
        Eigen3::Eigen
    )
endif()

# sas (https://github.com/MarinhoLab/sas_cpp): the ThreadManager runs the physics and viewer threads.
# Use the installed package (libmarinholab-sas-core) if found, otherwise build it from source.
find_package(marinholab_sas_core QUIET)
if(NOT marinholab_sas_core_FOUND)
    message(STATUS "marinholab_sas_core not found: fetching sas_cpp")
    set(MARINHO_LAB_SAS_CORE_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        sas_cpp
        GIT_REPOSITORY https://github.com/MarinhoLab/sas_cpp.git
        GIT_TAG 78cf20cc1753bd5bd63d32f02c76f9b82192e404
    )
    FetchContent_MakeAvailable(sas_cpp)
endif()
