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

# MuJoCo links its dependencies (tinyxml2, qhull, ccd, ...) statically into libmujoco.so, and
# tinyxml2 marks its symbols visible: without this, libmujoco.so exports them, and the libraries of
# the same process that use their own tinyxml2 bind to MuJoCo's copy and crash (e.g., Fast DDS, in
# any ROS 2 node). --exclude-libs hides the symbols of the static libraries (those of MuJoCo itself
# are not in them, so they stay exported), but the linker ignores it for objects built with LTO,
# which MuJoCo enables: the dependencies are built without it (MuJoCo itself keeps it).
if(UNIX AND NOT APPLE AND TARGET mujoco)
    target_link_options(mujoco PRIVATE "LINKER:--exclude-libs,ALL")
    foreach(dependency IN ITEMS tinyxml2 qhullstatic_r ccd miniz tinyobjloader lodepng)
        if(TARGET ${dependency})
            set_property(TARGET ${dependency} PROPERTY INTERPROCEDURAL_OPTIMIZATION OFF)
        endif()
    endforeach()
endif()

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
    # EGL renders the cameras in HEADLESS mode (no window). Without it, cameras need WINDOW mode.
    find_package(OpenGL COMPONENTS EGL)
    if(OpenGL_EGL_FOUND)
        list(APPEND TOOL_LIBS OpenGL::EGL)
        set(DQ_MUJOCO_HAS_EGL ON)
        message(STATUS "EGL found: cameras can be rendered in HEADLESS mode")
    else()
        message(STATUS "EGL not found: cameras need WINDOW mode")
    endif()
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
