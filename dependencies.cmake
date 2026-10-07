if(UNIX AND NOT APPLE)
    add_compile_options(-Wno-format-truncation)
endif()

include(FetchContent)

set(MUJOCO_VERSION "3.9.0")

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



