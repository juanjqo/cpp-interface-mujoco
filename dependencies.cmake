
if(UNIX AND NOT APPLE)
    #This is required on Ubuntu 24.04 otherwise Mujoco does not compile.
    add_compile_options(-Wno-format-truncation)
endif()

include(FetchContent)

# Pick your MuJoCo version
set(MUJOCO_VERSION "3.8.1")

# Download MuJoCo release from GitHub
FetchContent_Declare(
  mujoco
  #URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-macos-arm64.tar.gz
  URL https://github.com/google-deepmind/mujoco/archive/refs/tags/${MUJOCO_VERSION}.tar.gz
)

FetchContent_MakeAvailable(mujoco)

# MuJoCo gets unpacked into ${mujoco_SOURCE_DIR}
set(MUJOCO_INCLUDE_DIR ${mujoco_SOURCE_DIR}/include)
set(MUJOCO_LIB_DIR     ${mujoco_SOURCE_DIR}/lib)


if(UNIX AND NOT APPLE)
    find_package(OpenGL REQUIRED)
    find_package(Eigen3 REQUIRED)
    set(TOOL_LIBS
        glfw
        OpenGL::GL
        Eigen3::Eigen
    )
add_compile_options(-Wno-format-truncation
    -Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual -fPIC)
endif()

if(APPLE)
    include_directories(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3
        /opt/howebrew/include/fmt
        /opt/howebrew/include/GLFW   #OPENGL 3
        #/opt/howebrew/include/SDL2   #METAL
        #/System/Library/Frameworks
        ${mujoco_SOURCE_DIR}/include
    )
add_compile_options(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
# The library is installed here when using the regular cmake ., make, sudo make install
link_directories(
    /usr/local/lib/
    /opt/homebrew/lib
    #${mujoco_SOURCE_DIR}/lib
    )

set(TOOL_LIBS
     "-framework Cocoa"
     "-framework IOKit"
     "-framework CoreVideo"
     "-framework CoreFoundation"
     "-framework OpenGL"
     glfw
)

endif()




