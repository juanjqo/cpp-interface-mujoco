cmake_minimum_required(VERSION  3.5...3.28)

project(test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Pick your MuJoCo version
set(MUJOCO_VERSION "3.3.5")

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



if(APPLE)

    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3
        /opt/howebrew/include/fmt
        /opt/howebrew/include/GLFW   #OPENGL 3
        #/opt/howebrew/include/SDL2   #METAL
        #/System/Library/Frameworks
    )
ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
# The library is installed here when using the regular cmake ., make, sudo make install
LINK_DIRECTORIES(
    /usr/local/lib/
    /opt/homebrew/lib
    )
endif()


include(CMakePrintHelpers)

#include_directories(MUJOCO_INCLUDE_DIR)

cmake_print_variables(MUJOCO_INCLUDE_DIR)

add_executable(test main.cpp )


target_link_libraries(test PRIVATE
     mujoco::mujoco
     "-framework Cocoa"
     "-framework IOKit"
     "-framework CoreVideo"
     "-framework CoreFoundation"
     glfw

)


set(XML_MODEL ${CMAKE_SOURCE_DIR}/example.xml)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${XML_MODEL}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)
