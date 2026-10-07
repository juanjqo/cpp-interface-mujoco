# Built from the top-level CMakeLists.txt (DQ_MUJOCO_BUILD_EXAMPLES=ON)

# Copy the models next to the executables, so the examples can be run from the build directory
add_custom_target(copy_example_models
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/models ${CMAKE_CURRENT_BINARY_DIR}
)

set(EXAMPLES
    joint_control
    site_poses
    reference_frames
)
foreach(example IN LISTS EXAMPLES)
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} PRIVATE dqrobotics_extensions::interface-mujoco)
    # Building a single example (e.g., from an IDE) must also copy the models
    add_dependencies(${example} copy_example_models)
endforeach()
