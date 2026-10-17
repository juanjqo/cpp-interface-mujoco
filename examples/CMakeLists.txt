# Built from the top-level CMakeLists.txt (DQ_MUJOCO_BUILD_EXAMPLES=ON)

# Copy the models next to the executables, so the examples can be run from the build directory
add_custom_target(copy_example_models
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/models ${CMAKE_CURRENT_BINARY_DIR}
)

set(EXAMPLES
    twist_open_loop
)
# Examples that use the robots library (DQ_MUJOCO_BUILD_ROBOTS=ON)
set(ROBOT_EXAMPLES
    joint_control
    site_poses
    reference_frames
    floating_base
    scene_builder
    sensors
    cameras
)
if(DQ_MUJOCO_BUILD_ROBOTS)
    list(APPEND EXAMPLES ${ROBOT_EXAMPLES})
endif()

foreach(example IN LISTS EXAMPLES)
    add_executable(${example} ${example}.cpp)
    if(example IN_LIST ROBOT_EXAMPLES)
        target_link_libraries(${example} PRIVATE dqrobotics_extensions::interface-mujoco-generic-robots)
    else()
        target_link_libraries(${example} PRIVATE dqrobotics_extensions::interface-mujoco)
    endif()
    # The examples pace their loops with a sas Clock
    target_link_libraries(${example} PRIVATE marinholab::sas::core)
    # Building a single example (e.g., from an IDE) must also copy the models
    add_dependencies(${example} copy_example_models)
endforeach()
