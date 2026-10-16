# Built from the top-level CMakeLists.txt (DQ_MUJOCO_BUILD_TESTS=ON)

# The tests run inside a copy of the models, so files that MuJoCo writes (e.g., MUJOCO_LOG.TXT)
# stay in the build directory
add_custom_target(copy_test_models
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/models ${CMAKE_CURRENT_BINARY_DIR}/models
)

add_executable(test_interface test_interface.cpp)
# GLFW is linked directly because one test checks the GLFW error state
target_link_libraries(test_interface PRIVATE dqrobotics_extensions::interface-mujoco glfw)
add_dependencies(test_interface copy_test_models)

# Each test runs in its own process, so a crash in one does not hide the others
set(TESTS
    joint_positions_readable_after_set
    size_mismatch_throws
    site_follows_moving_body
    set_site_pose_on_worldbody_persists
    set_site_pose_on_moving_body
    joint_force_with_free_joint
    actuator_lookup_ignores_tendon_actuators
    load_error_includes_mujoco_message
    reset_restores_joints_and_sites
    load_model_twice
    destroy_without_loading
    two_interfaces_at_once
    headless_mode_needs_no_display
    reference_frames_api
    reference_frames_more_than_scene_capacity
    base_pose_set_and_get
    base_twist_matches_motion
    base_twist_reference_frames
    base_falls_under_gravity
    joint_methods_reject_free_and_ball_joints
    joint_and_floating_base_names
    reset_to_keyframe
    camera_tracking
)
foreach(test_name IN LISTS TESTS)
    add_test(NAME ${test_name}
             COMMAND test_interface ${test_name}
             WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/models)
endforeach()

# Prove that HEADLESS mode never touches the display
set_tests_properties(headless_mode_needs_no_display PROPERTIES
    ENVIRONMENT_MODIFICATION "DISPLAY=unset:;WAYLAND_DISPLAY=unset:"
)

# Undefined behavior in the destructor does not always crash, so also check it with valgrind
find_program(VALGRIND valgrind)
if(VALGRIND)
    add_test(NAME destroy_without_loading_valgrind
             COMMAND ${VALGRIND} -q --error-exitcode=1 $<TARGET_FILE:test_interface> destroy_without_loading
             WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/models)
endif()
