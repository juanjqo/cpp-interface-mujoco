# Built from the top-level CMakeLists.txt (DQ_MUJOCO_BUILD_TESTS=ON)

# The tests run inside a copy of the models, so files that MuJoCo writes (e.g., MUJOCO_LOG.TXT)
# stay in the build directory
add_custom_target(copy_test_models
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/models ${CMAKE_CURRENT_BINARY_DIR}/models
)

if(NOT DQ_MUJOCO_BUILD_ROBOTS)
    message(FATAL_ERROR "The tests need the generic robots library: configure with -DDQ_MUJOCO_BUILD_ROBOTS=ON")
endif()

add_executable(test_interface test_interface.cpp)
# GLFW is linked directly because one test checks the GLFW error state
target_link_libraries(test_interface PRIVATE dqrobotics_extensions::interface-mujoco-generic-robots glfw)
add_dependencies(test_interface copy_test_models)

# Embeds tests/models/embedded, to test dq_mujoco_embed_model (DQ_MUJOCO_BUILD_ROBOTS includes it)
dq_mujoco_embed_model(test_interface
    FUNCTION  embedded_test_scene
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models/embedded
    MAIN      scene.xml
    ASSETS    parts/arm.xml meshes/tetra.obj data/blob.bin
)
# A robot with files (a mesh, textures, an include file), to test robots with files in scenes
dq_mujoco_embed_model(test_interface
    FUNCTION  meshed_robot
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models/meshed_robot
    MAIN      meshed_robot.xml
    ASSETS    parts/arm.xml meshes/link.obj textures/checker.png textures/sky.png
)

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
    scene_builder_build
    scene_builder_exact_numbers
    scene_builder_options_and_errors
    edit_scene_while_running
    edit_scene_of_a_model_file
    edit_scene_with_window
    kinematic_body
    kinematic_body_while_running
    real_time_simulation
    commands_and_teleports_while_running
    viewer_draws_in_both_modes
    load_model_from_memory
    load_model_from_memory_errors
    embedded_model_with_assets
    generic_quadruped_description
    generic_manipulator_description
    generic_manipulator_kinematics
    generic_quadruped_kinematics
    scene_builder_mounted_robot
    sensors_of_a_model
    scene_builder_sensors
    edit_scene_sensors_while_running
    cameras_headless
    cameras_while_running
    camera_in_window_mode
    scene_builder_robot_with_meshes
    edit_scene_robot_with_meshes
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
