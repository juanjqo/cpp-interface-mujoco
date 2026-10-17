# Built from the top-level CMakeLists.txt (DQ_MUJOCO_BUILD_ROBOTS=ON)
#
# The robots library: robot models embedded in the library (no files needed at runtime), each with
# a description class (e.g., GenericQuadrupedRobot::description()).
#
# The generic robots are in models/. The other robots are in cpp-interface-mujoco-models
# (https://github.com/juanjqo/cpp-interface-mujoco-models), fetched at the commit below. To use a
# local checkout instead (e.g., while changing a model), configure with
#   -DFETCHCONTENT_SOURCE_DIR_CPP_INTERFACE_MUJOCO_MODELS=<path to the checkout>
#
# To add a robot:
#   1. put its model in a directory (main XML directly in it, assets below it): models/<robot>/ for
#      a generic robot, or a directory of cpp-interface-mujoco-models
#   2. embed it with dq_mujoco_embed_model() below
#   3. add include/.../robots/<Robot>Robot.h and src/<Robot>Robot.cpp, which fill the description

include(${PROJECT_SOURCE_DIR}/cmake/DQMujocoEmbedModel.cmake)

include(FetchContent)
FetchContent_Declare(
    cpp_interface_mujoco_models
    GIT_REPOSITORY https://github.com/juanjqo/cpp-interface-mujoco-models.git
    GIT_TAG        main  # TODO: pin to a commit
)
FetchContent_MakeAvailable(cpp_interface_mujoco_models)  # no CMakeLists.txt: only downloaded
set(MODELS_DIR ${cpp_interface_mujoco_models_SOURCE_DIR})

set(ROBOTS_TARGET ${PROJECT_NAME}-robots)

add_library(${ROBOTS_TARGET}
    src/GenericManipulatorRobot.cpp
    src/GenericQuadrupedRobot.cpp
    src/UnitreeB1Robot.cpp
    src/UnitreeZ1Robot.cpp
)
add_library(dqrobotics_extensions::interface-mujoco-robots ALIAS ${ROBOTS_TARGET})
set_target_properties(${ROBOTS_TARGET} PROPERTIES EXPORT_NAME interface-mujoco-robots)

dq_mujoco_embed_model(${ROBOTS_TARGET}
    FUNCTION  generic_manipulator
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models/generic_manipulator
    MAIN      generic_manipulator.xml
)
dq_mujoco_embed_model(${ROBOTS_TARGET}
    FUNCTION  generic_quadruped
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models/generic_quadruped
    MAIN      generic_quadruped.xml
)
# The B1 with position actuators, and with velocity actuators and torque motors (b1_velocity.xml and
# b1_torque.xml, as more assets), sharing the robot (b1_robot.xml) and the meshes
file(GLOB unitree_b1_meshes RELATIVE ${MODELS_DIR}/unitree_b1
     CONFIGURE_DEPENDS ${MODELS_DIR}/unitree_b1/assets/*.obj)
dq_mujoco_embed_model(${ROBOTS_TARGET}
    FUNCTION  unitree_b1
    DIRECTORY ${MODELS_DIR}/unitree_b1
    MAIN      b1.xml
    ASSETS    b1_velocity.xml b1_torque.xml b1_robot.xml ${unitree_b1_meshes}
)
# The Z1 with its gripper, and without it (z1.xml, as one more asset), sharing the meshes
file(GLOB unitree_z1_meshes RELATIVE ${MODELS_DIR}/unitree_z1
     CONFIGURE_DEPENDS ${MODELS_DIR}/unitree_z1/assets/*.stl)
dq_mujoco_embed_model(${ROBOTS_TARGET}
    FUNCTION  unitree_z1
    DIRECTORY ${MODELS_DIR}/unitree_z1
    MAIN      z1_gripper.xml
    ASSETS    z1.xml ${unitree_z1_meshes}
)

target_include_directories(${ROBOTS_TARGET} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
# Public: the descriptions are loaded with DQ_MujocoInterface
target_link_libraries(${ROBOTS_TARGET} PUBLIC ${PROJECT_NAME})

if(APPLE)
    set_target_properties(${ROBOTS_TARGET} PROPERTIES INSTALL_RPATH "@loader_path")
elseif(UNIX)
    set_target_properties(${ROBOTS_TARGET} PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

install(TARGETS ${ROBOTS_TARGET}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# The licenses of the embedded models that are not ours (BSD-3-Clause asks to ship its notice with
# the binaries)
foreach(model unitree_b1 unitree_z1)
    install(FILES ${MODELS_DIR}/${model}/LICENSE DESTINATION ${CMAKE_INSTALL_DOCDIR}/robots/${model})
endforeach()
