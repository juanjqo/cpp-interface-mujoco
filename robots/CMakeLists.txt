# Built from the top-level CMakeLists.txt (DQ_MUJOCO_BUILD_ROBOTS=ON)
#
# The robots library: robot models embedded in the library (no files needed at runtime), each with
# a description class (e.g., GenericQuadrupedRobot::description()).
#
# To add a robot:
#   1. put its model in models/<robot>/ (main XML directly in that directory, assets below it)
#   2. embed it with dq_mujoco_embed_model() below
#   3. add include/.../robots/<Robot>Robot.h and src/<Robot>Robot.cpp, which fill the description

include(${PROJECT_SOURCE_DIR}/cmake/DQMujocoEmbedModel.cmake)

set(ROBOTS_TARGET ${PROJECT_NAME}-robots)

add_library(${ROBOTS_TARGET}
    src/GenericManipulatorRobot.cpp
    src/GenericQuadrupedRobot.cpp
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
# The Z1 with its gripper, and without it (z1.xml, as one more asset), sharing the meshes
file(GLOB unitree_z1_meshes RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/models/unitree_z1
     CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/unitree_z1/assets/*.stl)
dq_mujoco_embed_model(${ROBOTS_TARGET}
    FUNCTION  unitree_z1
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models/unitree_z1
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
install(FILES models/unitree_z1/LICENSE
        DESTINATION ${CMAKE_INSTALL_DOCDIR}/robots/unitree_z1)
