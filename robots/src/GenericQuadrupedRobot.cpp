/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#include <dqrobotics_extensions/interfaces/mujoco/robots/GenericQuadrupedRobot.h>

namespace DQ_robotics_extensions
{

// Generated from models/generic_quadruped by dq_mujoco_embed_model (see robots/CMakeLists.txt)
namespace embedded_models
{
DQ_MujocoModel generic_quadruped();
}

/**
 * @brief The MuJoCo model of the generic quadruped and the names needed to use it.
 * @return The description. Its model is embedded in the library, so no files are needed.
 */
DQ_MujocoRobotDescription GenericQuadrupedRobot::description()
{
    DQ_MujocoRobotDescription robot;
    robot.model = embedded_models::generic_quadruped();
    robot.floating_base = "trunk";
    robot.joint_names = {"FL_hip", "FL_knee", "FR_hip", "FR_knee",
                         "RL_hip", "RL_knee", "RR_hip", "RR_knee"};
    robot.home_keyframe = "home";
    robot.end_effector_sites = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    return robot;
}

}
