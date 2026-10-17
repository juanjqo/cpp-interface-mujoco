/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/


#pragma once
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoModel.h>
#include <string>
#include <vector>

namespace DQ_robotics_extensions
{

/**
 * @brief A robot of the robots library: its MuJoCo model and the names needed to use it, so that
 *        they do not have to be hardcoded.
 *
 * Example:
 * @code
 * const DQ_MujocoRobotDescription robot = GenericQuadrupedRobot::description();
 * DQ_MujocoInterface mi;
 * mi.load_model(robot.model);
 * mi.reset_simulation(robot.home_keyframe);
 * VectorXd q = mi.get_joint_positions(robot.joint_names);
 * DQ x_base = mi.get_base_pose(robot.floating_base);
 * @endcode
 */
struct DQ_MujocoRobotDescription
{
    /// The MuJoCo model, held in memory (no files needed).
    DQ_MujocoModel model;

    /// Body that owns the free joint, for get_base_pose() etc. Empty for fixed-base robots.
    std::string floating_base;

    /// Actuated joints, in the order of the actuators (and of the keyframe ctrl), without those of
    /// the grippers: the joints of the kinematic model of the robot.
    std::vector<std::string> joint_names;
    /// Actuated joints of the grippers, if any (e.g., set_joint_target_positions() opens and closes
    /// them).
    std::vector<std::string> gripper_joint_names;

    /// Keyframe with the default configuration, for reset_simulation(). Empty if there is none.
    std::string home_keyframe;

    /// Sites at the end effectors (e.g., the feet of a legged robot), for get_site_pose().
    std::vector<std::string> end_effector_sites;
};

}
