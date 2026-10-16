/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/


#pragma once
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics_extensions/interfaces/mujoco/robots/DQ_MujocoRobotDescription.h>
#include <vector>

namespace DQ_robotics_extensions
{

/**
 * @brief A generic quadruped built from primitive shapes (no mesh files).
 *
 * - Floating base: the body "trunk" (a 0.4 x 0.2 x 0.08 m box).
 * - Joints: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee (front/rear,
 *   left/right), all about the y axis and driven by position actuators.
 * - End effectors: the sites FL_foot, FR_foot, RL_foot, RR_foot, at the center of each foot.
 * - Keyframe "home": standing on the floor, with the actuators holding the pose.
 * - kinematics(): the DQ Robotics model of each leg, in the trunk frame (see kinematics()).
 *
 * Example:
 * @code
 * #include <dqrobotics_extensions/interfaces/mujoco/robots/GenericQuadrupedRobot.h>
 *
 * const auto robot = GenericQuadrupedRobot::description();
 * mi.load_model(robot.model);
 * mi.reset_simulation(robot.home_keyframe);
 *
 * // Pose of the front-left foot in the world
 * std::vector<DQ_SerialManipulatorDH> legs = GenericQuadrupedRobot::kinematics();
 * VectorXd q_FL = mi.get_joint_positions({robot.joint_names.at(0), robot.joint_names.at(1)});
 * DQ x_foot = mi.get_base_pose(robot.floating_base) * legs.at(0).fkm(q_FL);  // = get_site_pose("FL_foot")
 * @endcode
 */
class GenericQuadrupedRobot
{
public:
    static DQ_MujocoRobotDescription description();
    static std::vector<DQ_robotics::DQ_SerialManipulatorDH> kinematics();
};

}
