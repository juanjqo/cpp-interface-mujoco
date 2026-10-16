/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/


#pragma once
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoRobotDescription.h>

namespace DQ_robotics_extensions
{

/**
 * @brief A generic 6-DoF manipulator built from primitive shapes (no mesh files).
 *
 * - Fixed base (no floating base).
 * - Joints: joint1 (base yaw, z), joint2 (shoulder pitch, y), joint3 (elbow pitch, y),
 *   joint4 (wrist roll, z), joint5 (wrist pitch, y), joint6 (flange roll, z), driven by position
 *   actuators. At q = 0 the arm points straight up.
 * - End effector: the site end_effector, 0.06 m above joint6.
 * - Keyframe "home": q = (0, 0.5, 1.2, 0, 1.0, 0), with the actuators holding the pose.
 * - The model also has a free-standing site, "target", in the world (not part of the robot), to
 *   be moved with set_site_pose() as a visual target.
 * - kinematics(): the DQ Robotics model of the arm. Its forward kinematics, fkm(q), is the pose of
 *   the end_effector site in the world, with q in the order of joint_names.
 *
 * Example:
 * @code
 * #include <dqrobotics_extensions/interfaces/mujoco/robots/GenericManipulatorRobot.h>
 *
 * const auto robot = GenericManipulatorRobot::description();
 * mi.load_model(robot.model);
 * mi.reset_simulation(robot.home_keyframe);
 * DQ x = mi.get_site_pose(robot.end_effector_sites.at(0));
 *
 * DQ_SerialManipulatorDH arm = GenericManipulatorRobot::kinematics();
 * VectorXd q = mi.get_joint_positions(robot.joint_names);
 * DQ x_model = arm.fkm(q);              // equals x
 * MatrixXd J = arm.pose_jacobian(q);
 * @endcode
 */
class GenericManipulatorRobot
{
public:
    static DQ_MujocoRobotDescription description();
    static DQ_robotics::DQ_SerialManipulatorDH kinematics();
};

}
