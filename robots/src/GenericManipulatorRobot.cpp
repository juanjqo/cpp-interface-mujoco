/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#include <dqrobotics_extensions/interfaces/mujoco/robots/GenericManipulatorRobot.h>
#include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics_extensions
{

// Generated from models/generic_manipulator by dq_mujoco_embed_model (see robots/CMakeLists.txt)
namespace embedded_models
{
DQ_MujocoModel generic_manipulator();
}

/**
 * @brief The MuJoCo model of the generic manipulator and the names needed to use it.
 * @return The description. Its model is embedded in the library, so no files are needed.
 */
DQ_MujocoRobotDescription GenericManipulatorRobot::description()
{
    DQ_MujocoRobotDescription robot;
    robot.model = embedded_models::generic_manipulator();
    robot.floating_base = "";  // fixed base
    robot.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    robot.home_keyframe = "home";
    robot.end_effector_sites = {"end_effector"};
    return robot;
}

/**
 * @brief The DQ Robotics kinematic model of the generic manipulator.
 * @return A DQ_SerialManipulatorDH (standard DH) whose forward kinematics, fkm(q), is the pose of
 *         the end_effector site in the world, for q in the order of description().joint_names.
 *         The joint limits are the joint ranges of the MuJoCo model.
 *
 * The DH parameters follow the geometry of generic_manipulator.xml. At q = 0 the arm points
 * straight up, and the joint axes are z, y, y, z, y, z (see the comments below). The base frame
 * is the world frame, since the robot is at the origin. Frame 6 is at the origin of link6 (the
 * flange), and the effector is the 0.06 m offset from it to the end_effector site.
 */
DQ_robotics::DQ_SerialManipulatorDH GenericManipulatorRobot::kinematics()
{
    using namespace DQ_robotics;
    const double pi2 = pi/2.0;

    Matrix<double, 5, 6> dh;
    //     joint1  joint2  joint3  joint4  joint5  joint6
    dh <<  0,      -pi2,   pi2,    0,      0,      0,      // theta (offset added to q)
           0.25,   0,      0,      0.45,   0,      0.10,   // d
           0,      0.40,   0,      0,      0,      0,      // a
           -pi2,   0,      pi2,    -pi2,   pi2,    0,      // alpha
           0,      0,      0,      0,      0,      0;      // joint types (all revolute)
    // joint1: base yaw (z), joint2's axis (y) crosses it at the shoulder, 0.25 m up
    // joint2: x2 points along the upper arm, 0.40 m to joint3 (parallel axes)
    // joint3: z3 turns along the forearm, the axis of joint4
    // joint4: 0.35 + 0.10 m along the forearm to joint5, whose axis crosses it
    // joint5: joint5 and joint6 axes cross at joint5
    // joint6: 0.10 m to the flange (origin of link6)

    DQ_SerialManipulatorDH arm(dh);
    arm.set_effector(1 + 0.5*E_*(0.06*k_));

    VectorXd q_max(6);
    q_max << 3.1416, 2.2, 2.6, 3.1416, 2.2, 3.1416;  // joint ranges in the XML
    arm.set_lower_q_limit(-q_max);
    arm.set_upper_q_limit(q_max);
    return arm;
}

}
