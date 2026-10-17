/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#include <dqrobotics_extensions/interfaces/mujoco/robots/GenericQuadrupedRobot.h>
#include <dqrobotics/utils/DQ_Constants.h>

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
    robot.imu_site = "imu";
    return robot;
}

/**
 * @brief The DQ Robotics kinematic models of the legs of the generic quadruped.
 * @return The four legs, in the order FL, FR, RL, RR (the order of description().end_effector_sites).
 *         Leg i is a DQ_SerialManipulatorDH with the joints description().joint_names[2i] (hip)
 *         and [2i+1] (knee). Its joint limits are the joint ranges of the MuJoCo model.
 *
 * The legs are expressed in the TRUNK frame: the reference frame (and base frame) of each leg is
 * the pose of its hip relative to the trunk, so leg.fkm(q_leg) is the pose of the foot site
 * relative to the trunk. With the trunk pose x_trunk = get_base_pose("trunk"), the pose of the
 * foot in the world is
 *
 *     x_foot = x_trunk * leg.fkm(q_leg),
 *
 * and its pose Jacobian with respect to q_leg, in the world, is hamiplus8(x_trunk) * leg.pose_jacobian(q_leg).
 *
 * Alternatively, move a leg to the world with leg.set_reference_frame(x_trunk * hip), where hip is
 * the reference frame returned here (DQ Robotics' fkm() and pose_jacobian() use the reference
 * frame, not the base frame). Keep the original hip pose: calling it again with the moved frame
 * would apply the trunk pose twice.
 *
 * Geometry (from generic_quadruped.xml): the hips are at (+-0.17, +-0.13, 0) in the trunk frame,
 * both joints of a leg rotate about the y axis of the trunk, and at q = 0 the thigh (0.15 m) and
 * the calf (0.15 m) hang straight down. A planar 2-link arm models this: the base frame turns its
 * z axis to the y axis of the trunk (the joint axis) and its x axis down along the thigh, and the
 * effector turns back to the orientation of the foot site, which is that of the calf.
 */
std::vector<DQ_robotics::DQ_SerialManipulatorDH> GenericQuadrupedRobot::kinematics()
{
    using namespace DQ_robotics;

    Matrix<double, 5, 2> dh;
    //     hip    knee
    dh <<  0,     0,       // theta (offset added to q)
           0,     0,       // d
           0.15,  0.15,    // a: thigh and calf
           0,     0,       // alpha (parallel axes)
           0,     0;       // joint types (both revolute)

    // Rotation from the trunk axes to the DH base frame of a leg: z to the y axis of the trunk,
    // then x down along the thigh (-z of the trunk)
    const DQ r_x = cos(-pi/4.0) + i_*sin(-pi/4.0);
    const DQ r_y = cos(pi/4.0) + j_*sin(pi/4.0);
    const DQ r_base = r_y*r_x;

    const std::vector<DQ> hip_positions = { 0.17*i_ + 0.13*j_,    // FL
                                            0.17*i_ - 0.13*j_,    // FR
                                           -0.17*i_ + 0.13*j_,    // RL
                                           -0.17*i_ - 0.13*j_};   // RR
    VectorXd q_min(2), q_max(2);
    q_min << -1.5, -2.6;  // hip and knee ranges in the XML
    q_max <<  1.5, -0.2;

    std::vector<DQ_SerialManipulatorDH> legs;
    for (const DQ& p_hip : hip_positions)
    {
        DQ_SerialManipulatorDH leg(dh);
        leg.set_base_frame(r_base + 0.5*E_*p_hip*r_base);
        leg.set_reference_frame(r_base + 0.5*E_*p_hip*r_base);
        leg.set_effector(conj(r_base));
        leg.set_lower_q_limit(q_min);
        leg.set_upper_q_limit(q_max);
        legs.push_back(leg);
    }
    return legs;
}

}
