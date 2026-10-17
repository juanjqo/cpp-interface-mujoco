/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#include <dqrobotics_extensions/interfaces/mujoco/robots/UnitreeB1Robot.h>
#include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics_extensions
{

// Generated from cpp-interface-mujoco-models/unitree_b1 by dq_mujoco_embed_model (see
// robots/CMakeLists.txt): the model with position actuators, b1.xml, with the models with velocity
// actuators and torque motors, b1_velocity.xml and b1_torque.xml, as more assets, so that all of them
// share the robot (b1_robot.xml) and the meshes
namespace embedded_models
{
DQ_MujocoModel unitree_b1();
}

/**
 * @brief The MuJoCo model of the Unitree B1 and the names needed to use it.
 * @param actuators ACTUATORS::POSITION (default), ACTUATORS::VELOCITY or ACTUATORS::TORQUE.
 * @return The description. Its model is embedded in the library, so no files are needed.
 */
DQ_MujocoRobotDescription UnitreeB1Robot::description(const ACTUATORS& actuators)
{
    DQ_MujocoModel model = embedded_models::unitree_b1();
    for (const auto& [variant, file] : {std::make_pair(ACTUATORS::VELOCITY, "b1_velocity.xml"),
                                        std::make_pair(ACTUATORS::TORQUE, "b1_torque.xml")})
    {
        const auto it = model.assets.find(file);
        if (actuators == variant)
        {
            model.name = file;
            model.xml.assign(it->second.begin(), it->second.end());
        }
        model.assets.erase(it);
    }

    DQ_MujocoRobotDescription robot;
    robot.model = std::move(model);
    robot.floating_base = "base";
    for (const std::string leg : {"FL", "FR", "RL", "RR"})
        for (const std::string joint : {"hip", "thigh", "calf"})
            robot.joint_names.push_back(leg + "_" + joint + "_joint");
    robot.home_keyframe = "home";
    robot.end_effector_sites = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    return robot;
}

/**
 * @brief The DQ Robotics kinematic models of the legs of the Unitree B1.
 * @return The four legs, in the order FL, FR, RL, RR (the order of description().end_effector_sites).
 *         Leg i is a DQ_SerialManipulatorDH with the joints description().joint_names[3i] (hip),
 *         [3i+1] (thigh) and [3i+2] (calf). Its joint limits are the joint ranges of the MuJoCo model.
 *
 * The legs are expressed in the BASE frame: the reference frame (and base frame) of each leg is at
 * its hip, relative to the base, so leg.fkm(q_leg) is the pose of the foot site relative to the
 * base. With the base pose x_base = get_base_pose("base"), the pose of the foot in the world is
 *
 *     x_foot = x_base * leg.fkm(q_leg),
 *
 * and its pose Jacobian with respect to q_leg, in the world, is hamiplus8(x_base) * leg.pose_jacobian(q_leg).
 * Alternatively, move a leg to the world with leg.set_reference_frame(x_base * hip), where hip is
 * the reference frame returned here.
 *
 * Geometry (from the URDF of unitree_ros): the hips are at (+-0.3455, +-0.072, 0) in the base frame.
 * The hip joint rotates about the x axis of the base, and the thigh joint about its y axis; the two
 * axes cross at the hip. The thigh joint is 0.12675 m from the hip along its axis (to the left for
 * the left legs), and at q = 0 the thigh (0.35 m) and the calf (0.35 m) hang straight down, with the
 * calf joint about the y axis too. The DH base frame turns its z axis to the x axis of the base (the
 * hip axis) and its x axis down (along the thigh, at q = 0), and the effector turns back to the
 * orientation of the foot site, which is that of the calf.
 */
std::vector<DQ_robotics::DQ_SerialManipulatorDH> UnitreeB1Robot::kinematics()
{
    using namespace DQ_robotics;

    // Rotation from the base axes to the DH base frame of a leg: 90 degrees about y (z to x, x down)
    const DQ r_base = cos(pi/4.0) + j_*sin(pi/4.0);

    struct Leg { double x, y; };  // signs: front/rear, left/right
    const std::vector<Leg> legs_signs = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};  // FL, FR, RL, RR

    VectorXd q_min(3), q_max(3);
    q_min << -0.75, -1.0, -2.6;  // hip, thigh and calf ranges in the XML
    q_max <<  0.75,  3.5, -0.6;

    std::vector<DQ_SerialManipulatorDH> legs;
    for (const Leg& s : legs_signs)
    {
        Matrix<double, 5, 3> dh;
        //     hip     thigh            calf
        dh <<  0,      0,               0,      // theta (offset added to q)
               0,      0.12675*s.y,     0,      // d: the thigh joint along its axis
               0,      0.35,            0.35,   // a: thigh and calf
               -pi/2,  0,               0,      // alpha: from the hip axis to the thigh axis
               0,      0,               0;      // joint types (all revolute)
        DQ_SerialManipulatorDH leg(dh);
        const DQ p_hip = 0.3455*s.x*i_ + 0.072*s.y*j_;
        const DQ x_hip = r_base + 0.5*E_*p_hip*r_base;
        leg.set_base_frame(x_hip);
        leg.set_reference_frame(x_hip);
        // At q = 0, frame 3 is at the foot; the foot site has the orientation of the base
        const DQ r_3 = leg.fkm(VectorXd::Zero(3)).P();
        leg.set_effector(conj(r_3));
        leg.set_lower_q_limit(q_min);
        leg.set_upper_q_limit(q_max);
        legs.push_back(leg);
    }
    return legs;
}

}
