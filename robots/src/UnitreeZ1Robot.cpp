/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#include <dqrobotics_extensions/interfaces/mujoco/robots/UnitreeZ1Robot.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <cmath>

namespace DQ_robotics_extensions
{

// Generated from cpp-interface-mujoco-models/unitree_z1 by dq_mujoco_embed_model (see
// robots/CMakeLists.txt): the model with the gripper and position actuators, z1_gripper.xml, with the
// other variants (and the robots they include, z1_robot.xml and z1_gripper_robot.xml) as more assets,
// so that all of them share the meshes
namespace embedded_models
{
DQ_MujocoModel unitree_z1();
}

/**
 * @brief The MuJoCo model of the Unitree Z1 and the names needed to use it.
 * @param gripper GRIPPER::WITH (default) or GRIPPER::WITHOUT.
 * @param actuators The actuators of the arm: ACTUATORS::POSITION (default), ACTUATORS::VELOCITY or
 *        ACTUATORS::TORQUE. The gripper always has a position actuator.
 * @return The description. Its model is embedded in the library, so no files are needed.
 */
DQ_MujocoRobotDescription UnitreeZ1Robot::description(const GRIPPER& gripper, const ACTUATORS& actuators)
{
    // The main file of the variant (z1_gripper_velocity.xml, ...), which includes its robot file
    const std::string robot_file = gripper == GRIPPER::WITH ? "z1_gripper_robot.xml" : "z1_robot.xml";
    std::string main_file = gripper == GRIPPER::WITH ? "z1_gripper" : "z1";
    if (actuators == ACTUATORS::VELOCITY)
        main_file += "_velocity";
    if (actuators == ACTUATORS::TORQUE)
        main_file += "_torque";
    main_file += ".xml";

    DQ_MujocoModel model = embedded_models::unitree_z1();
    if (main_file != model.name)
    {
        const auto& xml = model.assets.at(main_file);
        model.xml.assign(xml.begin(), xml.end());
        model.name = main_file;
    }
    // Only the robot file and the meshes of the variant are kept
    for (auto it = model.assets.begin(); it != model.assets.end();)
    {
        const std::string& name = it->first;
        const bool other_xml = name.size() > 4 && name.compare(name.size() - 4, 4, ".xml") == 0 && name != robot_file;
        const bool gripper_mesh = gripper == GRIPPER::WITHOUT && name.find("Gripper") != std::string::npos;
        it = other_xml || gripper_mesh ? model.assets.erase(it) : std::next(it);
    }

    DQ_MujocoRobotDescription robot;
    robot.model = std::move(model);
    robot.floating_base = "";  // fixed base
    robot.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    if (gripper == GRIPPER::WITH)
        robot.gripper_joint_names = {"jointGripper"};
    robot.home_keyframe = "home";
    robot.end_effector_sites = {"end_effector"};
    return robot;
}

/**
 * @brief The DQ Robotics kinematic model of the Unitree Z1.
 * @param gripper GRIPPER::WITH (default): the end effector is the tip of the gripper.
 *        GRIPPER::WITHOUT: the end effector is the flange.
 * @return A DQ_SerialManipulatorDH (standard DH) whose forward kinematics, fkm(q), is the pose of
 *         the end_effector site of description(gripper) in the world, for q in the order of
 *         description().joint_names. The joint limits are the joint ranges of the MuJoCo model.
 *
 * The DH parameters are those of the UnitreeZ1Robot of the Adorno-Lab unitree_kinematics library
 * (https://github.com/Adorno-Lab/unitree_kinematics), written here with the exact geometry of the
 * MuJoCo model instead of rounded values. Frame 0 is at joint1, 0.0585 m above the origin of the
 * robot (the reference frame), and frame 6 is at the origin of link06 (joint6), with its z axis
 * along joint6. The effector is the offset from it to the end_effector site.
 */
DQ_robotics::DQ_SerialManipulatorDH UnitreeZ1Robot::kinematics(const GRIPPER& gripper)
{
    using namespace DQ_robotics;
    const double pi2 = pi/2.0;
    // joint4 is (0.218, 0, 0.057) m from joint3: a distance a3 at an angle elbow
    const double elbow = std::atan2(0.057, 0.218);
    const double a3 = std::hypot(0.218, 0.057);

    Matrix<double, 5, 6> dh;
    //     joint1  joint2  joint3      joint4  joint5  joint6
    dh <<  0,      0,      pi - elbow, elbow,  -pi2,   -pi2,    // theta (offset added to q)
           0.045,  0,      0,          0,      0,      0.0492,  // d
           0,      -0.35,  -a3,        -0.07,  0,      0,       // a
           -pi2,   0,      0,          -pi2,   pi2,    0,       // alpha
           0,      0,      0,          0,      0,      0;       // joint types (all revolute)

    DQ_SerialManipulatorDH arm(dh);
    arm.set_reference_frame(1 + 0.5*E_*(0.0585*k_));
    arm.set_base_frame(arm.get_reference_frame());
    // From link06 to the tip of the gripper, or to the flange
    arm.set_effector(1 + 0.5*E_*((gripper == GRIPPER::WITH ? 0.2 : 0.051)*k_));

    VectorXd q_min(6);
    VectorXd q_max(6);
    q_min << -2.61799, 0,       -2.87979, -1.51844, -1.3439, -2.79253;  // joint ranges in the XML
    q_max <<  2.61799, 2.96706,  0,        1.51844,  1.3439,  2.79253;
    arm.set_lower_q_limit(q_min);
    arm.set_upper_q_limit(q_max);
    return arm;
}

}
