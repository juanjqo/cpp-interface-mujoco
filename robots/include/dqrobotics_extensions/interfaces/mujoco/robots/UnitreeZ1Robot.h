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
 * @brief The Unitree Z1, a 6-DoF manipulator, with or without its gripper, with position actuators,
 *        velocity actuators or torque motors.
 *
 * The MuJoCo model is the unitree_z1 model of cpp-interface-mujoco-models (from MuJoCo Menagerie,
 * BSD-3-Clause, Unitree Robotics), with its meshes embedded in the library.
 *
 * - Fixed base: link00, the root body, at the origin of the robot.
 * - Joints: joint1 (z), joint2 (y), joint3 (y), joint4 (y), joint5 (z), joint6 (x). At q = 0 the arm
 *   is folded over its base.
 * - Actuators of the arm, all within the torque limits of the joints (30 N·m, 60 N·m for joint2):
 *   ACTUATORS::POSITION (default): position actuators, for set_joint_target_positions().
 *   ACTUATORS::VELOCITY: velocity actuators, for set_joint_target_velocities(), within 3.1415 rad/s.
 *   With zero velocities, the arm sinks slowly.
 *   ACTUATORS::TORQUE: torque motors, for set_joint_target_forces(). Without a controller, the arm
 *   falls.
 * - Gripper (GRIPPER::WITH): one more joint, jointGripper (in gripper_joint_names, not in
 *   joint_names), from -1.5 rad (open) to 0 (closed), always with a position actuator.
 * - End effector: the site end_effector, at the flange (GRIPPER::WITHOUT) or at the tip of the
 *   gripper (GRIPPER::WITH), with its z axis along joint6.
 * - Keyframe "home": q = (0, 0.785, -0.261, -0.523, 0, 0), with the gripper closed, and the actuators
 *   holding the pose (ACTUATORS::POSITION), at zero velocity or at zero torque.
 * - kinematics(): the DQ Robotics model of the arm. Its forward kinematics, fkm(q), is the pose of
 *   the end_effector site in the world, with q in the order of joint_names.
 *
 * Example:
 * @code
 * #include <dqrobotics_extensions/interfaces/mujoco/robots/UnitreeZ1Robot.h>
 *
 * const auto robot = UnitreeZ1Robot::description();   // with the gripper, position actuators
 * mi.load_model(robot.model);
 * mi.reset_simulation(robot.home_keyframe);
 *
 * DQ_SerialManipulatorDH arm = UnitreeZ1Robot::kinematics();
 * VectorXd q = mi.get_joint_positions(robot.joint_names);
 * DQ x = arm.fkm(q);                                   // equals get_site_pose("end_effector")
 * mi.set_joint_target_positions(robot.gripper_joint_names, (VectorXd(1) << -1.0).finished());  // open
 * @endcode
 */
class UnitreeZ1Robot
{
public:
    /**
     * @brief Whether the robot has its gripper.
     */
    enum class GRIPPER
    {
        WITH,
        WITHOUT
    };

    /**
     * @brief The actuators of the arm.
     */
    enum class ACTUATORS
    {
        POSITION,
        VELOCITY,
        TORQUE
    };

    static DQ_MujocoRobotDescription description(const GRIPPER& gripper = GRIPPER::WITH,
                                                 const ACTUATORS& actuators = ACTUATORS::POSITION);
    static DQ_robotics::DQ_SerialManipulatorDH kinematics(const GRIPPER& gripper = GRIPPER::WITH);
};

}
