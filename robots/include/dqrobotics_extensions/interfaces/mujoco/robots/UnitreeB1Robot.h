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
#include <vector>

namespace DQ_robotics_extensions
{

/**
 * @brief The Unitree B1, a quadruped with 3 joints per leg, with position actuators, velocity
 *        actuators or torque motors.
 *
 * The MuJoCo model is the unitree_b1 model of cpp-interface-mujoco-models (BSD-3-Clause, Unitree
 * Robotics), with its meshes embedded in the library.
 *
 * - Floating base: the body "base".
 * - Joints, in the order of the actuators: FL_hip_joint (about x: abduction), FL_thigh_joint (about
 *   y), FL_calf_joint (about y), then FR_, RL_ and RR_ (front/rear, left/right).
 * - Actuators, all within the torque limits of the joints (91, 93.33 and 140 N·m):
 *   ACTUATORS::POSITION (default): position actuators, for set_joint_target_positions().
 *   ACTUATORS::VELOCITY: velocity actuators, for set_joint_target_velocities(), within the velocity
 *   limits of the joints (19.69, 23.32 and 15.55 rad/s). With zero velocities, the robot sinks slowly.
 *   ACTUATORS::TORQUE: torque motors, for set_joint_target_forces(). Without a controller, the
 *   robot falls.
 * - Feet: the sites FL_foot, FR_foot, RL_foot, RR_foot.
 * - Keyframe "home": standing, each leg at (hip, thigh, calf) = (0, 0.7, -1.2), with the
 *   actuators holding the pose (ACTUATORS::POSITION), at zero velocity (ACTUATORS::VELOCITY) or at
 *   zero torque (ACTUATORS::TORQUE).
 * - kinematics(): the DQ Robotics models of the legs, in the frame of the base.
 *
 * Example:
 * @code
 * #include <dqrobotics_extensions/interfaces/mujoco/robots/UnitreeB1Robot.h>
 *
 * const auto robot = UnitreeB1Robot::description();   // position actuators
 * mi.load_model(robot.model);
 * mi.reset_simulation(robot.home_keyframe);
 *
 * std::vector<DQ_SerialManipulatorDH> legs = UnitreeB1Robot::kinematics();
 * const DQ x_base = mi.get_base_pose(robot.floating_base);
 * const VectorXd q_FL = mi.get_joint_positions({robot.joint_names.begin(), robot.joint_names.begin() + 3});
 * DQ x_foot = x_base*legs.at(0).fkm(q_FL);              // equals get_site_pose("FL_foot")
 * @endcode
 */
class UnitreeB1Robot
{
public:
    /**
     * @brief The actuators of the robot.
     */
    enum class ACTUATORS
    {
        POSITION,
        VELOCITY,
        TORQUE
    };

    static DQ_MujocoRobotDescription description(const ACTUATORS& actuators = ACTUATORS::POSITION);
    static std::vector<DQ_robotics::DQ_SerialManipulatorDH> kinematics();
};

}
