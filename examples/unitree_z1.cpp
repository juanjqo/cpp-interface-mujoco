/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Unitree Z1 example.

Two Unitree Z1 manipulators of the robots library, one with its gripper and one without it, placed
in a scene with DQ_MujocoSceneBuilder. The end effector of each one follows a target (a site) that
moves along a circle, with a kinematic controller of its position that uses the kinematics of the
robot (UnitreeZ1Robot::kinematics()), moved to where it was placed. The gripper opens and closes.

The models (and their meshes) are embedded in the robots library, so the example runs from any
directory.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/UnitreeZ1Robot.h"
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <marinholab/sas/core/sas_clock.hpp>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>

using namespace DQ_robotics;
using namespace DQ_robotics_extensions;
using namespace Eigen;
using marinholab::sas::core::Clock;
using GRIPPER = UnitreeZ1Robot::GRIPPER;

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/

static std::atomic_bool kill_this_process(false);

static void sig_int_handler(int)
{
    kill_this_process = true;
}

static DQ translation(const double& x, const double& y, const double& z)
{
    return 1 + 0.5*E_*(x*i_ + y*j_ + z*k_);
}

// A Z1 in the scene: its names, its kinematics where it was placed, and its commanded configuration
struct Arm
{
    DQ_MujocoRobotDescription robot;
    DQ_SerialManipulatorDH kinematics;
    std::string target;
    VectorXd q;
    DQ p_home;  // position of the end effector at home
};


int main()
{
    std::signal(SIGINT, sig_int_handler);

    //--------------------------------------------------------------------------------------------
    // The scene: a Z1 with its gripper (left) and one without it (right)
    //--------------------------------------------------------------------------------------------
    DQ_MujocoSceneBuilder scene("Unitree Z1");
    scene.add_floor();
    scene.add_light("sun", 3*k_, DQ(0, 0.3, 0.2, -1), true);

    std::vector<Arm> arms;
    for (const auto& [gripper, prefix, x] : {std::make_tuple(GRIPPER::WITH, "left_", translation(0, 0.35, 0)),
                                            std::make_tuple(GRIPPER::WITHOUT, "right_", translation(0, -0.35, 0))})
    {
        Arm arm{scene.add_robot(UnitreeZ1Robot::description(gripper), prefix, x),
                UnitreeZ1Robot::kinematics(gripper), std::string(prefix) + "target", VectorXd(), DQ(0)};
        arm.kinematics.set_reference_frame(x*arm.kinematics.get_reference_frame());
        scene.add_site(arm.target, x, 0.02, {1, 0, 0, 0.5});
        arms.push_back(arm);
    }

    DQ_MujocoInterface mi;
    mi.load_model(scene.build());
    mi.reset_simulation("home");  // both arms at home
    for (Arm& arm : arms)
    {
        arm.q = mi.get_joint_positions(arm.robot.joint_names);
        arm.p_home = arm.kinematics.fkm(arm.q).translation();
    }

    //--------------------------------------------------------------------------------------------
    // Run it in real time: each end effector follows its target, which moves along a vertical
    // circle that starts at its home position
    //--------------------------------------------------------------------------------------------
    mi.start_simulation();
    const double T = 0.002;      // control loop at 500 Hz
    const double gain = 20.0;
    const double radius = 0.08;
    Clock clock(T);
    clock.init();
    double next_print_time = 1.0;
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();
        for (Arm& arm : arms)
        {
            const DQ pd = arm.p_home + radius*(sin(1.5*t)*j_ + (1 - cos(1.5*t))*k_);
            mi.set_site_pose(arm.target, 1 + 0.5*E_*pd);

            // Kinematic control of the position: q_dot = -gain * pinv(Jt) * vec4(p - pd), integrated
            const DQ x = arm.kinematics.fkm(arm.q);
            const MatrixXd J = arm.kinematics.pose_jacobian(arm.q);
            const MatrixXd Jt = DQ_Kinematics::translation_jacobian(J, x);
            const VectorXd q_dot = -gain*pinv(Jt)*vec4(x.translation() - pd);
            arm.q = (arm.q + q_dot*T).cwiseMax(arm.kinematics.get_lower_q_limit())
                                     .cwiseMin(arm.kinematics.get_upper_q_limit());
            mi.set_joint_target_positions(arm.robot.joint_names, arm.q);

            // The gripper opens and closes (from -1.5 rad, open, to 0, closed)
            if (!arm.robot.gripper_joint_names.empty())
                mi.set_joint_target_positions(arm.robot.gripper_joint_names,
                                              (VectorXd(1) << -0.6*(1 - cos(2.0*t))).finished());
        }

        if (t >= next_print_time)
        {
            std::cout << "t = " << t;
            for (const Arm& arm : arms)
            {
                const DQ x_measured = mi.get_site_pose(arm.robot.end_effector_sites.at(0));
                const DQ xd = mi.get_site_pose(arm.target);
                std::cout << "  " << arm.robot.end_effector_sites.at(0) << " to target: "
                          << 1000*(x_measured.translation() - xd.translation()).vec3().norm() << " mm";
            }
            std::cout << "  (real-time factor " << mi.get_real_time_factor() << ")" << std::endl;
            next_print_time += 1.0;
        }

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
