/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Site poses example.

Sites are frames with no physics. This example:
  - moves the free-standing "target" site along a circle while rotating it, with set_site_pose()
  - moves the arm with its position actuators
  - reads the pose of the "end_effector" site, attached to the last link, with get_site_pose()
and prints both poses and the distance between them.

The model is embedded in the robots library (GenericManipulatorRobot), so the example runs from any
directory.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/GenericManipulatorRobot.h"
#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>

using namespace DQ_robotics;
using namespace DQ_robotics_extensions;
using namespace Eigen;

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/

static std::atomic_bool kill_this_process(false);

static void sig_int_handler(int)
{
    kill_this_process = true;
}


int main()
{
    std::signal(SIGINT, sig_int_handler);

    const DQ_MujocoRobotDescription robot = GenericManipulatorRobot::description();

    DQ_MujocoInterface mi;
    mi.load_model(robot.model);

    const std::vector<std::string>& jointnames = robot.joint_names;

    mi.reset_simulation(robot.home_keyframe);  // q = (0, 0.5, 1.2, 0, 1.0, 0)
    const VectorXd q_home = mi.get_joint_positions(jointnames);

    while (!mi.should_close_window() && !kill_this_process)
    {
        double simstart = mi.get_simulation_time();
        while (mi.get_simulation_time() - simstart < 1.0/60.0)
        {
            const double t = mi.get_simulation_time();

            // Target: a circle of radius 0.2 m at height 0.4 m, rotating about a moving axis
            DQ axis = (i_*cos(t) + j_*sin(t)).normalize();
            DQ r = cos(M_PI/8) + axis*sin(M_PI/8);
            DQ p = 0.4*i_ + 0.2*cos(t)*j_ + (0.4 + 0.2*sin(t))*k_;
            mi.set_site_pose("target", r + 0.5*E_*p*r);

            // Arm: sweep the base joint back and forth
            VectorXd q_ref = q_home;
            q_ref(0) = 0.8 * sin(0.5 * t);
            mi.set_joint_target_positions(jointnames, q_ref);

            mi.trigger_next_simulation_step();
        }

        DQ x_target = mi.get_site_pose("target");
        DQ x_effector = mi.get_site_pose(robot.end_effector_sites.at(0));
        VectorXd p_target = x_target.translation().vec3();
        VectorXd p_effector = x_effector.translation().vec3();

        std::cout << "--------------------------------------" << std::endl;
        std::cout << "target:       " << p_target.transpose() << std::endl;
        std::cout << "end effector: " << p_effector.transpose() << std::endl;
        std::cout << "distance:     " << (p_target - p_effector).norm() << " m" << std::endl;

        mi.render();
    }
    return 0;
}
