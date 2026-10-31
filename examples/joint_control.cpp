/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Joint space control example.

Moves every joint of the generic manipulator with a sinusoidal reference around a home
configuration, using MuJoCo's position actuators, and prints the joint positions,
velocities and actuator torques.

The model is embedded in the robots library (GenericManipulatorRobot), so the example runs from any
directory.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/GenericManipulatorRobot.h"
#include <marinholab/sas/core/sas_clock.hpp>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>

using namespace DQ_robotics;
using namespace DQ_robotics_extensions;
using namespace Eigen;
using marinholab::sas::core::Clock;

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
    mi.set_gui_enabled(DQ_MujocoInterface::is_gui_available());  // the GUI, if the library has it
    mi.load_model(robot.model);

    //mi.set_wireframe_view(true);
    //mi.set_joint_view(true);
    //mi.set_contact_view(true);
    //mi.set_center_of_mass_view(true);

    const std::vector<std::string>& jointnames = robot.joint_names;

    // Start from the "home" keyframe, a bent configuration, instead of the arm pointing straight up
    mi.reset_simulation(robot.home_keyframe);
    const VectorXd q_home = mi.get_joint_positions(jointnames);

    // The simulation runs in real time in its own thread, and the window is drawn automatically
    mi.start_simulation();

    // Control loop at 500 Hz, like the control loop of a real robot
    Clock clock(0.002);
    clock.init();
    double next_print_time = 0.0;
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();
        VectorXd q_ref = q_home + VectorXd::Constant(q_home.size(), 0.3 * sin(2.0 * t));
        mi.set_joint_target_positions(jointnames, q_ref);

        if (t >= next_print_time)
        {
            std::cout << "--------------------------------------" << std::endl;
            std::cout << "t:     " << t << " s (real-time factor " << mi.get_real_time_factor() << ")" << std::endl;
            std::cout << "q:     " << mi.get_joint_positions(jointnames).transpose() << std::endl;
            std::cout << "q_dot: " << mi.get_joint_velocities(jointnames).transpose() << std::endl;
            std::cout << "tau:   " << mi.get_joint_forces(jointnames).transpose() << std::endl;
            next_print_time += 0.1;
        }

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
