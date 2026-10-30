/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Reference frames example.

Reference frames are created at runtime with add_reference_frame(). They do not need to be
declared in the XML model, and they are only drawn (no physics). This example draws:
  - "home": the end-effector pose at the start (a fixed frame)
  - "x_desired": a semi-transparent desired pose moving along a circle
  - "x": the measured end-effector pose (read from the "end_effector" site)
  - a trail with the last end-effector poses, created and removed on the fly

The model is embedded in the robots library (GenericManipulatorRobot), so the example runs from any
directory.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/GenericManipulatorRobot.h"
#include <marinholab/sas/core/sas_clock.hpp>
#include <atomic>
#include <cmath>
#include <csignal>
#include <deque>
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

    const std::vector<std::string>& jointnames = robot.joint_names;

    mi.reset_simulation(robot.home_keyframe);  // q = (0, 0.5, 1.2, 0, 1.0, 0)
    const VectorXd q_home = mi.get_joint_positions(jointnames);

    // Hide the frames of the XML sites, so they do not overlap the reference frames
    mi.set_site_frame_view(false);

    //                     name         axis length  axis width  opacity  label
    mi.add_reference_frame("home",      0.15,        0.006);
    mi.add_reference_frame("x_desired", 0.2,         0.009,      0.5);
    mi.add_reference_frame("x",         0.15,        0.006);
    mi.set_reference_frame_pose("home", mi.get_site_pose(robot.end_effector_sites.at(0)));

    // Trail of past end-effector poses: one small unlabeled frame every 0.5 s, keeping the last 8
    std::deque<std::string> trail;
    int trail_count = 0;
    double next_trail_time = 0.0;

    // The simulation runs in real time in its own thread, and the window is drawn automatically
    mi.start_simulation();

    // Control loop at 500 Hz
    Clock clock(0.002);
    clock.init();
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();

        // Desired pose: a circle of radius 0.2 m at height 0.4 m, rotating about a moving axis
        DQ axis = (i_*cos(t) + j_*sin(t)).normalize();
        DQ r = cos(M_PI/8) + axis*sin(M_PI/8);
        DQ p = 0.4*i_ + 0.2*cos(t)*j_ + (0.4 + 0.2*sin(t))*k_;
        mi.set_reference_frame_pose("x_desired", r + 0.5*E_*p*r);

        // Arm: sweep the base joint back and forth
        VectorXd q_ref = q_home;
        q_ref(0) = 0.8 * sin(0.5 * t);
        mi.set_joint_target_positions(jointnames, q_ref);

        // Draw the measured end-effector pose
        const DQ x = mi.get_site_pose(robot.end_effector_sites.at(0));
        mi.set_reference_frame_pose("x", x);

        // Update the trail
        if (t >= next_trail_time)
        {
            const std::string name = "trail_" + std::to_string(trail_count++);
            mi.add_reference_frame(name, 0.05, 0.003, 0.6, false);
            mi.set_reference_frame_pose(name, x);
            trail.push_back(name);
            if (trail.size() > 8)
            {
                mi.remove_reference_frame(trail.front());
                trail.pop_front();
            }
            next_trail_time += 0.5;
        }

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
