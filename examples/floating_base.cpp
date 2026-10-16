/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Floating base example.

A generic quadruped stands on its "home" keyframe and rocks its trunk by moving the legs with
position actuators. Every few seconds it gets a push (a change of the trunk twist) that makes it
slide and turn. The example shows how to:
  - load a robot of the robots library, with no model file (GenericQuadrupedRobot::description())
  - start from a keyframe (reset_simulation(robot.home_keyframe))
  - follow the robot with the camera (set_camera_tracking)
  - read and write the pose and twist of the trunk (get_base_pose, get_base_twist, set_base_twist)
  - read the pose of any body (get_body_pose)

The model is embedded in the robots library, so the example runs from any directory.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/GenericQuadrupedRobot.h"
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

    // The model and the names needed to use it: no file, no hardcoded joint names
    const DQ_MujocoRobotDescription robot = GenericQuadrupedRobot::description();

    DQ_MujocoInterface mi;
    mi.load_model(robot.model);

    const std::string& base = robot.floating_base;  // "trunk"
    // FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
    const std::vector<std::string>& jointnames = robot.joint_names;
    const std::vector<std::string> calves = {"FL_calf", "FR_calf", "RL_calf", "RR_calf"};

    mi.reset_simulation(robot.home_keyframe);
    mi.set_camera_tracking(base);
    mi.set_site_frame_view(false);

    mi.add_reference_frame(base, 0.3, 0.008);
    for (const auto& calf : calves)
        mi.add_reference_frame(calf, 0.08, 0.004, 1.0, false);

    double next_push_time = 3.0;
    double next_print_time = 0.0;

    while (!mi.should_close_window() && !kill_this_process)
    {
        double simstart = mi.get_simulation_time();
        while (mi.get_simulation_time() - simstart < 1.0/60.0)
        {
            const double t = mi.get_simulation_time();

            // Rock the trunk: the front and rear legs bend in opposite phase. The knee moves twice
            // as much as the hip, which keeps each foot right below its hip.
            const double front_hip = 0.6 + 0.25*sin(2.0*t);
            const double rear_hip  = 0.6 - 0.25*sin(2.0*t);
            VectorXd q_ref(8);
            q_ref << front_hip, -2.0*front_hip,  front_hip, -2.0*front_hip,
                     rear_hip,  -2.0*rear_hip,   rear_hip,  -2.0*rear_hip;
            mi.set_joint_target_positions(jointnames, q_ref);

            // Push: add a sideways velocity and a yaw rate to the current twist of the trunk.
            // A twist is xi = w + E_*(p_dot + cross(p, w)), with everything in the world frame.
            if (t >= next_push_time)
            {
                const DQ p = mi.get_base_pose(base).translation();
                const DQ w = 3.0*k_;
                const DQ p_dot = 0.8*j_;
                mi.set_base_twist(base, mi.get_base_twist(base) + w + E_*(p_dot + cross(p, w)));
                next_push_time += 4.0;
            }

            mi.trigger_next_simulation_step();
        }

        const DQ x_base = mi.get_base_pose(base);
        mi.set_reference_frame_pose(base, x_base);
        for (const auto& calf : calves)
            mi.set_reference_frame_pose(calf, mi.get_body_pose(calf));

        if (mi.get_simulation_time() >= next_print_time)
        {
            const DQ xi = mi.get_base_twist(base);
            std::cout << "t = " << mi.get_simulation_time()
                      << "  trunk position: " << x_base.translation().vec3().transpose()
                      << "  angular velocity: " << xi.P().vec3().transpose() << std::endl;
            next_print_time += 1.0;
        }

        mi.render();
    }
    return 0;
}
