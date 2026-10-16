/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Scene builder example.

Builds a scene in code with DQ_MujocoSceneBuilder: a floor, a light, a table, two quadrupeds and an
arm of the robots library, placed with unit dual quaternions, and a KINEMATIC block, moved by the
program with set_body_pose(): it slides on the floor and pushes what is in its way. Then it runs
the scene in real time and edits it while it runs, with edit_scene(): every 2 s a ball is dropped
on the table, and the oldest ball is removed when there are more than 5. The example also shows
how to:
  - use the names of the placed robots (prefixed: "arm_joint1", "dog1_trunk", ...)
  - start every robot at its home configuration, where it was placed (the "home" keyframe)
  - use the kinematics of the arm where it was placed: its forward kinematics is drawn as a
    reference frame, on top of the end effector of the simulation

No model file is needed.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/GenericManipulatorRobot.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/GenericQuadrupedRobot.h"
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
using BODY = DQ_MujocoSceneBuilder::BODY;

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


int main()
{
    std::signal(SIGINT, sig_int_handler);

    //--------------------------------------------------------------------------------------------
    // Build the scene
    //--------------------------------------------------------------------------------------------
    DQ_MujocoSceneBuilder scene("scene builder example");
    scene.add_floor();
    scene.add_light("sun", 3*k_, DQ(0, 0.3, 0.2, -1), true);
    scene.add_box("table", translation(0.6, 0, 0.2), {0.25, 0.25, 0.2}, BODY::FIXED, {0.55, 0.4, 0.3, 1});
    scene.add_box("slider", translation(0, -0.4, 0.05), {0.05, 0.05, 0.05}, BODY::KINEMATIC, {0.2, 0.7, 0.3, 1});

    const DQ x_arm = translation(0, 0, 0);
    const DQ r_dog = cos(M_PI/4) + k_*sin(M_PI/4);  // the quadrupeds face +y
    const DQ x_dog1 = r_dog + 0.5*E_*(-0.6*i_ - 0.8*j_)*r_dog;
    const DQ x_dog2 = r_dog + 0.5*E_*( 0.6*i_ - 0.8*j_)*r_dog;
    const auto arm  = scene.add_robot(GenericManipulatorRobot::description(), "arm_", x_arm);
    const auto dog1 = scene.add_robot(GenericQuadrupedRobot::description(), "dog1_", x_dog1);
    const auto dog2 = scene.add_robot(GenericQuadrupedRobot::description(), "dog2_", x_dog2);

    DQ_MujocoInterface mi;
    mi.load_model(scene.build());
    mi.reset_simulation("home");  // every robot at its home configuration, where it was placed
    mi.set_site_frame_view(false);

    // The kinematics of the arm, moved to where it was placed
    DQ_SerialManipulatorDH arm_kinematics = GenericManipulatorRobot::kinematics();
    arm_kinematics.set_reference_frame(x_arm*arm_kinematics.get_reference_frame());
    mi.add_reference_frame("fkm", 0.15, 0.006, 0.6);

    //--------------------------------------------------------------------------------------------
    // Run it in real time, and edit it while it runs
    //--------------------------------------------------------------------------------------------
    mi.start_simulation();
    const VectorXd q_arm_home = mi.get_joint_positions(arm.joint_names);
    std::deque<std::string> balls;
    int ball_count = 0;
    double next_ball_time = 1.0;

    Clock clock(0.002);  // control loop at 500 Hz
    clock.init();
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();

        // The arm sweeps its base joint; the quadrupeds rock their trunks, out of phase
        VectorXd q_arm = q_arm_home;
        q_arm(0) = 0.8*sin(0.5*t);
        mi.set_joint_target_positions(arm.joint_names, q_arm);
        for (const auto& [dog, phase] : {std::make_pair(dog1, 0.0), std::make_pair(dog2, M_PI)})
        {
            const double front_hip = 0.6 + 0.25*sin(2.0*t + phase);
            const double rear_hip  = 0.6 - 0.25*sin(2.0*t + phase);
            VectorXd q_ref(8);
            q_ref << front_hip, -2.0*front_hip,  front_hip, -2.0*front_hip,
                     rear_hip,  -2.0*rear_hip,   rear_hip,  -2.0*rear_hip;
            mi.set_joint_target_positions(dog.joint_names, q_ref);
        }

        // The kinematic block slides along x, between the table and the quadrupeds
        mi.set_body_pose("slider", translation(0.9*sin(0.4*t), -0.4, 0.05));

        // The forward kinematics of the arm, drawn on top of its end effector
        mi.set_reference_frame_pose("fkm", arm_kinematics.fkm(mi.get_joint_positions(arm.joint_names)));

        // Edit the scene while it runs: drop a ball on the table, and remove the oldest one
        if (t >= next_ball_time)
        {
            const std::string name = "ball_" + std::to_string(ball_count++);
            const double x = 0.6 + 0.15*cos(1.3*ball_count);  // on the table
            const double y = 0.15*sin(1.7*ball_count);
            const Vector4d color(0.5 + 0.5*cos(ball_count), 0.5 + 0.5*sin(ball_count), 0.3, 1);
            const std::string oldest = balls.size() >= 5 ? balls.front() : "";
            mi.edit_scene([&](DQ_MujocoSceneBuilder& edit) {
                edit.add_sphere(name, translation(x, y, 1.0), 0.04, BODY::FREE, color, 0.1);
                if (!oldest.empty())
                    edit.remove(oldest);
            });
            balls.push_back(name);
            if (!oldest.empty())
                balls.pop_front();
            std::cout << "t = " << t << "  dropped " << name
                      << (oldest.empty() ? "" : ", removed " + oldest)
                      << "  (real-time factor " << mi.get_real_time_factor() << ")" << std::endl;
            next_ball_time += 2.0;
        }

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
