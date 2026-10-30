/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
GUI example.

Shows the GUI of the viewer (set_gui_enabled()) with a scene built in code: an arm of the robots
library, a quadruped (with the IMU of its model), a rangefinder, a camera, and a few free objects. Try:
  - Scene panel: select a body (e.g., "dog_trunk") to see its pose, also as a unit dual quaternion,
    in the Properties panel, and follow it with the camera.
  - Joints and Sensors panels: the joint positions within their limits, and the values of the
    IMU of the quadruped and of the rangefinder.
  - Visualization panel: contact forces, joint axes, frames, labels, ...
  - Menu bar: pause, step and reset the simulation (or Space, Right and Backspace), and look
    through the camera of the scene (Camera menu).

The program keeps commanding the robots while the simulation is paused: the commands are applied
when it runs again.

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

    if (!DQ_MujocoInterface::is_gui_available())
    {
        std::cerr << "This build of the library has no GUI (DQ_MUJOCO_BUILD_GUI=OFF)." << std::endl;
        return 1;
    }

    //--------------------------------------------------------------------------------------------
    // Build the scene
    //--------------------------------------------------------------------------------------------
    DQ_MujocoSceneBuilder scene("gui example");
    scene.add_floor();
    scene.add_light("sun", 3*k_, DQ(0, 0.3, 0.2, -1), true);
    scene.add_box("table", translation(0.6, 0, 0.2), {0.25, 0.25, 0.2}, BODY::FIXED, {0.55, 0.4, 0.3, 1});
    scene.add_box("crate", translation(0.6, 0.1, 0.45), {0.04, 0.04, 0.04}, BODY::FREE, {0.85, 0.55, 0.2, 1}, 0.2);
    scene.add_sphere("ball", translation(0.55, -0.12, 0.45), 0.04, BODY::FREE, {0.3, 0.55, 0.85, 1}, 0.1);

    const DQ r_dog = cos(M_PI/4) + k_*sin(M_PI/4);  // the quadruped faces +y
    const auto arm = scene.add_robot(GenericManipulatorRobot::description(), "arm_", translation(0, 0, 0));
    const auto dog = scene.add_robot(GenericQuadrupedRobot::description(), "dog_",
                                     r_dog + 0.5*E_*(-0.7*j_)*r_dog);
    scene.add_rangefinder("range", translation(0.6, 0, 1.2)*i_);  // above the table, its z axis down
    scene.add_camera("overview", DQ_MujocoSceneBuilder::look_at(1.6*i_ + 1.2*j_ + 1.1*k_, 0.2*i_ - 0.2*j_ + 0.2*k_));

    //--------------------------------------------------------------------------------------------
    // Show it with the GUI, and run it in real time
    //--------------------------------------------------------------------------------------------
    DQ_MujocoInterface mi(DQ_MujocoInterface::RENDERING::WINDOW, "", 1600, 960);
    mi.set_gui_enabled(true);
    mi.load_model(scene.build());
    mi.reset_simulation("home");  // every robot at its home configuration, where it was placed
    mi.set_site_frame_view(false);
    mi.start_simulation();

    const VectorXd q_arm_home = mi.get_joint_positions(arm.joint_names);
    Clock clock(0.002);  // control loop at 500 Hz
    clock.init();
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();

        // The arm sweeps its first two joints; the quadruped rocks its trunk
        VectorXd q_arm = q_arm_home;
        q_arm(0) = 0.8*sin(0.5*t);
        q_arm(1) = q_arm_home(1) + 0.3*sin(0.7*t);
        mi.set_joint_target_positions(arm.joint_names, q_arm);
        const double front_hip = 0.6 + 0.25*sin(2.0*t);
        const double rear_hip  = 0.6 - 0.25*sin(2.0*t);
        VectorXd q_dog(8);
        q_dog << front_hip, -2.0*front_hip, front_hip, -2.0*front_hip,
                 rear_hip,  -2.0*rear_hip,  rear_hip,  -2.0*rear_hip;
        mi.set_joint_target_positions(dog.joint_names, q_dog);

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
