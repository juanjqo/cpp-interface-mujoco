/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Sensors example.

Builds a scene with sensors, and reads them in real time:
  - the generic quadruped, whose model has an IMU (its imu_site), with a touch sensor added to each
    foot and a rangefinder under its trunk, looking down. It bobs up and down with its legs: the IMU
    measures the acceleration, the feet share its weight, and the rangefinder measures the height.
  - the generic manipulator, with a force/torque sensor added to its wrist (its last link). It moves
    its wrist joint: the sensor measures the weight of the last link, in the frame of the sensor.

The sensors are read with get_imu_orientation(), get_imu_linear_acceleration(), get_touch(),
get_range() and get_wrench(), by the names of their sites.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/GenericManipulatorRobot.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/GenericQuadrupedRobot.h"
#include <marinholab/sas/core/sas_clock.hpp>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iomanip>
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

    //--------------------------------------------------------------------------------------------
    // The scene, with the sensors
    //--------------------------------------------------------------------------------------------
    DQ_MujocoSceneBuilder scene("sensors example");
    scene.add_floor();
    scene.add_light("sun", 3*k_, DQ(0, 0.3, 0.2, -1), true);

    const DQ_MujocoRobotDescription dog = scene.add_robot(GenericQuadrupedRobot::description(), "dog_");
    const std::vector<std::string> legs = {"FL", "FR", "RL", "RR"};
    for (const auto& leg : legs)  // a box around each foot (0.15 m below the knee)
        scene.add_touch_sensor(leg + "_touch", 1 + 0.5*E_*(-0.15*k_), {0.04, 0.04, 0.04}, "dog_" + leg + "_calf");
    const DQ down = i_;  // 180 degrees about x: the z axis of the rangefinder points down
    scene.add_rangefinder("height", down + 0.5*E_*(-0.05*k_)*down, dog.floating_base);

    const DQ_MujocoRobotDescription arm = scene.add_robot(GenericManipulatorRobot::description(), "arm_",
                                                          1 + 0.5*E_*(-1.0*j_));
    scene.add_force_torque_sensor("wrist", DQ(1), "arm_link6");

    DQ_MujocoInterface mi;
    mi.load_model(scene.build());
    mi.reset_simulation("home");
    mi.set_site_frame_view(false);

    std::cout << "Sensors:";
    for (const auto& name : mi.get_sensor_names())
        std::cout << " " << name;
    std::cout << std::endl;

    //--------------------------------------------------------------------------------------------
    // Run it in real time
    //--------------------------------------------------------------------------------------------
    mi.start_simulation();
    const VectorXd q_arm_home = mi.get_joint_positions(arm.joint_names);
    Clock clock(0.002);  // control loop at 500 Hz
    clock.init();
    double next_print_time = 0.5;
    std::cout << std::fixed << std::setprecision(2);
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();

        // The quadruped bobs up and down (hips and knees together), and the arm turns its wrist
        const double hip = 0.6 + 0.2*sin(3.0*t);
        VectorXd q_dog(8);
        for (int i = 0; i < 4; i++)
            q_dog.segment(2*i, 2) << hip, -2.0*hip;
        mi.set_joint_target_positions(dog.joint_names, q_dog);
        VectorXd q_arm = q_arm_home;
        q_arm(4) = q_arm_home(4) + 0.8*sin(1.0*t);
        mi.set_joint_target_positions(arm.joint_names, q_arm);

        if (t >= next_print_time)
        {
            const DQ a = mi.get_imu_linear_acceleration(dog.imu_site);
            const DQ wrench = mi.get_wrench("wrist");
            std::cout << "t = " << t
                      << "  IMU acceleration: (" << a.q(1) << ", " << a.q(2) << ", " << a.q(3) << ") m/s^2"
                      << "  feet:";
            for (const auto& leg : legs)
                std::cout << " " << mi.get_touch(leg + "_touch");
            std::cout << " N  height: " << mi.get_range("height") << " m"
                      << "  wrist force: (" << wrench.q(1) << ", " << wrench.q(2) << ", " << wrench.q(3) << ") N"
                      << std::endl;
            next_print_time += 0.5;
        }

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
