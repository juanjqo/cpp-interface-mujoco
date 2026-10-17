/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Unitree B1 example.

The Unitree B1 quadruped of the robots library stands with its feet on the floor and moves its base:
up and down, and rolling and pitching. The desired pose of the base gives the desired position of
each foot relative to the base, and each leg follows it with a kinematic controller that uses the
kinematics of the legs (UnitreeB1Robot::kinematics(), in the frame of the base). The desired pose of
the base is drawn as a reference frame.

The model (and its meshes) is embedded in the robots library, so the example runs from any
directory.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/UnitreeB1Robot.h"
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
    // The scene: a B1 with position actuators (UnitreeB1Robot::ACTUATORS::TORQUE for torque motors)
    //--------------------------------------------------------------------------------------------
    DQ_MujocoSceneBuilder scene("Unitree B1");
    scene.add_floor();
    scene.add_light("sun", 3*k_, DQ(0, 0.3, 0.2, -1), true);
    const DQ_MujocoRobotDescription b1 = scene.add_robot(UnitreeB1Robot::description(), "b1_");

    DQ_MujocoInterface mi;
    mi.load_model(scene.build());
    mi.reset_simulation("home");
    mi.add_reference_frame("desired base", 0.3, 0.01, 0.7);

    // The legs, in the frame of the base, and where the feet are in the world
    std::vector<DQ_SerialManipulatorDH> legs = UnitreeB1Robot::kinematics();
    VectorXd q = mi.get_joint_positions(b1.joint_names);
    const DQ x_base_home = mi.get_base_pose(b1.floating_base);
    std::vector<DQ> p_feet;
    for (std::size_t i = 0; i < legs.size(); i++)
        p_feet.push_back((x_base_home*legs.at(i).fkm(q.segment(3*i, 3))).translation());

    //--------------------------------------------------------------------------------------------
    // Run it in real time: the base moves, and the feet stay where they are
    //--------------------------------------------------------------------------------------------
    mi.start_simulation();
    const double T = 0.002;      // control loop at 500 Hz
    const double gain = 50.0;
    Clock clock(T);
    clock.init();
    double next_print_time = 1.0;
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();

        // The desired pose of the base: up and down 6 cm, rolling 0.15 rad and pitching 0.1 rad
        const double roll = 0.15*sin(1.1*t);
        const double pitch = 0.10*sin(0.7*t);
        const DQ r = (cos(roll/2) + i_*sin(roll/2))*(cos(pitch/2) + j_*sin(pitch/2));
        const DQ p = 0.06*sin(0.8*t)*k_;
        const DQ x_base_d = x_base_home*(r + 0.5*E_*p*r);
        mi.set_reference_frame_pose("desired base", x_base_d);

        // Each leg follows its foot, relative to the desired base:
        // q_leg_dot = -gain * pinv(Jt) * vec4(p_foot - p_foot_d), integrated
        for (std::size_t i = 0; i < legs.size(); i++)
        {
            const DQ p_foot_d = (conj(x_base_d)*(1 + 0.5*E_*p_feet.at(i))).translation();
            VectorXd q_leg = q.segment(3*i, 3);
            const DQ x_foot = legs.at(i).fkm(q_leg);
            const MatrixXd Jt = DQ_Kinematics::translation_jacobian(legs.at(i).pose_jacobian(q_leg), x_foot);
            q_leg += -gain*pinv(Jt)*vec4(x_foot.translation() - p_foot_d)*T;
            q.segment(3*i, 3) = q_leg.cwiseMax(legs.at(i).get_lower_q_limit())
                                     .cwiseMin(legs.at(i).get_upper_q_limit());
        }
        mi.set_joint_target_positions(b1.joint_names, q);

        if (t >= next_print_time)
        {
            const DQ x_base = mi.get_base_pose(b1.floating_base);
            std::cout << "t = " << t << "  base to desired: "
                      << 1000*(x_base.translation() - x_base_d.translation()).vec3().norm() << " mm, "
                      << vec4(x_base.P() - x_base_d.P()).norm() << " (rotation)"
                      << "  (real-time factor " << mi.get_real_time_factor() << ")" << std::endl;
            next_print_time += 1.0;
        }

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
