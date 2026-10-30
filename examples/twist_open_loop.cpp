/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Open-loop twist example.

Commands a floating base (a box) with constant twists, without any feedback: the pose of the box
is never used to compute the command. The box goes through these phases, 4 s each, repeatedly:

    1) rise                     WORLD_FRAME  xi = E_*(0.1*k_)             translation along the world z axis
    2) orbit the world z axis   WORLD_FRAME  xi = 0.5*k_                  rotation about the world z axis
    3) spin in place            BODY_FRAME   xi_body = 1.0*k_             rotation about the box's own z axis
    4) drive in a circle        BODY_FRAME   xi_body = 0.5*k_ + E_*0.2*i_  forward along the box's x axis while turning
    5) descend                  WORLD_FRAME  xi = E_*(-0.1*k_)

Phases 2 and 3 show the difference between the frames. In WORLD_FRAME, the reference point of the
twist is the world origin, so an angular velocity alone (phase 2) makes the box orbit the world z
axis. In BODY_FRAME, the reference point is the origin of the box, so the same kind of twist
(phase 3) makes it spin in place.

The twist is sent in every step. The box has gravity compensation and no collisions (see
floating_box.xml), but a twist sent only once would still not be kept: a free body keeps its
linear and angular momentum, so its origin would move in a straight line in phase 4 instead of
on a circle.

Run it from the directory that contains floating_box.xml (the build copies it next to the executable).
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
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

/*********************************************
 * PHASES
 * *******************************************/

using RF = DQ_MujocoInterface::REFERENCE_FRAME;

struct Phase
{
    std::string description;
    DQ twist;
    RF reference_frame;
};


int main()
{
    std::signal(SIGINT, sig_int_handler);

    DQ_MujocoInterface mi;
    mi.set_gui_enabled(DQ_MujocoInterface::is_gui_available());  // the GUI, if the library has it
    mi.load_model("floating_box.xml");

    const std::string box = "box";
    const double phase_duration = 4.0;  // s
    const std::vector<Phase> phases = {
        {"rise (world frame)",                 E_*(0.1*k_),          RF::WORLD_FRAME},
        {"orbit the world z axis (world frame)", 0.5*k_,             RF::WORLD_FRAME},
        {"spin in place (body frame)",         1.0*k_,               RF::BODY_FRAME},
        {"drive in a circle (body frame)",     0.5*k_ + E_*(0.2*i_), RF::BODY_FRAME},
        {"descend (world frame)",              E_*(-0.1*k_),         RF::WORLD_FRAME},
    };

    // Start 0.5 m away from the world z axis, so that the orbit in phase 2 is visible
    mi.set_base_pose(box, 1 + 0.5*E_*(0.5*i_ + 0.2*k_));

    mi.add_reference_frame("world", 0.2, 0.008);
    mi.add_reference_frame(box, 0.25, 0.006);

    int current_phase = -1;

    // Stepped mode: this loop advances the simulation itself, one timestep (2 ms) per iteration,
    // so the twist is sent before every step. The clock paces the loop to real time; without
    // it, the simulation would run as fast as the computer allows. The window is drawn
    // automatically.
    Clock clock(0.002);
    clock.init();
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();
        const int phase_index = static_cast<int>(t/phase_duration) % phases.size();
        const Phase& phase = phases.at(phase_index);

        if (phase_index != current_phase)
        {
            const DQ x = mi.get_base_pose(box);
            std::cout << "t = " << t << "  " << phase.description
                      << "  (box at " << x.translation().vec3().transpose() << ")" << std::endl;
            current_phase = phase_index;
        }

        // Open loop: the same twist in every step, whatever the pose of the box
        mi.set_base_twist(box, phase.twist, phase.reference_frame);
        mi.trigger_next_simulation_step();

        mi.set_reference_frame_pose(box, mi.get_base_pose(box));
        clock.update_and_sleep();
    }
    return 0;
}
