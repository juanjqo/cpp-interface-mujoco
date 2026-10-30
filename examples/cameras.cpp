/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Cameras example.

Builds a scene with two cameras, and renders them while the simulation runs in real time:
  - "head": on the trunk of the generic quadruped, looking forward (RGB and depth, 30 Hz). The
    quadruped rocks its trunk with its legs, so the camera moves with it.
  - "overview": fixed in the world, looking at the scene (RGB, 10 Hz, FAST quality).

The cameras are rendered offscreen by a camera thread, also in HEADLESS mode (run it with the
argument "headless"). Once per second, the latest image of each camera is written to <camera>.ppm
(open it with any image viewer), and the number of images rendered so far is printed.

The images have the layout of a ROS sensor_msgs/Image ("rgb8", and "32FC1" for the depth): a ROS
driver can publish them as they are, when their count changes, stamped with their simulation time.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/GenericQuadrupedRobot.h"
#include <marinholab/sas/core/sas_clock.hpp>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
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

// Writes the RGB image as a binary PPM file
static void write_ppm(const std::string& file, const DQ_MujocoInterface::CameraImage& image)
{
    std::ofstream out(file, std::ios::binary);
    out << "P6\n" << image.width << " " << image.height << "\n255\n";
    out.write(reinterpret_cast<const char*>(image.rgb.data()), static_cast<std::streamsize>(image.rgb.size()));
}


int main(int argc, char** argv)
{
    std::signal(SIGINT, sig_int_handler);
    const bool headless = argc > 1 && std::strcmp(argv[1], "headless") == 0;

    //--------------------------------------------------------------------------------------------
    // The scene, with the cameras
    //--------------------------------------------------------------------------------------------
    DQ_MujocoSceneBuilder scene("cameras example");
    scene.add_floor();
    scene.add_light("sun", 3*k_, DQ(0, 0.3, 0.2, -1), true);
    const DQ_MujocoRobotDescription dog = scene.add_robot(GenericQuadrupedRobot::description(), "dog_");
    scene.add_box("red", 1 + 0.5*E_*(1.0*i_ + 0.15*k_), {0.15, 0.15, 0.15}, BODY::FIXED, {0.9, 0.1, 0.1, 1});
    scene.add_sphere("blue", 1 + 0.5*E_*(1.2*i_ + 0.4*j_ + 0.1*k_), 0.1, BODY::FIXED, {0.1, 0.3, 0.9, 1});
    // On the front of the trunk, looking forward (along x) and a bit down
    scene.add_camera("head", DQ_MujocoSceneBuilder::look_at(0.2*i_ + 0.05*k_, 1.0*i_ - 0.1*k_),
                     dog.floating_base, 60, 640, 480);
    // In the world, looking at the scene
    scene.add_camera("overview", DQ_MujocoSceneBuilder::look_at(-1.0*i_ - 1.5*j_ + 1.2*k_, 0.4*i_ + 0.2*k_),
                     "", 45, 640, 480);

    DQ_MujocoInterface mi(headless ? DQ_MujocoInterface::RENDERING::HEADLESS : DQ_MujocoInterface::RENDERING::WINDOW);
    mi.set_gui_enabled(DQ_MujocoInterface::is_gui_available());  // the GUI, if the library has it
    mi.load_model(scene.build());
    mi.reset_simulation("home");

    //--------------------------------------------------------------------------------------------
    // Run it in real time, with the cameras
    //--------------------------------------------------------------------------------------------
    mi.start_simulation();
    mi.start_camera("head", 30.0, true);
    mi.start_camera("overview", 10.0, false, DQ_MujocoInterface::CAMERA_QUALITY::FAST);

    Clock clock(0.002);  // control loop at 500 Hz
    clock.init();
    double next_save_time = 1.0;
    while (!mi.should_close_window() && !kill_this_process && (!headless || mi.get_simulation_time() < 10.0))
    {
        const double t = mi.get_simulation_time();

        // The quadruped rocks its trunk forward and backward (front and rear hips out of phase)
        const double front = 0.6 + 0.25*sin(2.0*t);
        const double rear = 0.6 - 0.25*sin(2.0*t);
        VectorXd q(8);
        q << front, -2.0*front, front, -2.0*front, rear, -2.0*rear, rear, -2.0*rear;
        mi.set_joint_target_positions(dog.joint_names, q);

        if (t >= next_save_time)
        {
            std::cout << "t = " << t;
            for (const std::string camera : {"head", "overview"})
            {
                const auto image = mi.get_camera_image(camera);
                write_ppm(camera + ".ppm", image);
                std::cout << "  " << camera << ": " << image.count << " images";
            }
            const auto head = mi.get_camera_image("head");
            std::cout << "  (head: distance at the center " << head.depth.at(head.width*(head.height/2) + head.width/2)
                      << " m)" << std::endl;
            next_save_time += 1.0;
        }

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
