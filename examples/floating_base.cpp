/**
Floating base example.

A generic quadruped stands on its "home" keyframe and rocks its trunk by moving the legs with
position actuators. Every few seconds it gets a push (a change of the trunk twist) that makes it
slide and turn. The example shows how to:
  - list the floating bases and the joints of the legs (get_floating_base_names, get_joint_names(true))
  - start from a keyframe (reset_simulation("home"))
  - follow the robot with the camera (set_camera_tracking)
  - read and write the pose and twist of the trunk (get_base_pose, get_base_twist, set_base_twist)
  - read the pose of any body (get_body_pose)

Run it from the directory that contains quadruped.xml (the build copies it next to the executable).
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
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

    DQ_MujocoInterface mi;
    mi.load_model("quadruped.xml");

    const std::string base = mi.get_floating_base_names().at(0);  // "trunk"
    // Only the hinge joints of the legs (the free joint of the trunk is excluded):
    // FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
    const std::vector<std::string> jointnames = mi.get_joint_names(true);
    const std::vector<std::string> calves = {"FL_calf", "FR_calf", "RL_calf", "RR_calf"};

    mi.reset_simulation("home");
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
