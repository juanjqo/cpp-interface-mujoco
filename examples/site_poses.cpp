/**
Site poses example.

Sites are frames with no physics. This example:
  - moves the free-standing "target" site along a circle while rotating it, with set_site_pose()
  - moves the arm with its position actuators
  - reads the pose of the "end_effector" site, attached to the last link, with get_site_pose()
and prints both poses and the distance between them.

Run it from the directory that contains manipulator.xml (the build copies it next to the executable).
*/

#include "dqrobotics/interfaces/mujoco/DQ_MujocoInterface.h"
#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>

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
    mi.load_model("manipulator.xml");

    const std::vector<std::string> jointnames = mi.get_joint_names();
    const int n = jointnames.size();

    VectorXd q_home(n);
    q_home << 0.0, 0.5, 1.2, 0.0, 1.0, 0.0;
    mi.set_joint_positions(jointnames, q_home);

    while (!mi.should_close_window() && !kill_this_process)
    {
        double simstart = mi.get_simulation_time();
        while (mi.get_simulation_time() - simstart < 1.0/60.0)
        {
            const double t = mi.get_simulation_time();

            // Target: a circle of radius 0.2 m at height 0.4 m, rotating about a moving axis
            DQ axis = (i_*cos(t) + j_*sin(t)).normalize();
            DQ r = cos(M_PI/8) + axis*sin(M_PI/8);
            DQ p = 0.4*i_ + 0.2*cos(t)*j_ + (0.4 + 0.2*sin(t))*k_;
            mi.set_site_pose("target", r + 0.5*E_*p*r);

            // Arm: sweep the base joint back and forth
            VectorXd q_ref = q_home;
            q_ref(0) = 0.8 * sin(0.5 * t);
            mi.set_joint_target_positions(jointnames, q_ref);

            mi.trigger_next_simulation_step();
        }

        DQ x_target = mi.get_site_pose("target");
        DQ x_effector = mi.get_site_pose("end_effector");
        VectorXd p_target = x_target.translation().vec3();
        VectorXd p_effector = x_effector.translation().vec3();

        std::cout << "--------------------------------------" << std::endl;
        std::cout << "target:       " << p_target.transpose() << std::endl;
        std::cout << "end effector: " << p_effector.transpose() << std::endl;
        std::cout << "distance:     " << (p_target - p_effector).norm() << " m" << std::endl;

        mi.render();
    }
    return 0;
}
