/**
Joint space control example.

Moves every joint of the generic manipulator with a sinusoidal reference around a home
configuration, using MuJoCo's position actuators, and prints the joint positions,
velocities and actuator torques.

Run it from the directory that contains manipulator.xml (the build copies it next to the executable).
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
    mi.load_model("manipulator.xml");

    //mi.set_wireframe_view(true);
    //mi.set_joint_view(true);
    //mi.set_contact_view(true);
    //mi.set_center_of_mass_view(true);

    const std::vector<std::string> jointnames = mi.get_joint_names();
    const int n = jointnames.size();

    // Start from a bent configuration instead of the arm pointing straight up
    VectorXd q_home(n);
    q_home << 0.0, 0.5, 1.2, 0.0, 1.0, 0.0;
    mi.set_joint_positions(jointnames, q_home);

    while (!mi.should_close_window() && !kill_this_process)
    {
        // Simulate 1/60 s per rendered frame, so the simulation runs in real time with v-sync
        double simstart = mi.get_simulation_time();
        while (mi.get_simulation_time() - simstart < 1.0/60.0)
        {
            const double t = mi.get_simulation_time();
            VectorXd q_ref = q_home + VectorXd::Constant(n, 0.3 * sin(2.0 * t));
            mi.set_joint_target_positions(jointnames, q_ref);
            mi.trigger_next_simulation_step();
        }

        std::cout << "--------------------------------------" << std::endl;
        std::cout << "t:     " << mi.get_simulation_time() << " s" << std::endl;
        std::cout << "q:     " << mi.get_joint_positions(jointnames).transpose() << std::endl;
        std::cout << "q_dot: " << mi.get_joint_velocities(jointnames).transpose() << std::endl;
        std::cout << "tau:   " << mi.get_joint_forces(jointnames).transpose() << std::endl;

        mi.render();
    }
    return 0;
}
