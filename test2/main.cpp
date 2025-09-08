#include "mujoco/mujoco.h"
#include <iostream>
#include <thread>
#include <iostream>
#include <memory>
#include "dqrobotics/interfaces/mujoco/DQ_MujocoInterface.h"

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}


int main() {

    auto mi = DQ_MujocoInterface();
    mi.load_model("scene_arm.xml");
    while(!mi.should_close_window() && !kill_this_process)
    {
        std::vector<double> computation_times;
        double simstart = mi.get_simulation_time();
        while( mi.get_simulation_time() - simstart < 1.0/60.0 )
        {
            auto start = std::chrono::high_resolution_clock::now();
            mi.trigger_next_simulation_step();
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            computation_times.push_back(duration.count());
        }
        VectorXd computation_times_vec = Eigen::Map<Eigen::VectorXd>(computation_times.data(), computation_times.size());
        std::cout<<"Mean computation times: "<<computation_times_vec.mean()<<std::endl;
        //std::cout<<"q: "<<mi.get_joint_positions({"arm_sh0"}).transpose()<<std::endl;
        VectorXd q_cmd = VectorXd::Zero(1);
        q_cmd << 0.1 * sin(8.0 * mi.get_simulation_time());
        if (mi.get_simulation_time() > 10)
            mi.set_joint_target_positions({"arm_sh0"}, q_cmd);
        mi.render();
    }
    return 0;
}
